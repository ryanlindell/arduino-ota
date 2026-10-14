#!/usr/bin/env python3
"""
xbee_ota.py - Flash an ATmega328P (Arduino Uno / Optiboot) over an XBee Zigbee link.

How it works
------------
1. The local XBee (plugged into this computer) is in API mode (AP=1 or AP=2).
2. A Remote AT Command frame (0x17) toggles the target XBee's DIO3 pin, which
   resets the ATmega through your reset circuit. The bootloader then listens.
3. This script speaks the bootloader's STK500v1 protocol directly, sending the
   command bytes inside Transmit Request frames (0x10). The remote XBee is in
   transparent mode (AP=0), so it passes those bytes straight to the ATmega's UART.
4. The bootloader's replies come back to us as Receive Packet frames (0x90).

Because one program does the reset AND the upload back to back, the
bootloader's ~1 second listening window is never a problem.

If anything fails partway, just run it again. The upload always starts with a
hardware reset, and the bootloader always runs after a hardware reset, so a
half-written sketch can't lock you out.

Requirements:  pip install pyserial

Example:
  python xbee_ota.py --port COM16 --dest64 0013A2004174A009 --hex blink.ino.hex
"""

import argparse
import sys
import time
from typing import Callable, Dict, List, Optional, Tuple

import serial

# ---------------------------------------------------------------------------
# XBee API constants
# ---------------------------------------------------------------------------
START = 0x7E            # every API frame begins with this byte
ESC = 0x7D              # AP=2 escape marker
XON, XOFF = 0x11, 0x13
NEEDS_ESCAPE = {START, ESC, XON, XOFF}

TX_REQUEST = 0x10       # "send these bytes to that radio"
TX_STATUS = 0x8B        # local radio's report: did the send succeed?
RX_PACKET = 0x90        # data that arrived from another radio
RX_EXPLICIT = 0x91      # same, in the longer format used when the local AO=1
SERIAL_CLUSTER = b"\x00\x11"    # cluster ID the radio uses for plain serial data
REMOTE_AT = 0x17        # "change a setting on that radio"
REMOTE_AT_RESP = 0x97   # remote radio's reply to a Remote AT command

# ---------------------------------------------------------------------------
# STK500v1 (the protocol Optiboot speaks) constants
# ---------------------------------------------------------------------------
STK_INSYNC = 0x14       # every bootloader reply starts with this...
STK_OK = 0x10           # ...and ends with this
CRC_EOP = 0x20          # every command ends with this ("end of packet")

CMD_GET_SYNC = 0x30
CMD_ENTER_PROGMODE = 0x50
CMD_LEAVE_PROGMODE = 0x51
CMD_LOAD_ADDRESS = 0x55
CMD_PROG_PAGE = 0x64
CMD_READ_PAGE = 0x74
CMD_READ_SIGN = 0x75

ATMEGA328P_SIGNATURE = bytes([0x1E, 0x95, 0x0F])


# ===========================================================================
# Layer 1: XBee API framing
# ===========================================================================
class XBeeLink:
    """Sends and receives API frames through the local (coordinator) XBee."""

    def __init__(self, port: str, baud: int, escaped: bool, target64: bytes,
                 verbose: bool = False):
        self.ser = serial.Serial(port, baud, timeout=0.02, write_timeout=2)
        self.escaped = escaped
        self.target64 = target64
        self.verbose = verbose
        self.inbox: List[bytes] = []    # frames read but not yet claimed
        self.rx_stream = bytearray()    # bytes the target's ATmega sent us
        self._frame_id = 0

    def close(self):
        self.ser.close()

    # ---- sending ----------------------------------------------------------
    def _next_id(self) -> int:
        # Cycles 1..255. (0 means "don't send me a response", so we skip it.)
        self._frame_id = self._frame_id % 255 + 1
        return self._frame_id

    def _send(self, frame_data: bytes):
        """Wrap frame_data as: 7E | length | frame_data | checksum."""
        body = len(frame_data).to_bytes(2, "big") + frame_data
        body += bytes([0xFF - (sum(frame_data) & 0xFF)])
        if self.escaped:
            # AP=2: never let 7E/7D/11/13 appear raw after the start byte.
            out = bytearray()
            for b in body:
                if b in NEEDS_ESCAPE:
                    out += bytes([ESC, b ^ 0x20])
                else:
                    out.append(b)
            body = bytes(out)
        self.ser.write(bytes([START]) + body)

    # ---- receiving --------------------------------------------------------
    def _raw_byte(self, deadline: float) -> Optional[int]:
        while time.time() < deadline:
            b = self.ser.read(1)
            if b:
                return b[0]
        return None

    def _byte(self, deadline: float) -> Optional[int]:
        """Read one byte, undoing AP=2 escaping on the fly."""
        b = self._raw_byte(deadline)
        if b is not None and self.escaped and b == ESC:
            b2 = self._raw_byte(deadline)
            return None if b2 is None else b2 ^ 0x20
        return b

    def _read_frame(self, deadline: float) -> Optional[bytes]:
        """Read one complete, checksum-verified frame. None on timeout."""
        while True:
            # Hunt for the start byte. With AP=2 it can only mean "frame start".
            while True:
                b = self._raw_byte(deadline)
                if b is None:
                    return None
                if b == START:
                    break

            # Length bytes are escaped too, so read them through _byte().
            hi, lo = self._byte(deadline), self._byte(deadline)
            if hi is None or lo is None:
                return None
            length = (hi << 8) | lo

            data = bytearray()
            for _ in range(length):
                b = self._byte(deadline)
                if b is None:
                    return None
                data.append(b)

            checksum = self._byte(deadline)
            if checksum is None:
                return None
            if (sum(data) + checksum) & 0xFF == 0xFF:
                return bytes(data)
            if self.verbose:
                print("    (dropped a frame with a bad checksum)")

    def _pump(self, deadline: float) -> bool:
        """Read one frame and file it. Returns False if nothing arrived."""
        frame = self._read_frame(deadline)
        if frame is None:
            return False
        payload = None
        if frame[1:9] == self.target64:
            if frame[0] == RX_PACKET:
                # 0x90 layout: type(1) src64(8) src16(2) options(1) data...
                payload = frame[12:]
            elif frame[0] == RX_EXPLICIT and frame[13:15] == SERIAL_CLUSTER:
                # 0x91 layout: type(1) src64(8) src16(2) src_ep(1) dst_ep(1)
                #              cluster(2) profile(2) options(1) data...
                payload = frame[18:]
        if payload is not None:
            self.rx_stream += payload
            if self.verbose:
                print(f"    rx <- {payload.hex(' ')}")
        else:
            # Keep everything else (TX status, remote AT replies, ...) for
            # whoever is waiting for it, instead of throwing it away.
            self.inbox.append(frame)
        return True

    def wait_frame(self, match: Callable[[bytes], bool],
                   timeout: float) -> Optional[bytes]:
        deadline = time.time() + timeout
        while True:
            for i, frame in enumerate(self.inbox):
                if match(frame):
                    return self.inbox.pop(i)
            if not self._pump(deadline):
                return None

    def drain(self, seconds: float):
        """Absorb any frames that arrive in the next few moments."""
        deadline = time.time() + seconds
        while self._pump(deadline):
            pass

    # ---- high-level operations --------------------------------------------
    def transmit(self, data: bytes, chunk: int, timeout: float = 3.0):
        """Send bytes to the target's ATmega, split to fit the radio's max payload."""
        for i in range(0, len(data), chunk):
            piece = data[i:i + chunk]
            fid = self._next_id()
            # 0x10 layout: type id dest64 dest16 radius options data...
            # dest16 = FFFE means "look up its current 16-bit address for me".
            self._send(bytes([TX_REQUEST, fid]) + self.target64
                       + b"\xFF\xFE" + b"\x00\x00" + piece)
            if self.verbose:
                print(f"    tx -> {piece.hex(' ')}")
            status = self.wait_frame(
                lambda f: f[0] == TX_STATUS and f[1] == fid, timeout)
            if status is None:
                raise TimeoutError("local XBee never reported TX status")
            # 0x8B layout: type id dest16(2) retries delivery discovery
            if status[5] != 0x00:
                raise RuntimeError(
                    f"radio delivery failed (delivery status 0x{status[5]:02X})")

    def remote_at(self, cmd: str, value: bytes = b"", timeout: float = 3.0) -> bytes:
        """Change a setting on the target radio (option 0x02 = apply immediately),
        or read it when no value is given. Returns the value the radio sent back."""
        fid = self._next_id()
        self._send(bytes([REMOTE_AT, fid]) + self.target64 + b"\xFF\xFE"
                   + b"\x02" + cmd.encode("ascii") + value)
        resp = self.wait_frame(
            lambda f: f[0] == REMOTE_AT_RESP and f[1] == fid, timeout)
        if resp is None:
            raise TimeoutError(f"no reply to remote {cmd} (is the box joined?)")
        # 0x97 layout: type id src64(8) src16(2) cmd(2) status ...
        if resp[14] != 0x00:
            raise RuntimeError(f"remote {cmd} rejected (status {resp[14]})")
        return resp[15:]

    def read_stream(self, n: int, timeout: float) -> bytes:
        """Wait until the ATmega has sent us n bytes, then return them."""
        deadline = time.time() + timeout
        while len(self.rx_stream) < n:
            if not self._pump(deadline):
                got = bytes(self.rx_stream).hex(" ") or "nothing"
                raise TimeoutError(f"expected {n} bytes from the board, got: {got}")
        out = bytes(self.rx_stream[:n])
        del self.rx_stream[:n]
        return out


# ===========================================================================
# Layer 2: the STK500v1 bootloader protocol
# ===========================================================================
class Bootloader:
    def __init__(self, link: XBeeLink, chunk: int):
        self.link = link
        self.chunk = chunk

    def command(self, payload: bytes, body_len: int = 0,
                timeout: float = 3.0) -> bytes:
        """Send one command; expect  14 <body_len bytes> 10  back.

        Knowing the exact reply length matters: page data can contain 0x10,
        so we can't just search for the first 0x10 and call it "OK".
        """
        self.link.transmit(payload + bytes([CRC_EOP]), self.chunk)
        resp = self.link.read_stream(body_len + 2, timeout)
        if resp[0] != STK_INSYNC or resp[-1] != STK_OK:
            raise RuntimeError(f"bad bootloader reply: {resp.hex(' ')}")
        return resp[1:-1]

    def sync(self, window_s: float) -> bool:
        """Try to sync until window_s runs out. The bootloader gives up and
        starts the sketch about 1 s after reset, so trying longer is pointless."""
        give_up = time.time() + window_s
        i = 0
        while time.time() < give_up:
            i += 1
            self.link.rx_stream.clear()
            try:
                self.link.transmit(bytes([CMD_GET_SYNC, CRC_EOP]), self.chunk,
                                   timeout=max(0.1, give_up - time.time()))
            except Exception as e:
                print(f"  sync {i}: send failed ({e})")
                continue
            deadline = time.time() + 0.3
            while time.time() < deadline and b"\x14\x10" not in self.link.rx_stream:
                self.link._pump(deadline)
            got = bytes(self.link.rx_stream)
            if b"\x14\x10" in got:
                print(f"  sync {i}: bootloader answered")
                # Earlier attempts may still have replies in flight. Absorb
                # them, then confirm we're aligned with one clean exchange.
                self.link.drain(0.25)
                self.link.rx_stream.clear()
                self.command(bytes([CMD_GET_SYNC]))
                return True
            if got.startswith(b"\x30\x20"):
                print(f"  sync {i}: our bytes came back unchanged. A sketch is"
                      " echoing them, so the reset didn't reach the chip.")
            else:
                print(f"  sync {i}: reply = {got.hex(' ') or 'nothing'}")
        return False

    def signature(self) -> bytes:
        return self.command(bytes([CMD_READ_SIGN]), body_len=3)

    def load_address(self, byte_addr: int):
        word = byte_addr // 2   # STK500 addresses flash in 16-bit words
        self.command(bytes([CMD_LOAD_ADDRESS, word & 0xFF, word >> 8]))

    def write_page(self, addr: int, data: bytes):
        self.load_address(addr)
        n = len(data)
        self.command(bytes([CMD_PROG_PAGE, n >> 8, n & 0xFF]) + b"F" + data,
                     timeout=5.0)

    def read_page(self, addr: int, n: int) -> bytes:
        self.load_address(addr)
        return self.command(bytes([CMD_READ_PAGE, n >> 8, n & 0xFF]) + b"F",
                            body_len=n, timeout=5.0)


# ===========================================================================
# Intel HEX parsing
# ===========================================================================
def load_hex_pages(path: str, page_size: int) -> List[Tuple[int, bytes]]:
    memory: Dict[int, int] = {}
    upper = 0
    with open(path, "r", encoding="ascii") as f:
        for num, line in enumerate(f, 1):
            line = line.strip()
            if not line:
                continue
            if not line.startswith(":"):
                raise ValueError(f"{path} line {num}: not an Intel HEX record")
            raw = bytes.fromhex(line[1:])
            if sum(raw) & 0xFF != 0:
                raise ValueError(f"{path} line {num}: bad checksum")
            count, addr, rtype = raw[0], (raw[1] << 8) | raw[2], raw[3]
            data = raw[4:4 + count]
            if rtype == 0x00:                       # data
                base = (upper << 16) | addr
                for i, b in enumerate(data):
                    memory[base + i] = b
            elif rtype == 0x01:                     # end of file
                break
            elif rtype == 0x04:                     # extended linear address
                upper = (data[0] << 8) | data[1]

    if not memory:
        raise ValueError("HEX file contains no data")

    first = min(memory) // page_size * page_size
    last = max(memory) // page_size * page_size
    pages = []
    for addr in range(first, last + page_size, page_size):
        page = bytes(memory.get(addr + i, 0xFF) for i in range(page_size))
        if any(b != 0xFF for b in page):
            pages.append((addr, page))
    return pages


# ===========================================================================
# Main
# ===========================================================================
# Each mode is two D3 values. The bootloader starts running when the SECOND
# one lands, so that's the moment the ~1 s sync window opens.
#   npn-cap: D3 -> capacitor -> transistor base. Only the rising edge of D3
#            pulses RESET, so go low first (lets the cap discharge), then high.
#   npn:     D3 -> resistor -> transistor base. High holds RESET low.
#   direct:  D3 wired straight to RESET, used open-drain style.
#            Output low (4) = reset, disabled/high-Z (0) = released.
RESET_MODES = {"npn-cap": (4, 5), "npn": (5, 4), "direct": (4, 0)}

BOOT_WINDOW_S = 0.9     # Optiboot's watchdog gives up 1 s after reset


def reset_board(link: XBeeLink, mode: str, pulse_s: float) -> float:
    """Reset the ATmega. Returns the earliest time the bootloader could have
    started, i.e. when we sent the command that triggers it."""
    first, second = RESET_MODES[mode]
    link.remote_at("D3", bytes([first]))
    time.sleep(pulse_s)
    sent = time.time()
    link.remote_at("D3", bytes([second]))
    return sent


BD_CODES = {1200: 0, 2400: 1, 4800: 2, 9600: 3, 19200: 4, 38400: 5,
            57600: 6, 115200: 7}


def enter_upload_mode(link: XBeeLink, boot_baud: int) -> Dict[str, bytes]:
    """Put the target radio in transparent mode at the bootloader's baud rate.
    Returns its previous AP and BD so they can be put back afterwards.

    Nothing is saved with WR, so power-cycling the radio also undoes this."""
    saved = {"AP": link.remote_at("AP"), "BD": link.remote_at("BD")}
    print(f"Target radio was AP={saved['AP'].hex()} BD={saved['BD'].hex()};"
          f" switching to AP=0 at {boot_baud} baud for the upload...")
    link.remote_at("BD", bytes([BD_CODES[boot_baud]]))
    link.remote_at("AP", b"\x00")
    return saved


def restore_radio(link: XBeeLink, saved: Dict[str, bytes]) -> None:
    for attempt in range(1, 4):
        try:
            link.inbox.clear()
            link.remote_at("AP", saved["AP"])
            link.remote_at("BD", saved["BD"])
            print("Target radio's AP and BD restored.")
            return
        except Exception as e:
            print(f"  restore attempt {attempt}/3 failed: {e}", file=sys.stderr)
    print("Could not restore the target radio's AP/BD. Power-cycle it to get"
          " its saved settings back.", file=sys.stderr)


def upload(link: XBeeLink, boot: Bootloader, args, pages) -> None:
    """One complete reset + flash attempt. Raises on any failure."""
    link.inbox.clear()
    link.rx_stream.clear()

    print("Resetting board via remote DIO3...")
    boot_start = reset_board(link, args.reset_mode, pulse_s=0.05)
    used = time.time() - boot_start
    if used > BOOT_WINDOW_S - 0.3:
        raise RuntimeError(f"radio link too slow: the reset command took {used:.1f} s"
                           " to confirm, so the bootloader has already timed out.")

    print("Syncing with bootloader...")
    if not boot.sync(BOOT_WINDOW_S - used):
        raise RuntimeError("bootloader never answered. Check the reset circuit,"
                           " the TX/RX wiring, and that the remote XBee's BD"
                           " matches the bootloader.")

    sig = boot.signature()
    print(f"Chip signature: {sig.hex(' ')}")
    if sig != ATMEGA328P_SIGNATURE and not args.force:
        raise RuntimeError("not an ATmega328P (use --force to override)")

    boot.command(bytes([CMD_ENTER_PROGMODE]))

    for i, (addr, page) in enumerate(pages, 1):
        for attempt in range(1, 4):
            boot.write_page(addr, page)
            if args.no_verify or boot.read_page(addr, len(page)) == page:
                break
            print(f"  page 0x{addr:04X}: verify mismatch, rewriting ({attempt}/3)")
        else:
            raise RuntimeError(f"page 0x{addr:04X} failed verification 3 times")
        print(f"  [{i}/{len(pages)}] 0x{addr:04X}"
              f"{' written' if args.no_verify else ' written + verified'}")

    boot.command(bytes([CMD_LEAVE_PROGMODE]))   # bootloader starts the new sketch


def main() -> int:
    p = argparse.ArgumentParser(description="Flash an Uno over an XBee API link")
    p.add_argument("--port", required=True, help="local XBee port, e.g. COM16")
    p.add_argument("--dest64", required=True, help="target XBee SH+SL, e.g. 0013A2004174A009")
    p.add_argument("--hex", required=True, help="compiled sketch (.ino.hex, NOT with_bootloader)")
    p.add_argument("--baud", type=int, default=115200, help="local XBee BD (default 115200)")
    p.add_argument("--ap", type=int, choices=[1, 2], default=2, help="local XBee AP setting")
    p.add_argument("--boot-baud", type=int, choices=sorted(BD_CODES),
                   help="for a target XBee that normally runs in API mode: switch it to "
                        "transparent mode at this baud (the bootloader's) for the upload, "
                        "then restore its AP and BD")
    p.add_argument("--reset-mode", choices=sorted(RESET_MODES), default="npn-cap",
                   help="npn-cap = D3 -> capacitor -> transistor (default); "
                        "npn = D3 -> resistor -> transistor; direct = D3 wired to RESET")
    p.add_argument("--tries", type=int, default=5,
                   help="full reset + upload attempts before giving up (default 5)")
    p.add_argument("--chunk", type=int, default=64,
                   help="max bytes per radio packet; must be <= the radio's NP (default 64)")
    p.add_argument("--page-size", type=int, default=128, help="flash page size (328P: 128)")
    p.add_argument("--no-verify", action="store_true", help="skip reading pages back")
    p.add_argument("--force", action="store_true", help="flash even if the chip signature is wrong")
    p.add_argument("-v", "--verbose", action="store_true", help="print every byte sent/received")
    args = p.parse_args()

    target64 = bytes.fromhex(args.dest64.replace(":", "").replace(" ", ""))
    if len(target64) != 8:
        print("--dest64 must be 16 hex digits (SH followed by SL)", file=sys.stderr)
        return 1

    pages = load_hex_pages(args.hex, args.page_size)
    total_bytes = sum(len(pg) for _, pg in pages)
    print(f"Loaded {args.hex}: {len(pages)} pages ({total_bytes} bytes)")

    try:
        link = XBeeLink(args.port, args.baud, args.ap == 2, target64, args.verbose)
    except serial.SerialException as e:
        print(f"Could not open {args.port}: {e}\n"
              "(Is XCTU or the Serial Monitor still holding the port?)", file=sys.stderr)
        return 1

    boot = Bootloader(link, args.chunk)
    start = time.time()
    saved = None
    try:
        link.ser.reset_input_buffer()

        if args.boot_baud:
            saved = enter_upload_mode(link, args.boot_baud)

        # Every attempt begins with a hardware reset, so a failed one (usually
        # a slow or dropped radio packet) can simply be started over.
        for attempt in range(1, args.tries + 1):
            try:
                upload(link, boot, args, pages)
                print(f"Done in {time.time() - start:.1f} s.")
                return 0
            except Exception as e:
                print(f"Attempt {attempt}/{args.tries} failed: {e}", file=sys.stderr)
                link.drain(0.5)

        print("\nUpload failed. Safe to retry: the next run starts with a fresh"
              " hardware reset.", file=sys.stderr)
        return 3

    except KeyboardInterrupt:
        print("\nInterrupted.", file=sys.stderr)
        return 2
    finally:
        if saved:
            restore_radio(link, saved)
        link.close()


if __name__ == "__main__":
    sys.exit(main())
