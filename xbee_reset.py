#!/usr/bin/env python3
"""
xbee_reset.py - Reset a remote board by toggling its XBee's DIO3 pin.

Sends the same Remote AT Command frames xbee_ota.py uses for its reset step,
without flashing anything. Useful for rebooting a board, or for checking that
the reset circuit works before trying an OTA upload.

The D3 change is not saved with WR, so power-cycling the radio undoes it.

Example:
  python xbee_reset.py --port COM17 --baud 57600 --dest64 0013A200424EDA4D --watch 10
"""

import argparse
import sys
import time

import serial

from xbee_ota import RESET_MODES, XBeeLink, reset_board


def main() -> int:
    p = argparse.ArgumentParser(description="Reset a remote board via its XBee's DIO3")
    p.add_argument("--port", required=True, help="local XBee port, e.g. COM16")
    p.add_argument("--dest64", required=True, help="target XBee SH+SL, e.g. 0013A2004174A009")
    p.add_argument("--baud", type=int, default=115200, help="local XBee BD (default 115200)")
    p.add_argument("--ap", type=int, choices=[1, 2], default=2, help="local XBee AP setting")
    p.add_argument("--reset-mode", choices=sorted(RESET_MODES), default="npn-cap",
                   help="npn-cap = D3 -> capacitor -> transistor (default); "
                        "npn = D3 -> resistor -> transistor; direct = D3 wired to RESET")
    p.add_argument("--pulse", type=float, default=0.05,
                   help="seconds between the two D3 changes (default 0.05)")
    p.add_argument("--watch", type=float, default=0,
                   help="afterwards, print whatever the board sends for this many seconds")
    args = p.parse_args()

    target64 = bytes.fromhex(args.dest64.replace(":", "").replace(" ", ""))
    if len(target64) != 8:
        print("--dest64 must be 16 hex digits (SH followed by SL)", file=sys.stderr)
        return 1

    try:
        link = XBeeLink(args.port, args.baud, args.ap == 2, target64)
    except serial.SerialException as e:
        print(f"Could not open {args.port}: {e}\n"
              "(Is XCTU or the Serial Monitor still holding the port?)", file=sys.stderr)
        return 1

    try:
        link.ser.reset_input_buffer()
        first, second = RESET_MODES[args.reset_mode]
        sent = reset_board(link, args.reset_mode, args.pulse)
        print(f"Reset sent (D3={first} then D3={second}).")

        # Each radio packet from the board is printed on its own line.
        deadline = sent + args.watch
        while time.time() < deadline:
            if link._pump(deadline) and link.rx_stream:
                print(f"  +{time.time() - sent:5.1f}s  {bytes(link.rx_stream).hex(' ')}")
                link.rx_stream.clear()
        return 0
    except (TimeoutError, RuntimeError) as e:
        print(f"Reset failed: {e}", file=sys.stderr)
        return 3
    except KeyboardInterrupt:
        print("\nInterrupted.", file=sys.stderr)
        return 2
    finally:
        link.close()


if __name__ == "__main__":
    sys.exit(main())
