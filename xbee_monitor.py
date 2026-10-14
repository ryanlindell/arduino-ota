#!/usr/bin/env python3
"""
xbee_monitor.py - Print the text a remote Arduino sends through its XBee.

A wireless Serial Monitor: the remote XBee (transparent mode) radios whatever
the sketch prints, and it arrives here at the coordinator as Receive Packet
frames. This reuses the frame handling from xbee_ota.py.

Example:
  python xbee_monitor.py --port COM16 --dest64 0013A2004174A009
  python xbee_monitor.py --port COM16 --dest64 0013A2004174A009 --log accel.csv
"""

import argparse
import sys
import time

import serial

from xbee_ota import XBeeLink


def main() -> int:
    p = argparse.ArgumentParser(description="Print serial output from a remote XBee")
    p.add_argument("--port", required=True, help="local XBee port, e.g. COM16")
    p.add_argument("--dest64", required=True, help="remote XBee SH+SL to listen to")
    p.add_argument("--baud", type=int, default=115200, help="local XBee BD (default 115200)")
    p.add_argument("--ap", type=int, choices=[1, 2], default=2, help="local XBee AP setting")
    p.add_argument("--log", help="also append each line, with a timestamp, to this file")
    args = p.parse_args()

    target64 = bytes.fromhex(args.dest64.replace(":", "").replace(" ", ""))
    if len(target64) != 8:
        print("--dest64 must be 16 hex digits (SH followed by SL)", file=sys.stderr)
        return 1

    try:
        link = XBeeLink(args.port, args.baud, args.ap == 2, target64)
    except serial.SerialException as e:
        print(f"Could not open {args.port}: {e}", file=sys.stderr)
        return 1

    log = open(args.log, "a", encoding="ascii") if args.log else None
    print("Listening... Ctrl+C to stop.")
    try:
        link.ser.reset_input_buffer()
        while True:
            link.drain(0.1)
            link.inbox.clear()      # status frames etc. that nobody is waiting for
            # A line can be split across radio packets, so only take whole lines.
            while b"\n" in link.rx_stream:
                raw, _, rest = bytes(link.rx_stream).partition(b"\n")
                link.rx_stream[:] = rest
                line = raw.decode("ascii", errors="replace").strip()
                print(line)
                if log:
                    log.write(f"{time.time():.3f},{line}\n")
                    log.flush()
    except KeyboardInterrupt:
        return 0
    finally:
        if log:
            log.close()
        link.close()


if __name__ == "__main__":
    sys.exit(main())
