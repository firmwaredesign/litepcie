#!/usr/bin/env python3
#
# This file is part of LitePCIe.
#
# Copyright (c) 2026 Enjoy-Digital <enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""Test the LitePCIe network interface with the core's Ethernet loopback.

With "ethernet_loopback" enabled in the core (or the User's module looping frames back), every frame
sent on the interface comes back on it. This sends raw Ethernet frames of various sizes and checks
that each one is received back byte-exact, which exercises the whole path: netdev -> MAC TX slot ->
User stream -> MAC RX slot -> IRQ -> netdev.

Usage: sudo python3 liteeth_loopback_test.py [interface] [--count N] [--timeout S]
"""

import argparse
import os
import socket
import struct
import sys
import time

ETH_P_ALL   = 0x0003
ETH_P_TEST  = 0x88b5 # Local Experimental Ethertype 1 (IEEE Std 802).
MIN_FRAME   = 60     # Without FCS.

def build_frame(src_mac, seq, size):
    dst    = b"\x02\x00\x00\x00\x00\x01"
    header = dst + src_mac + struct.pack("!H", ETH_P_TEST)
    payload = struct.pack("!I", seq) + bytes((seq + i) & 0xff for i in range(size - len(header) - 4))
    return header + payload

def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("interface",  nargs="?", default="liteeth0", help="Network interface (default: liteeth0).")
    parser.add_argument("--count",    default=32,  type=int,   help="Number of frames per size (default: 32).")
    parser.add_argument("--timeout",  default=1.0, type=float, help="Receive timeout in seconds (default: 1.0).")
    parser.add_argument("--sizes",    default="60,64,128,512,1024,1514", help="Frame sizes in bytes.")
    args = parser.parse_args()

    if os.geteuid() != 0:
        sys.exit("Run as root (raw sockets).")

    sizes = [int(s) for s in args.sizes.split(",")]
    for size in sizes:
        if size < MIN_FRAME:
            sys.exit(f"Frame size {size} is below the {MIN_FRAME}-byte minimum.")

    sock = socket.socket(socket.AF_PACKET, socket.SOCK_RAW, socket.htons(ETH_P_ALL))
    sock.bind((args.interface, 0))
    sock.settimeout(args.timeout)
    src_mac = sock.getsockname()[4]
    print(f"Interface {args.interface}, MAC {':'.join(f'{b:02x}' for b in src_mac)}")

    errors = 0
    total_bytes, total_time = 0, 0.0
    for size in sizes:
        received, corrupted, lost = 0, 0, 0
        start = time.time()
        for seq in range(args.count):
            frame = build_frame(src_mac, seq, size)
            sock.send(frame)

            # Wait for our frame to come back (skipping any other traffic on the interface).
            deadline = time.time() + args.timeout
            while True:
                try:
                    data = sock.recv(2048)
                except socket.timeout:
                    lost += 1
                    break
                if data[12:14] != struct.pack("!H", ETH_P_TEST):
                    if time.time() > deadline:
                        lost += 1
                        break
                    continue
                received += 1
                if data[:len(frame)] != frame:
                    corrupted += 1
                break
        elapsed = time.time() - start
        total_bytes += received*size
        total_time  += elapsed
        status = "OK" if (received == args.count and corrupted == 0) else "FAILED"
        errors += (received != args.count) + corrupted
        print(f"  {size:5d} bytes: sent {args.count:4d}, received {received:4d}, corrupted {corrupted:4d}, "
              f"lost {lost:4d}  {status}")

    if total_time:
        print(f"\nThroughput (frames looped back): {8*total_bytes/total_time/1e6:.1f} Mbps")
    print("RESULT:", "PASS" if errors == 0 else f"FAIL ({errors} error(s))")
    return 0 if errors == 0 else 1

if __name__ == "__main__":
    sys.exit(main())
