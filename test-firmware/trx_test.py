#!/usr/bin/env python3
"""
trx_test.py — Loopback test tool for Calypso TRX bridge

Listens for TX bursts from QEMU and optionally sends RX bursts back.

Usage:
  python3 trx_test.py              # Listen only
  python3 trx_test.py --loopback   # Echo TX back as RX
  python3 trx_test.py --inject     # Send periodic RX bursts
"""

import socket
import struct
import sys
import time
import argparse

QEMU_PORT = 4729       # QEMU TRX data port (we send RX here)
LISTEN_PORT = 4829     # We listen for TX here (QEMU sends to port+100)

def parse_tx_burst(data):
    """Parse TX burst from QEMU"""
    if len(data) < 6:
        return None
    tn = data[0]
    fn = struct.unpack('>I', data[1:5])[0]
    pwr = data[5]
    bits = data[6:6+148] if len(data) >= 154 else data[6:]
    return {'tn': tn, 'fn': fn, 'pwr': pwr, 'bits': bits}

def make_rx_burst(tn, fn, rssi=-60, toa=0, bits=None):
    """Create RX burst packet to send to QEMU"""
    if bits is None:
        bits = bytes([128] * 148)  # Erasure / noise
    rssi_u = rssi & 0xFF
    pkt = struct.pack('>BIbh', tn, fn, rssi_u, toa)
    return pkt + bytes(bits[:148]).ljust(148, b'\x80')

def main():
    parser = argparse.ArgumentParser(description='Calypso TRX test tool')
    parser.add_argument('--loopback', action='store_true',
                        help='Echo TX bursts back as RX')
    parser.add_argument('--inject', action='store_true',
                        help='Send periodic RX bursts to QEMU')
    parser.add_argument('--port', type=int, default=LISTEN_PORT,
                        help=f'Listen port (default {LISTEN_PORT})')
    parser.add_argument('--qemu-port', type=int, default=QEMU_PORT,
                        help=f'QEMU RX port (default {QEMU_PORT})')
    args = parser.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(('0.0.0.0', args.port))
    sock.settimeout(0.1)

    print(f"[trx_test] Listening for TX bursts on port {args.port}")
    print(f"[trx_test] Sending RX bursts to 127.0.0.1:{args.qemu_port}")
    if args.loopback:
        print("[trx_test] LOOPBACK mode: echoing TX → RX")
    if args.inject:
        print("[trx_test] INJECT mode: sending periodic RX bursts")

    tx_count = 0
    rx_count = 0
    fn_inject = 0

    try:
        while True:
            # Check for TX burst from QEMU
            try:
                data, addr = sock.recvfrom(512)
                burst = parse_tx_burst(data)
                if burst:
                    tx_count += 1
                    nonzero = sum(1 for b in burst['bits'] if b != 0)
                    if tx_count <= 10 or tx_count % 100 == 0:
                        print(f"[TX #{tx_count}] TN={burst['tn']} "
                              f"FN={burst['fn']} "
                              f"bits={len(burst['bits'])} "
                              f"nonzero={nonzero}")

                    # Loopback: send it back as RX
                    if args.loopback:
                        # Convert hard bits to soft bits
                        soft = bytes([255 if b else 0 for b in burst['bits']])
                        rx_pkt = make_rx_burst(burst['tn'], burst['fn'],
                                               rssi=-60, bits=soft)
                        sock.sendto(rx_pkt, ('127.0.0.1', args.qemu_port))
                        rx_count += 1
            except socket.timeout:
                pass

            # Inject mode: send periodic RX bursts
            if args.inject:
                rx_pkt = make_rx_burst(0, fn_inject, rssi=-60)
                sock.sendto(rx_pkt, ('127.0.0.1', args.qemu_port))
                fn_inject = (fn_inject + 1) % 2715648
                rx_count += 1
                if rx_count % 1000 == 0:
                    print(f"[inject] Sent {rx_count} RX bursts "
                          f"(FN={fn_inject})")
                time.sleep(0.004615)  # ~4.615 ms

    except KeyboardInterrupt:
        print(f"\n[trx_test] Totals: TX={tx_count} RX={rx_count}")

if __name__ == '__main__':
    main()
