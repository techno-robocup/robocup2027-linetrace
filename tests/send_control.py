#!/usr/bin/env python3
"""Send ControlPackets (matching common/proto.h) to the collector for testing."""
import socket
import struct
import sys
import time

MAGIC = 0x52435450
VERSION = 1
FLAG_RECORDING = 1


def crc16(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if (crc & 0x8000) else (crc << 1) & 0xFFFF
    return crc


def packet(seq: int, left: int, right: int, flags: int) -> bytes:
    ts = int(time.time() * 1000)
    body = struct.pack(">IHHIQhh", MAGIC, VERSION, flags, seq, ts, left, right)
    body += struct.pack(">6h", 0, 0, 0, 0, 0, 0)
    return body + struct.pack(">H", crc16(body))


def main():
    host = sys.argv[1] if len(sys.argv) > 1 else "127.0.0.1"
    port = int(sys.argv[2]) if len(sys.argv) > 2 else 9201
    secs = float(sys.argv[3]) if len(sys.argv) > 3 else 3.0
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    t0 = time.time()
    seq = 0
    while time.time() - t0 < secs:
        # sweep a gentle turn so left/right vary (range -8000..8000, neutral 0)
        phase = (time.time() - t0)
        left = 2000 + int(800 * (phase % 1))
        right = 2000 - int(800 * (phase % 1))
        sock.sendto(packet(seq, left, right, FLAG_RECORDING), (host, port))
        seq += 1
        time.sleep(0.02)  # 50 Hz
    print(f"sent {seq} control packets")


if __name__ == "__main__":
    main()
