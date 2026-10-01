#!/usr/bin/env python3
"""Passive CRSF channel discovery. Never transmits or accesses motor control."""
import argparse
import json
import time


def crc8(data):
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ (0xD5 if crc & 0x80 else 0)) & 255
    return crc


class Parser:
    def __init__(self):
        self.buffer = bytearray()
        self.bad_crc = 0

    def feed(self, data):
        self.buffer.extend(data)
        frames = []
        while len(self.buffer) >= 4:
            size = self.buffer[1]
            if not 2 <= size <= 62:
                del self.buffer[0]
                continue
            if len(self.buffer) < size + 2:
                break
            if crc8(self.buffer[2:size + 1]) != self.buffer[size + 1]:
                self.bad_crc += 1
                del self.buffer[0]
                continue
            frames.append((self.buffer[2], bytes(self.buffer[3:size + 1])))
            del self.buffer[:size + 2]
        return frames


def channels(payload):
    if len(payload) != 22:
        raise ValueError('RC channels payload must be 22 bytes')
    packed = int.from_bytes(payload, 'little')
    return [(packed >> (11 * i)) & 2047 for i in range(16)]


def main():
    import serial
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--port', default='/dev/ttyAMA4')
    ap.add_argument('--seconds', type=float, default=10)
    ap.add_argument('--baud', type=int, default=420000)
    args = ap.parse_args()
    parser = Parser()
    counts = {}
    last = low = high = None
    link = None
    received = 0
    end = time.monotonic() + args.seconds
    with serial.Serial(args.port, args.baud, timeout=.1, exclusive=True) as port:
        while time.monotonic() < end:
            data = port.read(port.in_waiting or 1)
            received += len(data)
            for kind, payload in parser.feed(data):
                key = hex(kind)
                counts[key] = counts.get(key, 0) + 1
                if kind == 0x16 and len(payload) == 22:
                    last = channels(payload)
                    low = last[:] if low is None else list(map(min, low, last))
                    high = last[:] if high is None else list(map(max, high, last))
                elif kind == 0x14 and len(payload) == 10:
                    link = {'rssi1_dbm': -payload[0], 'lq': payload[2], 'rf_mode': payload[5]}
    print(json.dumps(dict(bytes=received, frames=counts, bad_crc=parser.bad_crc,
                          channels=last, minimum=low, maximum=high, link=link)))


if __name__ == '__main__':
    main()
