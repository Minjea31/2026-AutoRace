#!/usr/bin/env python3
"""Offline decode of SDK [recv] logs: triangular 3-byte intensity packets.

Packet format/checksum follows YDLIDAR SDK 1.2.17 YDlidarDriver.cpp.
No serial devices are opened and no commands are sent.
"""
import argparse
import collections
import json
from pathlib import Path
import re


def analyze(path):
    pending = bytearray()
    seconds = collections.defaultdict(lambda: dict(packets=0, points=0, nonzero_distance=0, nonzero_intensity=0))
    checksum_failures = 0
    recv_bytes = 0
    package_index = -1
    pattern = re.compile(r'\[(\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2}(?:\.\d+)?)\]\[debug\] \[recv\] ([0-9A-Fa-f]+)')
    with Path(path).open(errors='replace') as source:
        for line in source:
            match = pattern.search(line)
            if not match or len(match[2]) % 2:
                continue
            stamp, encoded = match.groups()
            chunk = bytes.fromhex(encoded)
            recv_bytes += len(chunk)
            pending.extend(chunk)
            while len(pending) >= 10:
                offset = pending.find(b'\xaa\x55')
                if offset < 0:
                    # Preserve an incomplete first header byte.
                    pending[:] = pending[-1:] if pending[-1] == 0xAA else b''
                    break
                if offset:
                    del pending[:offset]
                if len(pending) < 10:
                    break
                count = pending[3]
                if not count or not pending[4] & 1 or not pending[6] & 1:
                    del pending[0]
                    continue
                size = 10 + 3 * count
                if len(pending) < size:
                    break
                packet = pending[:size]
                checksum = 0x55AA
                for start in (2, 4, 6):
                    checksum ^= int.from_bytes(packet[start:start+2], 'little')
                for start in range(10, size, 3):
                    checksum ^= packet[start]
                    checksum ^= int.from_bytes(packet[start+1:start+3], 'little')
                expected = int.from_bytes(packet[8:10], 'little')
                if checksum != expected:
                    checksum_failures += 1
                    del pending[0]
                    continue
                entry = seconds[stamp]
                # SDK parsePackageNode associates the third normal packet
                # after a ring-start packet with the module health byte.
                if packet[2] & 1:
                    package_index = 0
                elif package_index >= 0:
                    package_index += 1
                if package_index == 3:
                    health = entry.setdefault('module_health_byte_counts', {})
                    key = str(packet[2] >> 1)
                    health[key] = health.get(key, 0) + 1
                entry['packets'] += 1
                entry['points'] += count
                for start in range(10, size, 3):
                    distance = int.from_bytes(packet[start+1:start+3], 'little') & 0xFFFC
                    entry['nonzero_distance'] += int(distance != 0)
                    entry['nonzero_intensity'] += int(packet[start] != 0)
                del pending[:size]
    good_seconds = [s for s, counts in seconds.items() if counts['nonzero_distance']]
    zero_seconds = [s for s, counts in seconds.items() if counts['points'] and not counts['nonzero_distance']]
    return {'input': str(path), 'recv_bytes': recv_bytes,
            'valid_checksum_packets': sum(c['packets'] for c in seconds.values()),
            'checksum_candidate_failures': checksum_failures,
            'seconds_with_nonzero_distances': len(good_seconds),
            'seconds_with_only_zero_distances': len(zero_seconds),
            'first_good_second': good_seconds[0] if good_seconds else None,
            'last_good_second': good_seconds[-1] if good_seconds else None,
            'first_zero_second': zero_seconds[0] if zero_seconds else None,
            'last_zero_second': zero_seconds[-1] if zero_seconds else None,
            'per_second': dict(seconds)}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('log', type=Path)
    args = parser.parse_args()
    report = analyze(args.log)
    output = args.log.with_name('packet_analysis.json')
    output.write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps({key: value for key, value in report.items() if key != 'per_second'}, indent=2))
    print('REPORT', output)


if __name__ == '__main__':
    main()
