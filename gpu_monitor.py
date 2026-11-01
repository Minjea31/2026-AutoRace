#!/usr/bin/env python3
"""Readable GPU dashboard for Jetson and NVIDIA desktop GPUs (standard library only)."""
import argparse
import csv
from datetime import datetime
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import time


def positive_seconds(value):
    seconds = int(value)
    if seconds <= 0:
        raise argparse.ArgumentTypeError('갱신 간격은 1 이상의 정수여야 합니다.')
    return seconds


def bar(value):
    try:
        percent = max(0., min(100., float(value)))
    except (ValueError, TypeError):
        return '지원 안 됨'
    filled = round(percent / 5)
    return f'[{"#" * filled}{"-" * (20 - filled)}] {percent:5.1f}%'


def memory_line(label, used, total):
    return f'  {label:<8} {bar(100 * used / total if total else None)}  {used:,} / {total:,} MB'


def jetson_panel(line):
    gpu = re.search(r'GR3D_FREQ\s+(\d+)%', line)
    temperature = re.search(r'\bgpu@([\d.]+)C', line)
    ram = re.search(r'\bRAM (\d+)/(\d+)MB', line)
    swap = re.search(r'\bSWAP (\d+)/(\d+)MB', line)
    cpu = re.search(r'CPU \[([^]]+)\]', line)
    loads = [int(value) for value in re.findall(r'(\d+)%', cpu[1])] if cpu else []
    rows = [
        '  Jetson GPU',
        f'  GPU 사용률  {bar(gpu[1] if gpu else None)}',
        f'  GPU 온도    {float(temperature[1]):.1f} °C' if temperature else '  GPU 온도    지원 안 됨',
        '',
    ]
    if ram:
        rows.append(memory_line('공유 RAM', int(ram[1]), int(ram[2])))
    if swap:
        rows.append(memory_line('SWAP', int(swap[1]), int(swap[2])))
    if loads:
        rows.append(f'  CPU 평균    {bar(sum(loads) / len(loads))}')
    rows += ['', '  공유 RAM은 CPU와 GPU가 함께 사용하는 메모리입니다.']
    return rows


def nvidia_panel():
    query = 'name,utilization.gpu,memory.used,memory.total,temperature.gpu,power.draw'
    output = subprocess.run(
        ['nvidia-smi', f'--query-gpu={query}', '--format=csv,noheader,nounits'],
        capture_output=True, text=True, check=True, timeout=10).stdout
    rows = []
    for index, fields in enumerate(csv.reader(output.splitlines(), skipinitialspace=True)):
        name, load, used, total, temp, power = fields
        rows += [f'  GPU {index}: {name}', f'  GPU 사용률  {bar(load)}',
                 f'  GPU 온도    {temp} °C', f'  전력        {power} W']
        try:
            rows.append(memory_line('GPU 메모리', int(float(used)), int(float(total))))
        except ValueError:
            rows.append('  GPU 메모리  지원 안 됨')
        rows.append('')
    return rows


def main():
    parser = argparse.ArgumentParser(description='GPU 사용률·온도·메모리 터미널 대시보드')
    parser.add_argument('--interval', type=positive_seconds, default=1, metavar='SECONDS')
    parser.add_argument('--once', action='store_true', help='한 번만 표시하고 종료')
    args = parser.parse_args()
    jetson = Path('/etc/nv_tegra_release').is_file() and shutil.which('tegrastats')
    if not jetson and not shutil.which('nvidia-smi'):
        parser.exit(1, '오류: tegrastats 또는 nvidia-smi가 필요합니다.\n')
    interactive = sys.stdout.isatty() and os.environ.get('TERM') != 'dumb' and not args.once
    process = None
    try:
        if jetson:
            process = subprocess.Popen(
                ['tegrastats', '--interval', str(args.interval * 1000)],
                stdout=subprocess.PIPE, text=True)
        if interactive:
            print('\033[?1049h\033[?25l', end='', flush=True)
        while True:
            if process:
                line = process.stdout.readline()
                if not line:
                    raise RuntimeError('tegrastats가 데이터를 보내지 않고 종료했습니다.')
                rows = jetson_panel(line)
            else:
                rows = nvidia_panel()
            header = ['  GPU MONITOR', f'  {datetime.now():%H:%M:%S}  |  {args.interval}초 간격  |  Ctrl+C 종료', '  ' + '─' * 60, '']
            screen = '\n'.join(header + rows) + '\n'
            print(('\033[H' if interactive else '') + screen + ('\033[J' if interactive else ''), end='', flush=True)
            if args.once:
                break
            if not process:
                time.sleep(args.interval)
    except KeyboardInterrupt:
        pass
    except (OSError, RuntimeError, subprocess.SubprocessError) as exc:
        print(f'GPU 모니터 오류: {exc}', file=sys.stderr)
        return 1
    finally:
        if process:
            process.terminate()
            try:
                process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
        if interactive:
            print('\033[?25h\033[?1049l', end='', flush=True)
    return 0


if __name__ == '__main__':
    sys.exit(main())
