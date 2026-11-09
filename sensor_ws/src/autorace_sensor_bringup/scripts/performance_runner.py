#!/usr/bin/env python3
"""Apply a process-local CPU/thread budget, then exec the original ROS node."""
import argparse
import os
import sys


def cpu_partition(available, role):
    available = sorted(available)
    if role not in ('localization', 'vision'):
        raise ValueError('Unknown process role')
    if len(available) < 4:
        return available
    return available[-1:] if role == 'localization' else available[:-1]


def configure(role):
    for key in ('OMP_NUM_THREADS', 'OPENBLAS_NUM_THREADS', 'MKL_NUM_THREADS', 'NUMEXPR_NUM_THREADS'):
        os.environ[key] = '1'
    if hasattr(os, 'sched_getaffinity'):
        cpus = cpu_partition(os.sched_getaffinity(0), role)
        try:
            os.sched_setaffinity(0, cpus)
        except OSError as error:
            print(f'CPU profile: affinity unavailable: {error}', file=sys.stderr, flush=True)
    if role == 'vision':
        # Never requires negative nice, root, real-time policy or global CPU settings.
        try:
            current = os.nice(0)
            if current < 5:
                os.nice(5-current)
        except OSError as error:
            print(f'CPU profile: nice unavailable: {error}', file=sys.stderr, flush=True)
    affinity = sorted(os.sched_getaffinity(0)) if hasattr(os, 'sched_getaffinity') else 'system'
    print(f'CPU profile: role={role}, cpus={affinity}, nice={os.nice(0)}, math_threads=1', flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--role', choices=('localization', 'vision'), required=True)
    parser.add_argument('command', nargs=argparse.REMAINDER)
    args = parser.parse_args()
    command = args.command
    if command and command[0] == '--':
        command = command[1:]
    if not command:
        parser.error('A node command is required')
    configure(args.role)
    os.execvpe(command[0], command, os.environ)


if __name__ == '__main__':
    main()
