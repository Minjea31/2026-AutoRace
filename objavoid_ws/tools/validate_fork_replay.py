#!/usr/bin/env python3
"""Recompute real bag inputs and measure the NEW planner, never recorded marker output."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import time
import urllib.request
from collections import Counter


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bags', nargs='+', type=Path)
    parser.add_argument('--output-root', type=Path, default=Path(__file__).resolve().parents[1] / '.replay' / 'road_island_validation')
    args = parser.parse_args()
    ws = Path(__file__).resolve().parents[1]
    summaries = []
    for bag in args.bags:
        logdir = args.output_root / bag.name
        logdir.mkdir(parents=True, exist_ok=True)
        env = dict(os.environ, REPLAY_DOMAIN='110', PORT='8087', RATE='1.0', REPLAY_LOOP='false', REPLAY_LOG_DIR=str(logdir))
        samples = Counter()
        max_frames = 0
        last_state = None
        trace = []
        best_mapping = {}
        best_road = {}
        with (logdir / 'replay.log').open('w') as log:
            process = subprocess.Popen(['bash', str(ws / 'scripts/replay_view.sh'), str(bag)], env=env,
                                       stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            started = time.monotonic()
            try:
                while process.poll() is None:
                    if time.monotonic() - started > 180:
                        raise TimeoutError(f'Replay exceeded 180s: {bag}')
                    try:
                        with urllib.request.urlopen('http://127.0.0.1:8087/data', timeout=1) as response:
                            data = json.load(response)
                        max_frames = max(max_frames, data.get('camera_seq', 0))
                        state = data.get('missions', {}).get('/objavoid_v3/state', {}).get('value', {})
                        if isinstance(state, dict):
                            last_state = state
                            samples[state.get('reason', 'waiting')] += 1
                            mapping = state.get('island_mapping', {})
                            samples['map_valid'] += bool(mapping.get('valid'))
                            samples['map_confirmed'] += bool(mapping.get('confirmed'))
                            samples['reject:' + mapping.get('rejection', 'none')] += 1
                            road = state.get('road_mapping', {})
                            samples['road_valid'] += bool(road.get('valid'))
                            if mapping.get('confirmed'):
                                best_mapping = mapping
                            if road.get('valid'):
                                best_road = road
                            trace.append({'state': state.get('state'), 'reason': state.get('reason'), 'road': road,
                                          'island': mapping})
                    except (OSError, ValueError):
                        pass
                    time.sleep(0.25)
            finally:
                if process.poll() is None:
                    os.killpg(process.pid, 15)
                try:
                    process.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, 9)
                    process.wait()
        summary = {'bag': bag.name, 'exit': process.returncode, 'camera_frames': max_frames,
                   'samples': dict(samples), 'best_mapping': best_mapping, 'best_road': best_road, 'last_state': last_state,
                   'note': 'Open-loop recorded poses: does not prove closed-loop driving success.'}
        (logdir / 'trace.json').write_text(json.dumps(trace, ensure_ascii=False))
        (logdir / 'summary.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2))
        summaries.append(summary)
        print(json.dumps(summary, ensure_ascii=False), flush=True)
    all_summaries = [json.loads(p.read_text()) for p in sorted(args.output_root.glob('*/summary.json'))]
    (args.output_root / 'validation.json').write_text(json.dumps(all_summaries, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
