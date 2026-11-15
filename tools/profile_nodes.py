#!/usr/bin/env python3
"""Sample real process CPU deltas; 100% is one logical CPU. No ROS imports."""
import argparse
import json
import os
from pathlib import Path
import time


def snapshot():
    result = {}
    for entry in Path('/proc').iterdir():
        if not entry.name.isdecimal():
            continue
        try:
            stat = (entry/'stat').read_text().rsplit(')', 1)[1].split()
            argv = (entry/'cmdline').read_bytes().replace(b'\0', b' ').decode(errors='replace').strip()
            if not argv:
                continue
            result[int(entry.name)] = dict(ticks=int(stat[11])+int(stat[12]),
                start=stat[19], rss_mib=int(stat[21])*os.sysconf('SC_PAGE_SIZE')/1048576,
                threads=int(stat[17]), command=argv)
        except (OSError, ValueError, IndexError):
            continue
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds',type=float,default=20.)
    parser.add_argument('--output',type=Path)
    args=parser.parse_args()
    before=snapshot();start=time.monotonic()
    time.sleep(args.seconds)
    after=snapshot();elapsed=time.monotonic()-start
    rows=[]
    for pid,row in after.items():
        if pid not in before or row['start']!=before[pid]['start']:
            continue
        row['pid']=pid
        row['cpu_percent']=(row['ticks']-before[pid]['ticks'])/os.sysconf('SC_CLK_TCK')/elapsed*100
        rows.append(row)
    rows.sort(key=lambda row:row['cpu_percent'],reverse=True)
    doc=dict(elapsed_s=elapsed,cpu_count=os.cpu_count(),rows=rows)
    if args.output:
        args.output.parent.mkdir(parents=True,exist_ok=True)
        args.output.write_text(json.dumps(doc,indent=2)+'\n')
    for row in rows[:20]:
        print(f"{row['pid']:>7} {row['cpu_percent']:6.1f}% {row['rss_mib']:6.1f} MiB {row['threads']:3} threads {row['command'][:200]}")


if __name__=='__main__':main()
