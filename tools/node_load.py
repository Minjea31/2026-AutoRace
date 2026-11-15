#!/usr/bin/env python3
"""ROS 노드별 CPU/메모리 부하 측정 (Jetson). 표준 라이브러리만 사용, ROS source 불필요.

  python3 tools/node_load.py                 # 10초 측정 후 한 번 출력
  python3 tools/node_load.py -s 5 --watch    # 5초 단위로 계속 갱신 (Ctrl+C 종료)
  python3 tools/node_load.py --all           # ROS 밖 프로세스까지 모두
  python3 tools/node_load.py --csv load.csv  # 노드별 결과를 CSV에 누적 기록
  python3 tools/node_load.py --topics        # 주요 토픽 주기/지연도 함께 (ROS 환경 source 필요)

CPU %는 논리 코어 1개 = 100% 기준입니다 (6코어 = 600%).
"최대 스레드"가 100%에 가까우면 그 노드는 코어를 더 줘도 빨라지지 않는 단일 스레드 병목입니다.
"""
import argparse
from collections import defaultdict
import csv
from datetime import datetime
import os
from pathlib import Path
import re
import subprocess
import sys
import threading
import time

REPO = Path(__file__).resolve().parent.parent
TICKS = os.sysconf('SC_CLK_TCK')
PAGE = os.sysconf('SC_PAGE_SIZE')
CPUS = os.cpu_count()
GPU_LOAD = [Path('/sys/devices/platform/gpu.0/load'),
            Path('/sys/devices/platform/bus@0/17000000.gpu/load'), Path('/sys/devices/gpu.0/load')]
DEFAULT_TOPICS = [
    '/camera/color/image_raw/compressed', '/odom', '/imu', '/scan', '/odometry/filtered',
    '/distance', '/lane_fusion/deep_mask', '/lane_fusion/target', '/mission/lane',
    '/yolo/slow_sign', '/cmd_vel',
]


def read(path):
    try:
        return Path(path).read_text()
    except (OSError, TypeError, ValueError):  # e.g. EAGAIN from an unpowered thermal zone
        return ''


def stat_fields(path):
    """Fields after the command name of /proc/.../stat (state is index 0)."""
    text = read(path)
    return text[text.rfind(')') + 2:].split() if text else None


def snapshot():
    procs = {}
    for entry in os.listdir('/proc'):
        if not entry.isdigit():
            continue
        text = read(f'/proc/{entry}/stat')
        fields = text[text.rfind(')') + 2:].split() if text else None
        argv = read(f'/proc/{entry}/cmdline').split('\0')
        argv = [a for a in argv if a]
        if not fields:
            continue
        threads = {}
        if argv and is_ros(argv):
            for tid in os.listdir(f'/proc/{entry}/task') if os.path.isdir(f'/proc/{entry}/task') else []:
                t = stat_fields(f'/proc/{entry}/task/{tid}/stat')
                if t:
                    threads[tid] = int(t[11]) + int(t[12])
        procs[int(entry)] = dict(
            comm=text[text.find('(') + 1:text.rfind(')')], ppid=int(fields[1]), ticks=int(fields[11]) + int(fields[12]), nice=int(fields[16]),
            nthreads=int(fields[17]), start=fields[19], rss=int(fields[21]) * PAGE, argv=argv,
            threads=threads)
    return procs


def cpu_times():
    rows = {}
    for line in read('/proc/stat').splitlines():
        if re.match(r'cpu\d+ ', line):
            name, *values = line.split()
            values = list(map(int, values))
            rows[name] = (sum(values), values[3] + values[4])  # total, idle + iowait
    return rows


def is_ros(argv):
    text = ' '.join(argv)
    return '--ros-args' in text or '/opt/ros/' in text or str(REPO) in text or 'ros2cli' in text


def executable(argv):
    """(package, executable, label) of a process command line."""
    args = argv[1:] if argv and re.search(r'python[\d.]*$', argv[0]) else argv
    while args and args[0].startswith('-') and args[0] not in ('-c', '-m'):
        args = args[1:]
    if not args:
        return None, None, ' '.join(argv)[:40]
    if args[0] == '-c':
        return None, None, 'ros2 daemon' if 'ros2cli.daemon' in ' '.join(args) else 'python -c'
    if args[0] == '-m':
        return None, args[1] if len(args) > 1 else None, f'python -m {args[1] if len(args) > 1 else ""}'
    path = args[0]
    if path.endswith('/ros2') and len(args) > 1:
        return None, 'ros2', 'ros2 ' + ' '.join(args[1:4])
    match = re.search(r'/lib/([^/]+)/([^/]+)$', path)
    name = Path(path).name
    return (match.group(1) if match else None), name, re.sub(r'\.py$', '', name)


def node_name(argv):
    node = namespace = None
    for arg in argv:
        if arg.startswith('__node:='):
            node = arg.split(':=', 1)[1]
        elif arg.startswith('__ns:='):
            namespace = arg.split(':=', 1)[1]
    if not node:
        return None
    return (namespace.rstrip('/') if namespace else '') + '/' + node


def package_workspace(package):
    for ws in sorted(REPO.iterdir()):
        if (ws / 'install' / package).is_dir():
            return ws.name
    return None


def workspace(pid, procs):
    argv = procs[pid]['argv']
    text = ' '.join(argv)
    match = re.search(re.escape(str(REPO)) + r'/([^/ ]+)/', text)
    if match:
        return match.group(1)
    if 'ros2cli.daemon' in text:
        return 'ROS 데몬'
    # /opt/ros executables: the workspace whose `ros2 launch <pkg>` started them.
    seen, current = 0, pid
    while current in procs and seen < 12:
        launch = re.search(r'ros2 launch (\S+)', ' '.join(procs[current]['argv']))
        if launch:
            return package_workspace(launch.group(1)) or 'ROS'
        current, seen = procs[current]['ppid'], seen + 1
    return 'ROS (launch 밖)' if is_ros(argv) else '시스템'


def under_launch(pid, procs):
    seen, current = 0, procs[pid]['ppid']
    while current in procs and current > 1 and seen < 12:
        if 'launch' in ' '.join(procs[current]['argv']):
            return True
        current, seen = procs[current]['ppid'], seen + 1
    return False


def affinity(pid):
    match = re.search(r'Cpus_allowed_list:\s*(\S+)', read(f'/proc/{pid}/status'))
    return match.group(1) if match else '?'


def system_info():
    mem = dict(re.findall(r'(\w+):\s+(\d+) kB', read('/proc/meminfo')))
    temps = {}
    for zone in Path('/sys/class/thermal').glob('thermal_zone*'):
        kind, value = read(zone / 'type').strip(), read(zone / 'temp').strip()
        if value.lstrip('-').isdigit() and kind.split('-')[0] in ('cpu', 'gpu', 'tj', 'soc0'):
            temps[kind.replace('-thermal', '')] = int(value) / 1000
    freqs = [int(read(f'/sys/devices/system/cpu/cpu{i}/cpufreq/scaling_cur_freq') or 0) // 1000
             for i in range(CPUS)]
    try:
        mode = subprocess.run(['nvpmodel', '-q'], capture_output=True, text=True, timeout=2).stdout
        mode = re.search(r'NV Power Mode:\s*(.+)', mode).group(1).strip()
    except (OSError, subprocess.SubprocessError, AttributeError):
        mode = None
    return dict(load=read('/proc/loadavg').split()[:3], mem_total=int(mem.get('MemTotal', 0)) / 1024,
                mem_avail=int(mem.get('MemAvailable', 0)) / 1024,
                swap_used=(int(mem.get('SwapTotal', 0)) - int(mem.get('SwapFree', 0))) / 1024,
                temps=temps, freqs=freqs, mode=mode)


class TopicProbe:
    """Rate, worst gap and header-stamp age per topic, spun while the CPU window runs."""

    def __init__(self, topics):
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
        rclpy.init()
        self.rclpy, self.node = rclpy, Node('node_load_probe')
        self.topics, self.arrivals, self.ages, self.subs = topics, defaultdict(list), defaultdict(list), []
        deadline = time.monotonic() + 2
        types = {}
        while time.monotonic() < deadline and not all(t in types for t in topics):
            rclpy.spin_once(self.node, timeout_sec=.1)
            types = dict(self.node.get_topic_names_and_types())
        self.missing = [t for t in topics if t not in types]
        for topic in topics:
            if topic not in types:
                continue
            cls = get_message(types[topic][0])
            stamped = hasattr(cls(), 'header')

            def callback(raw, topic=topic, cls=cls, stamped=stamped):
                self.arrivals[topic].append(time.monotonic())
                if stamped:
                    s = deserialize_message(raw, cls).header.stamp
                    now = self.node.get_clock().now().nanoseconds / 1e9
                    self.ages[topic].append(now - (s.sec + s.nanosec * 1e-9))
            self.subs.append(self.node.create_subscription(cls, topic, callback, qos_profile_sensor_data, raw=True))
        self.stop = threading.Event()
        self.thread = threading.Thread(target=self.spin, daemon=True)

    def spin(self):
        while not self.stop.is_set():
            self.rclpy.spin_once(self.node, timeout_sec=.05)

    def window(self):
        self.arrivals.clear()
        self.ages.clear()

    def report(self, seconds):
        lines = [f'{"토픽":40s} {"Hz":>6s} {"최대간격ms":>10s} {"지연ms(중앙/최대)":>18s}']
        for topic in self.topics:
            if topic in self.missing:
                lines.append(f'{topic:40s} {"(없음)":>6s}')
                continue
            t = list(self.arrivals[topic])
            hz = (len(t) - 1) / (t[-1] - t[0]) if len(t) > 1 else len(t) / seconds
            gap = max((b - a for a, b in zip(t, t[1:])), default=0) * 1000
            ages = sorted(self.ages[topic])
            age = f'{ages[len(ages) // 2] * 1000:6.0f} / {ages[-1] * 1000:5.0f}' if ages else ''
            lines.append(f'{topic:40s} {hz:6.1f} {gap:10.0f} {age:>18s}')
        return lines


def gpu_load():
    for path in GPU_LOAD:
        value = read(path).strip()
        if value.isdigit():
            return int(value) / 10
    return None


def measure(seconds, probe):
    before, cores_before = snapshot(), cpu_times()
    if probe:
        probe.window()
    start, gpu = time.monotonic(), []
    while time.monotonic() - start < seconds:  # sample the GPU while waiting
        value = gpu_load()
        if value is not None:
            gpu.append(value)
        time.sleep(min(.5, max(0., seconds - (time.monotonic() - start))))
    elapsed = time.monotonic() - start
    after, cores_after = snapshot(), cpu_times()
    cores = {name: 100 * (1 - (cores_after[name][1] - cores_before[name][1]) /
                          max(1, cores_after[name][0] - cores_before[name][0]))
             for name in cores_after if name in cores_before}
    rows = []
    for pid, p in after.items():
        if pid == os.getpid() or pid not in before or before[pid]['start'] != p['start']:
            continue
        cpu = (p['ticks'] - before[pid]['ticks']) / TICKS / elapsed * 100
        hot = max(((ticks - before[pid]['threads'].get(tid, ticks)) / TICKS / elapsed * 100
                   for tid, ticks in p['threads'].items()), default=None)
        package, exe, label = executable(p['argv']) if p['argv'] else (None, None, f'[{p["comm"]}]')
        rows.append(dict(pid=pid, cpu=cpu, hot=hot, rss=p['rss'] / 1e6, threads=p['nthreads'], nice=p['nice'],
                         ros=is_ros(p['argv']), name=node_name(p['argv']) or label, package=package,
                         ws=workspace(pid, after),
                         launched=exe == 'ros2' or under_launch(pid, after)))
    return elapsed, rows, cores, (sum(gpu) / len(gpu) if gpu else None)


def render(elapsed, rows, cores, gpu, info, args, probe):
    out = []
    total = sum(r['cpu'] for r in rows)
    now = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
    out.append(f'[{now}] 측정 {elapsed:.1f}초 | 전체 CPU {total:.0f}% / {CPUS * 100}% '
               f'({total / CPUS:.0f}%) | load {" ".join(info["load"])}'
               + (f' | 전원 모드 {info["mode"]}' if info['mode'] else ''))
    core_text = '  '.join(f'{name[3:]}:{value:3.0f}%@{freq}MHz' for (name, value), freq
                          in zip(sorted(cores.items(), key=lambda kv: int(kv[0][3:])), info['freqs']))
    out.append(f'코어  {core_text}')
    temps = ' '.join(f'{k} {v:.0f}°C' for k, v in sorted(info['temps'].items()))
    out.append(f'RAM {info["mem_total"] - info["mem_avail"]:.0f}/{info["mem_total"]:.0f} MB | '
               f'swap {info["swap_used"]:.0f} MB | GPU {"-" if gpu is None else f"{gpu:.0f}%"} | {temps}')

    names = defaultdict(int)
    for r in rows:
        if r['ros']:
            names[r['name']] += 1
    shown = [r for r in rows if args.all or (r['ros'] and (r['cpu'] >= args.min_cpu or r['rss'] > 200)) or r['cpu'] >= 5]
    shown.sort(key=lambda r: -r['cpu'])
    out.append('')
    out.append(f'{"CPU%":>6s} {"최대스레드":>8s} {"스레드":>5s} {"RSS MB":>7s} {"nice":>4s}  {"노드/프로세스":34s} {"ws":18s} {"PID":>7s}  비고')
    for r in shown[:args.top]:
        notes = []
        if r['hot'] is not None and r['hot'] >= 85:
            notes.append('단일 스레드 한계')
        if r['ros'] and names[r['name']] > 1:
            notes.append('같은 이름 중복')
        if r['ros'] and not r['launched'] and r['ws'] not in ('ROS 데몬',):
            notes.append('launch 밖에서 실행 중')
        hot = '-' if r['hot'] is None else f'{r["hot"]:.0f}%'
        out.append(f'{r["cpu"]:6.1f} {hot:>8s} {r["threads"]:5d} {r["rss"]:7.0f} {r["nice"]:4d}  '
                   f'{r["name"][:34]:34s} {r["ws"][:18]:18s} {r["pid"]:7d}  {", ".join(notes)}')

    groups = defaultdict(lambda: [0., 0., 0])
    for r in rows:
        g = groups[r['ws']]
        g[0] += r['cpu']
        g[1] += r['rss']
        g[2] += 1
    out.append('')
    out.append(f'{"ws":24s} {"CPU%":>7s} {"머신%":>6s} {"RSS MB":>7s} {"프로세스":>6s}')
    for ws, (cpu, rss, count) in sorted(groups.items(), key=lambda kv: -kv[1][0]):
        if cpu >= .5 or args.all:
            out.append(f'{ws[:24]:24s} {cpu:7.1f} {cpu / CPUS:6.1f} {rss:7.0f} {count:6d}')

    warnings = []
    busy = sum(cores.values()) / max(1, len(cores))
    if busy >= 90:
        warnings.append(f'CPU 포화: 코어 평균 {busy:.0f}% (실행 대기 load {info["load"][0]})')
    for r in shown:
        if r['hot'] is not None and r['hot'] >= 85:
            warnings.append(f'{r["name"]}: 한 스레드가 {r["hot"]:.0f}% → 코어 하나로 처리 한계')
    for name, count in names.items():
        if count > 1:
            warnings.append(f'{name}: 프로세스 {count}개 (이전 실행이 남았는지 확인)')
    leftovers = [r for r in rows if r['ros'] and not r['launched'] and r['ws'] not in ('ROS 데몬',) and r['cpu'] >= .5]
    if leftovers:
        warnings.append('launch 밖 ROS 프로세스: ' + ', '.join(f'{r["name"]}({r["pid"]}, {r["cpu"]:.0f}%)' for r in leftovers))
    hot_temp = [f'{k} {v:.0f}°C' for k, v in info['temps'].items() if v >= 80]
    if hot_temp:
        warnings.append('온도 높음: ' + ', '.join(hot_temp))
    if gpu is not None and busy >= 90 and gpu < 50:
        warnings.append(f'GPU는 여유 ({gpu:.0f}%): CPU 작업을 GPU로 옮길 여지')
    out.append('')
    out.append('진단: ' + ('없음' if not warnings else ''))
    out += [f'  - {w}' for w in warnings]
    if probe:
        out.append('')
        out += probe.report(elapsed)
    return '\n'.join(out)


def write_csv(path, elapsed, rows):
    new = not Path(path).exists()
    with open(path, 'a', newline='') as stream:
        writer = csv.writer(stream)
        if new:
            writer.writerow(['time', 'window_s', 'pid', 'name', 'ws', 'cpu_percent', 'max_thread_percent',
                             'threads', 'rss_mb', 'nice'])
        stamp = datetime.now().isoformat(timespec='seconds')
        for r in sorted(rows, key=lambda r: -r['cpu']):
            if r['ros'] or r['cpu'] >= 1:
                writer.writerow([stamp, f'{elapsed:.1f}', r['pid'], r['name'], r['ws'], f'{r["cpu"]:.1f}',
                                 '' if r['hot'] is None else f'{r["hot"]:.1f}', r['threads'],
                                 f'{r["rss"]:.0f}', r['nice']])


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('-s', '--seconds', type=float, default=10., help='측정 구간 (기본 10초)')
    parser.add_argument('-w', '--watch', action='store_true', help='측정을 반복하며 화면 갱신')
    parser.add_argument('-n', '--top', type=int, default=40, help='표시할 최대 프로세스 수')
    parser.add_argument('--min-cpu', type=float, default=.5, help='이보다 CPU가 낮은 ROS 프로세스는 숨김')
    parser.add_argument('--all', action='store_true', help='모든 프로세스 표시')
    parser.add_argument('--csv', help='노드별 결과를 이 CSV 파일에 누적')
    parser.add_argument('--topics', action='store_true', help='주요 토픽 주기/간격/지연 측정 (ROS source 필요)')
    parser.add_argument('--topic', action='append', default=[], help='추가로 볼 토픽 (반복 지정 가능)')
    args = parser.parse_args()
    if args.seconds <= 0:
        parser.error('--seconds 는 0보다 커야 합니다')

    probe = None
    if args.topics or args.topic:
        try:
            probe = TopicProbe((DEFAULT_TOPICS if args.topics else []) + args.topic)
            probe.thread.start()
        except ImportError:
            sys.exit('토픽 측정에는 ROS 환경이 필요합니다: source /opt/ros/humble/setup.bash')
    try:
        while True:
            elapsed, rows, cores, gpu = measure(args.seconds, probe)
            text = render(elapsed, rows, cores, gpu, system_info(), args, probe)
            if args.watch:
                print('\033[H\033[J', end='')
            print(text, flush=True)
            if args.csv:
                write_csv(args.csv, elapsed, rows)
            if not args.watch:
                break
    except KeyboardInterrupt:
        pass
    finally:
        if probe:
            probe.stop.set()
            probe.thread.join(timeout=1)
            probe.node.destroy_node()
            if probe.rclpy.ok():
                probe.rclpy.shutdown()


if __name__ == '__main__':
    main()
