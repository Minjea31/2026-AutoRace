#!/usr/bin/env python3
"""headless Panda3D 시뮬레이터 + ROS 2 실제 노드로 자동 주행 테스트.

테스트마다 격리된 ROS_DOMAIN_ID에서
  시뮬레이터(--headless --mission obstacle --map 시나리오) -> 어댑터(/objavoid_v3/cmd만 ON)
  -> [고장 주입기] -> objavoid_v3_node -> 평가기(활성화 대행 + 지표)
를 띄우고 끝나면 모두 종료한다. 실행 전·중에 같은 도메인의 기존 장애물 노드 부재를 기록한다.

  python3 tools/run_sim_tests.py --suite all
  python3 tools/run_sim_tests.py --suite course --only default7_1
결과: results/sim/<suite>.json, results/sim/<suite>.md, 로그 results/sim/logs/
"""
import argparse
import json
import os
import signal
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
WS = os.path.dirname(HERE)
ROOT = os.path.dirname(WS)
SC = os.path.join(WS, 'scenarios')
OUT = os.path.join(WS, 'results', 'sim')


def T(name, scen, expect='done', v=0.3, faults=None, timeout=45.0, note='', spawn=None):
    return {'name': name, 'map': os.path.join(SC, scen + '.yaml'), 'expect': expect, 'v': v,
            'faults': faults, 'timeout': timeout, 'note': note, 'spawn': spawn}


SUITES = {
    'single': [T('single_mid', 'single_mid'), T('single_left', 'single_left'),
               T('single_right', 'single_right'), T('single_oblique', 'single_oblique')],
    'multi': [T('multi2', 'multi2'), T('multi3', 'multi3')],
    'zigzag': [T('zigzag4', 'zigzag4'), T('alternating', 'alternating')],
    'course': [T('default7_1', 'default7'), T('default7_2', 'default7'), T('default7_3', 'default7'),
               T('local7_1', 'local7'), T('local7_2', 'local7'), T('local7_3', 'local7'),
               T('orig7', 'orig7'), T('rand7_1', 'rand7_1'), T('rand7_3', 'rand7_3'),
               T('default7_v035', 'default7', v=0.35), T('local7_v040', 'local7', v=0.40),
               T('rand7_2_signside', 'rand7_2', note='마지막 차가 갈림길 표지판 바로 옆(1열): 비현실 경계 사례')],
    'narrow': [T('narrow', 'narrow'), T('blocked', 'blocked', 'stop', timeout=30)],
    'faults': [
        T('scan_drop20', 'default7', faults={'scan_drop': 0.2, 'seed': 1}),
        T('scan_drop20_s2', 'local7', faults={'scan_drop': 0.2, 'seed': 2}),
        T('scan_blackout_1s', 'local7', faults={'scan_blackout': [8.0, 9.0]}),
        T('scan_delay_150ms', 'local7', faults={'scan_delay': 0.15}),
        T('odom_delay_100ms', 'local7', faults={'odom_delay': 0.10}),
        T('odom_blackout_1s', 'local7', faults={'odom_blackout': [8.0, 9.0]}),
        T('scan_lost_permanent', 'local7', 'stop', faults={'scan_blackout': [8.0, -1]}, timeout=25),
    ],
    # 사용자 편집 배치(scenarios/user7.obstacles.yaml: 8열 가운데+아래, 6열 위+가운데 막힘)를 여러 시작 자세에서
    'edited': [T('edit_straight_9.05', 'user7', spawn=(9.05, 1.0, 3.1416)),
               T('edit_yaw0.19_9.05', 'user7', spawn=(9.05, 1.0, 2.95)),
               T('edit_yaw0.04_8.85', 'user7', spawn=(8.85, 0.96, 3.1)),
               T('edit_spawn_8.55', 'user7', 'stop', timeout=25, note='8열 두 차로가 시작점 0.6 m 앞에서 막힘: 기하적으로 통과 불가 -> 정지'),
               T('edit_curve_yaw0.5', 'user7', 'stop', timeout=25, spawn=(9.3, 1.1, 2.6416),
                 note='곡선 위 0.5 rad 비스듬한 시작: 정렬 후 통과 경로 없음 -> 정지(이전에는 엉뚱한 방향으로 직진)')],
    'newobs': [T('newobs_reveal', 'newobs', faults={'hide_box': [5.80, 6.20, 0.45, 0.75], 'hide_until_x': 7.45},
                 note='2번째 차를 차량 x<7.45가 될 때까지 LiDAR에서 숨김(경로 추종 중 새 장애물 등장)')],
}
_by = {t['name']: t for k in ('single', 'multi', 'zigzag', 'course', 'narrow', 'faults', 'newobs', 'edited')
       for t in SUITES[k]}
SUITES['compare'] = [_by[n] for n in ('single_mid', 'multi3', 'zigzag4', 'alternating', 'default7_1', 'local7_1', 'orig7',
                                      'rand7_1', 'rand7_3', 'narrow', 'blocked', 'newobs_reveal', 'scan_drop20')]
SUITES['all'] = [t for k in ('single', 'multi', 'zigzag', 'course', 'narrow', 'faults', 'newobs', 'edited')
                 for t in SUITES[k]]


def last_obstacle_x(map_path):
    import yaml
    m = yaml.safe_load(open(map_path))
    xs = [o['pose'][0] for o in m['obstacles'] if o['name'].startswith('v3_')]
    return min(xs) if xs else None


def node_cpu(pgid, match):
    """프로세스 그룹에서 회피 노드의 누적 CPU 시간/경과 시간 -> 평균 CPU %(코어 1개=100%)."""
    try:
        out = subprocess.run(['ps', '--no-headers', '-o', 'times,etimes,args', '-g', str(pgid)],
                             capture_output=True, text=True).stdout
    except Exception:
        return None
    for ln in out.splitlines():
        parts = ln.split(None, 2)
        if len(parts) == 3 and match in parts[2] and 'ros2 launch' not in parts[2] and 'bash' not in parts[2]:
            cpu_s, el = float(parts[0]), max(float(parts[1]), 1.0)
            return {'cpu_s': cpu_s, 'elapsed_s': el, 'cpu_pct': round(100 * cpu_s / el, 1)}
    return None


def start(cmd, env, log):
    f = open(log, 'w')
    return subprocess.Popen(cmd, env=env, stdout=f, stderr=subprocess.STDOUT, start_new_session=True), f


def stop_all(procs):
    for p, _ in procs:
        try:
            os.killpg(p.pid, signal.SIGINT)
        except ProcessLookupError:
            pass
    t0 = time.time()
    for p, f in procs:
        try:
            p.wait(timeout=max(0.1, 4 - (time.time() - t0)))
        except subprocess.TimeoutExpired:
            try:
                os.killpg(p.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
        f.close()


LANE = True
IMPL = {
    'v3': ('/objavoid_v3/cmd', '/objavoid_v3/state', '/objavoid_v3/done', 'objavoid_v3_node'),
}


def node_command(impl, t, remap):
    if impl != 'v3':
        raise ValueError('Only the production C++ v3 planner is supported')
    return (f'source /opt/ros/humble/setup.bash && source {WS}/install/setup.bash && '
            f'exec ros2 launch autorace_objavoid_v3 objavoid_v3.launch.py use_sim_time:=true v_max:={t["v"]} {remap}')


def run_test(t, domain, logdir, impl='v3'):
    name = t['name']
    os.makedirs(logdir, exist_ok=True)
    base = dict(os.environ, ROS_DOMAIN_ID=str(domain), ROS_LOCALHOST_ONLY='1', PYTHONNOUSERSITE='1')
    clean = {'HOME': os.environ['HOME'], 'PATH': '/usr/bin:/bin', 'PYTHONNOUSERSITE': '1',
             'ROS_DOMAIN_ID': str(domain), 'ROS_LOCALHOST_ONLY': '1', 'ROS_LOG_DIR': os.path.join(logdir, 'ros')}
    sim_env = dict(base, AUTORACE_SIM_DOMAIN_ID=str(domain))
    eval_out = os.path.join(logdir, name + '.eval.json')
    if os.path.exists(eval_out):
        os.remove(eval_out)
    procs = []
    spawn = ['--spawn'] + [str(v) for v in t['spawn']] if t.get('spawn') else []
    procs.append(start([os.path.join(ROOT, 'simul/run.sh'), '--headless', '--mission', 'obstacle', '--map', t['map']] + spawn,
                       sim_env, os.path.join(logdir, name + '.sim.log')))
    time.sleep(4.0)
    cmd_topic, state_topic, done_topic, match = IMPL[impl]
    t = dict(t, node_match=match)
    procs.append(start([os.path.join(ROOT, 'simul/adapter.sh'), '--no-gui', '--topic', cmd_topic],
                       sim_env, os.path.join(logdir, name + '.adapter.log')))
    if impl == 'v3' and LANE:
        # 퓨전 딥 마스크 노드(lane_fusion_ws 소스 그대로) -> V3 노란 실선 입력
        procs.append(start(['bash', '-c', f'ROS_DOMAIN_ID={domain} exec {WS}/scripts/lane_mask.sh'],
                           dict(os.environ, ROS_DOMAIN_ID=str(domain)), os.path.join(logdir, name + '.mask.log')))
    remap = ''
    if t['faults']:
        procs.append(start(['bash', '-c', 'source /opt/ros/humble/setup.bash && exec python3 -u '
                            f'{HERE}/fault_injector.py --faults \'{json.dumps(t["faults"])}\' '
                            '--ros-args -p use_sim_time:=true'], clean, os.path.join(logdir, name + '.inj.log')))
        remap = 'scan_topic:=/objavoid_v3/scan_f odom_topic:=/objavoid_v3/odom_f'
    procs.append(start(['bash', '-c', node_command(impl, t, remap)], clean, os.path.join(logdir, name + '.node.log')))
    procs.append(start(['bash', '-c', f'source {ROOT}/simul/env.sh >/dev/null && exec python -u {HERE}/evaluator.py '
                        f'--map {t["map"]} --out {eval_out} --activate-at 3.0 --cmd-topic {cmd_topic} '
                        f'--state-topic {state_topic} --done-topic {done_topic}'], sim_env,
                       os.path.join(logdir, name + '.eval.log')))
    w0 = time.time()
    lx = last_obstacle_x(t['map'])
    res, end_reason, stop_since = None, 'timeout', None
    try:
        while time.time() - w0 < t['timeout'] + 15:
            time.sleep(0.5)
            if not os.path.exists(eval_out):
                continue
            try:
                res = json.load(open(eval_out))
            except (json.JSONDecodeError, OSError):
                continue
            st = res.get('state') or {}
            if res.get('done'):
                time.sleep(1.5)
                end_reason = 'done'
                break
            if res.get('collisions', 0) > 0:
                time.sleep(1.0)
                end_reason = 'collision'
                break
            if impl == 'team' and lx is not None and res.get('min_x') is not None and res['min_x'] < lx - 0.6:
                time.sleep(1.0)
                end_reason = 'passed'  # 완료 신호가 없는 구현: 마지막 장애물 0.6 m 지나면 종료
                break
            if res.get('sim_t', 0) > t['timeout']:
                end_reason = 'timeout'
                break
            if t['expect'] == 'stop' and st.get('state') == 'STOP' and st.get('replans', 0) + st.get('plan_failures', 0) > 0:
                stop_since = stop_since or res.get('sim_t', 0)
                if res.get('sim_t', 0) - stop_since > 6.0:
                    end_reason = 'stopped'
                    break
            else:
                stop_since = None
    finally:
        cpu = node_cpu(procs[-2][0].pid, t.get('node_match', 'objavoid_v3_node'))
        stop_all(list(reversed(procs)))
    try:
        res = json.load(open(eval_out))
    except Exception:
        res = res or {}
    res['test'], res['expect'], res['end'], res['note'] = name, t['expect'], end_reason, t['note']
    res['node_cpu'] = cpu
    res['faults'], res['v_max'] = t['faults'], t['v']
    lx = last_obstacle_x(t['map'])
    st = res.get('state') or {}
    passed_all = lx is not None and res.get('min_x') is not None and res['min_x'] < lx - 0.32
    res['impl'] = impl
    if t['expect'] == 'done':
        ok = (res.get('done') or impl == 'team') and res.get('collisions', 1) == 0 and passed_all
    else:
        ok = (not res.get('done')) and res.get('collisions', 1) == 0 and not passed_all and (
            st.get('state') == 'STOP' or impl != 'v3')
    res['passed_all_obstacles'] = passed_all
    res['result'] = 'PASS' if ok else 'FAIL'
    res['foreign_avoid_nodes'] = [n for n in res.get('nodes', []) if 'objavoid' in n and 'v3' not in n]
    return res


def summarize(rows, path):
    hdr = ('| 테스트 | 결과 | 종료 | 충돌 | 최소간격 m | 노란선 여유 m | 통과/재계획/경로변경 | 계획 ms 평균/최대 | '
           '|e_y| 평균/최대 m | |e_psi| 최대 rad | 조향 최대 rad | 조향률 최대 rad/s | 명령주기 ms 평균/p99/최대 | 정지 이벤트 | 완료 이유 |')
    lines = [hdr, '|' + '---|' * 15]
    for r in rows:
        st = r.get('state') or {}
        pm, ey, ep, cp = st.get('plan_ms', {}), st.get('ey', {}), st.get('epsi', {}), st.get('cmd_period_ms') or {}
        lines.append(
            f"| {r['test']} | {r['result']} | {r['end']} | {r.get('collisions')} | {r.get('min_gap')} | "
            f"{r.get('yellow_min_margin')} | {st.get('passed')}/{st.get('replans')}/{st.get('route_changes')} | "
            f"{pm.get('mean')}/{pm.get('max')} | {ey.get('mean')}/{ey.get('max')} | {ep.get('max')} | "
            f"{r.get('steer_max')} | {r.get('steer_rate_max')} | {cp.get('mean')}/{cp.get('p99')}/{cp.get('max')} | "
            f"{st.get('stop_events')} | {st.get('done_reason') or st.get('reason')} |")
    with open(path, 'w') as f:
        f.write('\n'.join(lines) + '\n')
    return lines


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--suite', default='all')
    ap.add_argument('--only', nargs='*')
    ap.add_argument('--domain', type=int, default=83)
    ap.add_argument('--tag', default='')
    ap.add_argument('--impl', default='v3', choices=sorted(IMPL))
    ap.add_argument('--no-lane', action='store_true', help='V3를 차선 입력 없이(사전 경계만) 실행')
    a = ap.parse_args()
    global LANE
    LANE = not a.no_lane
    tests = [t for t in SUITES[a.suite] if not a.only or t['name'] in a.only]
    os.makedirs(OUT, exist_ok=True)
    tag = a.suite + (('_' + a.tag) if a.tag else '') + ('' if a.impl == 'v3' else '_' + a.impl)
    rows = []
    for i, t in enumerate(tests):
        w = time.time()
        r = run_test(t, a.domain + (i % 5), os.path.join(OUT, 'logs', tag), a.impl)
        rows.append(r)
        st = r.get('state') or {}
        print(f"[{i + 1}/{len(tests)}] {t['name']:22s} {r['result']} end={r['end']} coll={r.get('collisions')} "
              f"gap={r.get('min_gap')} replans={st.get('replans')} plan_ms={st.get('plan_ms')} "
              f"stops={st.get('stop_events')} wall={time.time() - w:.0f}s", flush=True)
        json.dump(rows, open(os.path.join(OUT, tag + '.json'), 'w'), ensure_ascii=False, indent=1)
    summarize(rows, os.path.join(OUT, tag + '.md'))
    n = sum(r['result'] == 'PASS' for r in rows)
    print(f'{n}/{len(rows)} PASS -> {os.path.join(OUT, tag + ".md")}')


if __name__ == '__main__':
    main()
