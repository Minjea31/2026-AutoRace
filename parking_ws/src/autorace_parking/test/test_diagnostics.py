"""Operator diagnostics must explain a stopped vehicle without changing APIs."""
import json
import math
from pathlib import Path

import pytest

from autorace_parking.core import Gap, Scan, Settings
from autorace_parking.control import build_plan_report
from autorace_parking.diagnostics import operator_reason, reason_text, state_text


def clear_scan():
    return Scan([10.]*361, -math.pi, math.pi/180, .01, 12., (0., 0., 0.))


def test_no_gap_and_obstacle_at_start_have_different_operator_reasons():
    report = build_plan_report(clear_scan(), Settings())
    assert report['diagnostics']['reason_code'] == 'no_gap'
    text = operator_reason('PLANNING', 'No visible reachable gap/path', report['diagnostics'])
    assert '빈 칸 후보를 찾지 못했습니다' in text
    scan = clear_scan(); scan.points = [(.1, 0.)]
    report = build_plan_report(scan, Settings(), Gap(1., -.6, -math.pi/2, .8, 0, 1))
    assert report['path'] == []
    assert report['diagnostics']['reason_code'] == 'start_blocked'
    assert report['diagnostics']['blocked_by'] == 'obstacle'
    text = operator_reason('PLANNING', 'No visible reachable gap/path', report['diagnostics'])
    assert '출발 위치' in text and '장애물' in text


def test_hidden_space_at_start_is_reported_without_being_treated_as_free():
    scan = Scan([0.]*361, -math.pi, math.pi/180, .01, 12., (0., 0., 0.))
    cfg = Settings(initial_clear_radius=0.)
    report = build_plan_report(scan, cfg, Gap(1., -.6, -math.pi/2, .8, 0, 1))
    assert not report['path']
    assert report['diagnostics']['blocked_by'] == 'unknown_space'
    assert '관측되지 않은 공간' in operator_reason('PLANNING', 'No visible reachable gap/path', report['diagnostics'])


@pytest.mark.parametrize('sample', range(6))
def test_real_scan_report_carries_filtered_candidate_count_and_matching_reason(sample):
    import yaml
    from dataclasses import fields
    package = Path(__file__).resolve().parents[1]
    cfg_data = yaml.safe_load((package/'config/parking_real.yaml').read_text())['parking_node']['ros__parameters']
    cfg = Settings(**{f.name: cfg_data[f.name] for f in fields(Settings) if f.name in cfg_data})
    cfg = __import__('dataclasses').replace(cfg,automatic_target_selection=False,slot_map_enabled=False)
    data = json.loads((package/'test/data/limo_pro_stationary_scans.json').read_text())['scans'][sample]
    report = build_plan_report(Scan(**data), cfg)
    details = report['diagnostics']
    assert details['ignored_small_clusters'] > 0
    # The old wider aligned bays exceed 70 cm; narrower unaligned pairs are rejected.
    assert details['candidates'] == 0
    assert not report['path'] and not report['final']
    assert details['reason_code'] == 'no_gap'
    assert details['recovery_no_target']
    assert details['assume_unobserved_free']
    assert details['path_poses'] == len(report['path'])


@pytest.mark.parametrize(('reason', 'expected'), [
    ('Only 5 scan points', '5개'),
    ('Only 9 reference surface points', '기준 면'),
    ('Invalid scan interval: 0.700s', '0.700초'),
    ('Insufficient overlap: 5/120, dt=0.100s', '120개 중 5개'),
    ('Scan matching failed: Waiting for EKF pose at scan acquisition time; retry required', '라이다 측정 시각'),
    ('Fusion inputs not ready: Invalid wheel frame/timestamp/velocity', '휠 센서'),
    ('EKF only: Insufficient reference surfaces', '기준 면'),
    ('Planner result expired before acceptance; check planning load', '너무 늦게'),
])
def test_dynamic_and_nested_reasons_are_korean(reason, expected):
    assert expected in reason_text(reason)
    assert reason not in reason_text(reason)


def test_stop_states_explain_why_zero_speed_is_intentional():
    for state in ('WAITING_SIGN', 'REOBSERVE', 'REPLAN', 'CONFIRMING', 'PARKED', 'EXIT_PATH_ERROR', 'EXITED'):
        assert operator_reason(state)
        assert state_text(state) != state
