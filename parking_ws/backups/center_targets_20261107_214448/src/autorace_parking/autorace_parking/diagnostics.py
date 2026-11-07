"""Korean operator messages; internal state/reason identifiers remain stable."""
import re


STATES = {
    'WAITING_SIGN': '주차 시작 신호 대기', 'WAITING_SCAN': '라이다 수신 대기',
    'WAITING_LOCALIZATION': '위치 추정 준비 대기', 'TRACKING_LOST': '위치 추정 실패',
    'ROUTE_BLOCKED': '추종 경로 장애물로 정지·재계획 중',
    'PATH_HELD': '진입 경로 유지·정지 중',
    'PLANNING': '주차 경로 계산 중', 'APPROACHING': '주차 구역 접근 중',
    'FRONT_BLOCKED': '정면 장애물 근접으로 정지',
    'PARKING': '주차 진입 중', 'CONFIRMING': '주차 위치와 정지 확인 중',
    'REVERSING': '후진을 포함한 주차 진입 중', 'RECOVERING': '후진 복구 후 재탐색 중',
    'CHANGING_DIRECTION': '전진·후진 전환을 위한 정지 확인 중',
    'RECOVERY_BLOCKED': '후진 복구 경로의 새 장애물로 정지',
    'PARKED': '주차 완료 후 대기', 'EXITING': '기록한 진입 경로를 되짚어 출차 중',
    'EXIT_CONFIRMING': '출차 위치와 정지 확인 중', 'EXITED': '출차 완료',
    'REOBSERVE': '접근 구간 끝 도착·재관측 중', 'REPLAN': '추종 경로 재계산 중',
    'EXIT_PATH_ERROR': '후진 출차 경로 오류',
}

REASONS = {
    '': '',
    'Clock moved backwards': '차량의 시각이 뒤로 바뀌어 주차 미션을 초기화했습니다. 시작 신호를 다시 보내세요.',
    'Front obstacle too close before arrival': '목표 도착 전 정면 장애물이 앞범퍼 가까이에 있어 정지했습니다. 허용된 후진 복구를 확인합니다.',
    'Waiting for a fresh scan': '최신 라이다 스캔을 기다리고 있습니다. /scan 수신과 시각을 확인하세요.',
    'Invalid scan geometry/frame or stale timestamp': '라이다 스캔의 좌표계·각도·거리 설정 또는 측정 시각이 올바르지 않습니다.',
    'Wheel/IMU inputs unavailable or gyro calibration not complete': '휠·관성 센서 입력이 없거나 자이로 정지 보정이 끝나지 않았습니다.',
    'Fusion health unavailable': '센서 융합 상태를 받지 못했습니다. 센서 노드와 /localization/health를 확인하세요.',
    'Fusion health is stale or future-dated': '센서 융합 상태가 오래됐거나 차량과 측정 시각이 맞지 않습니다.',
    'Waiting for EKF and scan': '융합 위치와 라이다 스캔이 함께 준비되기를 기다리고 있습니다.',
    'Waiting for EKF pose at scan acquisition time': '라이다 측정 시각에 대응하는 융합 위치를 기다리고 있습니다.',
    'Waiting for calibrated wheel/IMU EKF odometry': '보정된 휠·관성 센서의 융합 위치를 기다리고 있습니다.',
    'Invalid EKF frame, timestamp, quaternion or covariance': '융합 위치의 좌표계·시각·자세 또는 오차 값이 올바르지 않습니다.',
    'Invalid localization values/covariance': '위치 추정값 또는 추정 오차가 올바르지 않습니다.',
    'Localization covariance exceeds configured limit': '위치 추정 오차가 허용 한계를 넘었습니다. 센서와 보정 상태를 확인하세요.',
    'Localization velocity exceeds physical bound': '위치 추정 속도 또는 회전속도가 차량의 허용 범위를 넘었습니다.',
    'Localization pose jumped; reset parking after checking sensors': '추정 위치가 갑자기 뛰었습니다. 센서를 확인한 뒤 /parking/reset으로 초기화하세요.',
    'EKF odometry is stale or future-dated': '융합 위치가 오래됐거나 측정 시각이 현재 시각보다 앞서 있습니다.',
    'LiDAR detects motion inconsistent with stationary wheel/IMU; reset required': '휠·관성 센서는 정지 상태인데 라이다에서 큰 장면 변화가 감지됐습니다. 주변과 센서를 확인한 뒤 초기화하세요.',
    'Insufficient reference surfaces': '위치 정합에 사용할 기준 면이 부족합니다.',
    'Insufficient keyframe overlap': '기준 스캔과 현재 스캔이 겹치는 영역이 부족합니다.',
    'Degenerate keyframe geometry': '기준 스캔의 형상이 단조로워 이동과 회전을 구분하기 어렵습니다.',
    'Keyframe residual exceeds 2 cm': '라이다 스캔 정합 오차가 2cm를 넘었습니다.',
    'No visible reachable gap/path': '관측된 빈 칸으로 진입할 경로를 찾지 못했습니다.',
    'Planner failure': '경로 계산 중 오류가 발생했습니다. 설정과 입력 데이터를 확인하세요.',
    'Localization time, velocity and covariance bounds must be positive': '위치 추정의 제한 시간·속도·오차 한계 설정은 0보다 커야 합니다.',
    'exit_heading_tolerance must be between zero and pi/2': '출차 방향 허용 오차는 0도보다 크고 90도보다 작아야 합니다.',
    'Planner result expired before acceptance; check planning load': '경로 계산 결과가 너무 늦게 도착해 폐기했습니다. 계산 부하와 센서 지연을 확인하세요.',
    'Invalid wheel frame/timestamp/velocity': '휠 센서의 좌표계·시각 또는 속도 값이 올바르지 않습니다.',
    'Waiting for valid base_link to IMU mounting TF': '차량 본체와 관성 센서 사이의 장착 좌표 변환을 기다리고 있습니다.',
    'Invalid IMU frame/timestamp/gyro': '관성 센서의 좌표계·시각 또는 자이로 값이 올바르지 않습니다.',
    'Invalid initial wheel pose for parking_odom anchor': '주차 기준 좌표계를 만들 초기 휠 위치가 올바르지 않습니다.',
    'Keep stationary for gyro calibration; check fresh wheel/IMU inputs': '자이로 보정이 끝날 때까지 정지해 주세요. 휠·관성 센서의 최신 입력도 확인하세요.',
    'waiting': '라이다 보정 준비 대기',
    'accepted': '라이다 위치 보정 적용',
    'stationary: correction held': '정지 상태이므로 라이다 잡음에 따른 위치 보정 보류',
    'rejected: disagrees with wheel/IMU prediction': '휠·관성 센서 예측과 차이가 커 라이다 보정 제외',
}


def state_text(state):
    return STATES.get(state, '확인되지 않은 상태')


def reason_text(reason):
    if not isinstance(reason, str):
        return '상태 설명의 형식이 올바르지 않습니다. 센서 입력을 확인하세요.'
    if reason in REASONS:
        return REASONS[reason]
    if reason.startswith('Scan matching failed: '):
        detail = reason[len('Scan matching failed: '):].removesuffix('; retry required')
        return '라이다 위치 정합 실패: '+reason_text(detail)+' 회복되지 않으면 /parking/reset으로 초기화하세요.'
    for prefix, label in (('Fusion inputs not ready: ', '센서 융합 준비 대기: '),
                          ('EKF only: ', '라이다 보정을 제외하고 융합 위치 사용: ')):
        if reason.startswith(prefix):
            return label+reason_text(reason[len(prefix):])
    match = re.fullmatch(r'Only (\d+) (scan points|reference surface points)', reason)
    if match:
        kind = '라이다 점' if match[2] == 'scan points' else '기준 면의 점'
        return f'{kind}이 {match[1]}개로 부족합니다. 최소 12개가 필요합니다.'
    match = re.fullmatch(r'Invalid scan interval: ([\d.-]+)s', reason)
    if match:
        return f'라이다 측정 간격이 올바르지 않습니다. 수신 간격: {match[1]}초.'
    match = re.fullmatch(r'Insufficient overlap: (\d+)/(\d+), dt=([\d.]+)s', reason)
    if match:
        return f'스캔 정합에 겹치는 점이 부족합니다. {match[2]}개 중 {match[1]}개, 간격 {match[3]}초.'
    if reason.startswith('Degenerate scan geometry'):
        return '라이다 형상이 단조로워 이동과 회전을 구분하기 어렵습니다.'
    if reason.startswith('ICP limits: '):
        return '라이다 정합 오차 또는 추정 이동·회전량이 허용 범위를 넘었습니다.'
    if reason.startswith('A process in the process pool was terminated'):
        return '경로 계산 작업 프로세스가 종료됐습니다. 주차 노드를 다시 실행하세요.'
    if re.search('[가-힣]', reason):
        return reason
    return '분류되지 않은 오류가 발생했습니다. 센서와 설정을 확인하고 주차 미션을 초기화하세요.'


def planning_text(details):
    if details.get('recovery_limit_reached'):
        return '설정된 후진 복구 횟수 또는 누적 거리 한도를 모두 사용했습니다. 주변 공간을 확인하고 /parking/reset으로 새 시도를 시작하세요.'
    recovery_note = ''
    if details.get('recovery_no_target'):
        recovery_note = ' 주차 목표가 없어 자동 후진하지 않습니다.'
    elif details.get('recovery_waiting_for_front_obstacle'):
        recovery_note = ' 정면 장애물 근접 조건이 아니므로 자동 후진하지 않습니다.'
        if details.get('recovery_on_path_failure'):
            recovery_note = (f" 후진 복구는 정면 근접 또는 목표 옆·후방에서 연속 계획 실패 조건이 필요합니다. "
                             f"실패 {details.get('failed_plans',0)}/{details.get('recovery_failure_count',3)}회.")
    elif details.get('recovery_blocked_by'):
        recovery_note = ' 후진 복구도 장애물 또는 후방 미관측 영역 때문에 생성하지 못했습니다.'
        if details.get('scan_fov_deg', 360.) < 300:
            remaining = details.get('blind_reverse_remaining_m', 0.)*100
            recovery_note += f' 후방은 라이다 미관측 영역입니다. 사전 확인 후진 허용 거리 잔여: {remaining:.0f}cm.'
    code = details.get('reason_code')
    if details.get('partial_path_rejected') and code in ('budget_exceeded','no_observed_path'):
        return '목표까지 이어진 완전한 진입 경로를 기다립니다. 짧은 부분 경로로 주차 제어를 시작하거나 기존 경로를 바꾸지 않습니다.'+recovery_note
    count = details.get('candidates', 0)
    if code == 'front_obstacle':
        return '목표 도착 전 정면 장애물 근접으로 정지했습니다.'+recovery_note
    if code == 'no_gap' and details.get('enclosed_space_detection'):
        return (f"둘러싼 빈 공간을 찾는 중입니다. 사용 가능한 객체 {details.get('enclosure_objects',0)}개, "
                f"검증된 공간 후보 {details.get('enclosure_candidates',0)}개. "
                '서로 다른 객체 3개 이상으로 둘러싸이고 차체가 들어갈 공간이 필요합니다.'+recovery_note)
    if code == 'no_gap':
        labels = {'not_right_side':'오른쪽 아님','object_too_small':'객체 작음','object_too_large':'객체 큼',
                  'gap_too_small':'간격 부족','gap_too_large':'간격 초과','not_aligned':'나란함 불일치',
                  'same_object':'같은 객체','intervening_object':'사이 객체'}
        counts = details.get('pair_filter_counts',{})
        summary = ', '.join(f'{labels.get(k,k)} {v}' for k,v in counts.items() if v)
        checks = details.get('candidate_checks',[])
        blocked = sum(not c.get('accepted',False) for c in checks)
        if blocked:
            summary += (', ' if summary else '')+f'빈 공간 검사 탈락 {blocked}'
        recovery_note = (' 제외 사유: '+summary+'.' if summary else '')+recovery_note
        return ('오른쪽에서 주차 가능한 빈 칸 후보를 찾지 못했습니다. '
                f'관측 점 {details.get("input_points", 0)}개, 장애물 군집 {details.get("clusters", 0)}개. '
                '장애물 크기·간격·주차 열 방향을 확인하세요.'+recovery_note)
    if code == 'start_blocked':
        if details.get('blocked_by') == 'obstacle':
            return '출발 위치의 차량 외곽 또는 여유 공간에 라이다 장애물이 검출돼 경로를 만들지 못했습니다.'
        return '출발 위치의 차량 외곽에 관측되지 않은 공간이 있어 경로를 만들지 못했습니다.'
    if code == 'budget_exceeded':
        return f'빈 칸 후보 {count}개의 경로를 제한 시간 안에 찾지 못했습니다. 계산 부하와 관측 범위를 확인하세요.'+recovery_note
    if code == 'no_observed_path' and details.get('assume_unobserved_free'):
        return (f'공간 후보 {count}개를 찾았지만 검출된 장애물 또는 차량 회전 조건을 만족하는 경로를 만들지 못했습니다.'+recovery_note)
    if code == 'no_observed_path':
        length = details.get('max_observed_prefix_m', 0.)*100
        return (f'빈 칸 후보 {count}개를 찾았지만 장애물·미관측 영역으로 진입 경로를 확보하지 못했습니다. '
                f'확인된 최장 접근 구간: {length:.0f}cm.'+recovery_note)
    return ''


def operator_reason(state, reason='', planning=None, detections=0, required=3):
    if state == 'WAITING_SIGN':
        if reason == 'Clock moved backwards':
            return reason_text(reason)
        return f'주차 시작 신호를 기다리고 있습니다. 연속 감지 {detections}/{required}회.'
    if state == 'PATH_HELD' and (planning or {}).get('continuous_replanning'):
        return '기존 경로 끝에서 정지한 채 연결 가능한 새 경로를 계산하고 있습니다. 표시 경로와 제어권을 유지합니다.'
    if state == 'RECOVERING' and (planning or {}).get('recovery_mode') == 'straight_then_left_reverse':
        return '주차 칸이 차량 오른쪽에 있어 직선 후진 후 좌측 후방 곡선으로 재배치합니다.'
    if reason:
        if reason == 'No visible reachable gap/path':
            return planning_text(planning or {}) or reason_text(reason)
        return reason_text(reason)
    return {
        'WAITING_SCAN': REASONS['Waiting for a fresh scan'],
        'WAITING_LOCALIZATION': REASONS['Waiting for EKF and scan'],
        'PLANNING': '라이다 데이터로 주차 경로를 계산하고 있습니다.',
        'REOBSERVE': '관측된 접근 구간 끝에 도착했습니다. 새 스캔으로 안쪽 경로를 다시 계산합니다.',
        'ROUTE_BLOCKED': '현재 경로의 가까운 구간에 장애물이 검출되어 경로를 유지한 채 새 경로를 계산합니다.',
        'PATH_HELD': '채택한 직선·곡선 경로를 유지한 채 정지했습니다. 경로를 자동 교체하지 않으며 정면 장애물 근접 시 허용된 후진 복구를 확인합니다.',
        'REPLAN': '추종할 경로가 차량 진행 방향과 맞지 않아 정지 후 다시 계산합니다.',
        'CHANGING_DIRECTION': '차량이 멈췄는지 연속 3회 확인한 뒤 전진·후진 방향을 바꿉니다.',
        'RECOVERING': '관측되었거나 사전에 비어 있다고 확인한 허용 범위에서 후진한 뒤 주차 칸과 진입 경로를 다시 탐색합니다.',
        'RECOVERY_BLOCKED': '후진 복구 경로에 새 장애물이 검출돼 정지했습니다. 현재 관측으로 경로를 다시 계산합니다.',
        'EXIT_PATH_ERROR': '기록된 후진 경로를 정상적으로 추종할 수 없습니다. 주변을 확인하고 미션을 초기화하세요.',
        'CONFIRMING': '목표 부근에서 차량이 멈췄는지 연속 관측으로 확인하고 있습니다.',
        'EXIT_CONFIRMING': '후진 출차 끝점에서 차량의 정지와 방향을 확인하고 있습니다.',
        'PARKED': '주차를 완료했습니다. 설정된 대기 시간이 지나면 기록한 경로로 후진합니다.',
        'EXITED': '출차를 완료해 정지 명령을 유지하고 있습니다.',
    }.get(state, '')
