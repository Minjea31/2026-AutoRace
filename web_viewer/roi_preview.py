"""Read-only ROI preview using the mission's actual white-pixel detector."""
import base64
import importlib.util
import math
from pathlib import Path
import sys

import cv2
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
DETECTOR = ROOT / 'crosswalk_ws/src/autorace_crosswalk/autorace_crosswalk/white_roi.py'
spec = importlib.util.spec_from_file_location('_viewer_white_roi', DETECTOR)
_detector = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = _detector
spec.loader.exec_module(_detector)
WhiteROI = _detector.WhiteROI
FIELDS = ('left', 'right', 'top', 'bottom', 'minimum_ratio')
CONFIGS = {
    'crosswalk': ('crosswalk_node', ROOT / 'crosswalk_ws/src/autorace_crosswalk/config/crosswalk.yaml'),
    'slow': ('slow_node', ROOT / 'slow_ws/src/autorace_slow/config/slow.yaml'),
}


def defaults():
    hsv = _detector.load_lane_white(ROOT / 'lane_fusion_ws/src/autorace_lane_fusion/config/real/lane.yaml')
    result = {}
    for mission, (node, path) in CONFIGS.items():
        params = yaml.safe_load(path.read_text())[node]['ros__parameters']
        roi = WhiteROI()
        result[mission] = {f'white_roi_{key}': params.get(f'white_roi_{key}', getattr(roi, key)) for key in FIELDS}
        result[mission].update(white_s_max=hsv[0], white_v_min=hsv[1])
    return {'missions': result, 'sample': 'bag 2026-11-10 06:58:38 / 약 25초',
            'domain_id': __import__('os').environ.get('ROS_DOMAIN_ID', '0')}


def validate(params):
    if not isinstance(params, dict):
        raise ValueError('파라미터는 JSON 객체여야 합니다.')
    keys = [f'white_roi_{key}' for key in FIELDS] + ['white_s_max', 'white_v_min']
    if set(params) != set(keys):
        raise ValueError('ROI 경계·흰색 기준·HSV 값을 모두 입력하세요.')
    if any(isinstance(params[key], bool) or not isinstance(params[key], (float, int))
           or not math.isfinite(params[key]) for key in keys):
        raise ValueError('모든 값은 유효한 숫자여야 합니다.')
    roi = WhiteROI(**{key: params[f'white_roi_{key}'] for key in FIELDS})
    hsv = _detector.validate_white(params['white_s_max'], params['white_v_min'])
    if any(params[key] != int(params[key]) for key in ('white_s_max', 'white_v_min')):
        raise ValueError('HSV 값은 0~255 정수로 입력하세요.')
    return roi, hsv


def render_preview(data, params):
    roi, hsv = validate(params)
    frame = cv2.imdecode(np.frombuffer(data, dtype=np.uint8), cv2.IMREAD_COLOR)
    if frame is None:
        raise ValueError('이미지를 읽을 수 없습니다. JPEG 또는 PNG를 사용하세요.')
    height, width = frame.shape[:2]
    if min(width, height) < 8 or max(width, height) > 4096 or width * height > 8_000_000:
        raise ValueError('이미지는 한 변 8~4096px, 총 800만 화소 이하로 사용하세요.')
    ratio, bounds, mask = roi.measure(frame, *hsv)
    x0, y0, x1, y1 = bounds
    overlay = frame.copy()
    patch = overlay[y0:y1, x0:x1]
    tint = patch.copy()
    tint[mask > 0] = (95, 230, 135)
    overlay[y0:y1, x0:x1] = cv2.addWeighted(patch, .55, tint, .45, 0)
    colour = (95, 230, 135) if ratio >= roi.minimum_ratio else (60, 195, 250)
    cv2.rectangle(overlay, (x0, y0), (x1 - 1, y1 - 1), colour, max(1, width // 400))
    for x, y in ((x0, y0), (x1 - 1, y0), (x0, y1 - 1), (x1 - 1, y1 - 1)):
        cv2.rectangle(overlay, (x - 4, y - 4), (x + 4, y + 4), colour, -1)
    ok, jpeg = cv2.imencode('.jpg', overlay, [cv2.IMWRITE_JPEG_QUALITY, 90])
    if not ok:
        raise ValueError('미리보기 이미지 생성에 실패했습니다.')
    return dict(image='data:image/jpeg;base64,' + base64.b64encode(jpeg).decode(),
                width=width, height=height, bounds=list(bounds),
                roi_width=x1-x0, roi_height=y1-y0, white_pixels=int(np.count_nonzero(mask)),
                roi_pixels=int(mask.size), ratio=ratio, detected=ratio >= roi.minimum_ratio,
                params=dict(params))
