"""Traffic lights through the real YOLO filtering, publication and signal state."""
import ast
import importlib.util
import os
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest

SOURCE = Path(os.environ.get('AUTORACE_YOLO_TEST_SOURCE', Path(__file__).resolve().parents[1] / 'src/yolo_detect.py'))
tree = ast.parse(SOURCE.read_text())
constants = {'CLASS_THRESHOLDS', 'PRESENCE_TOPICS', 'LR_TOPIC', 'SIGNAL_TOPIC',
             'SIGNAL_CLASS_IDS', 'CONFIRM_FRAMES', 'MAX_IMAGE_AGE_SEC'}
functions = {'detection_class_name', 'filter_detections', 'bbox_is_published'}
selected = [n for n in tree.body if
            isinstance(n, ast.Assign) and any(isinstance(t, ast.Name) and t.id in constants for t in n.targets)
            or isinstance(n, ast.FunctionDef) and n.name in functions
            or isinstance(n, ast.ClassDef) and n.name == 'DetectionState']
node = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'YoloDetect')
methods = [n for n in node.body if isinstance(n, ast.FunctionDef) and n.name in
           {'classify_detections', 'detect', 'publish_presence', 'log_detections', 'fresh'}]
selected.append(ast.ClassDef(name='Detector', bases=[], keywords=[], body=methods, decorator_list=[]))
clock = SimpleNamespace(monotonic=lambda: 10.)
ns = {'np': np, 'time': clock, 'Bool': lambda **kw: SimpleNamespace(**kw),
      'cv2': SimpleNamespace(error=type('CVError', (Exception,), {})),
      'CvBridgeError': type('CvBridgeError', (Exception,), {})}
exec(compile(ast.fix_missing_locations(ast.Module(body=selected, type_ignores=[])), str(SOURCE), 'exec'), ns)
TOPIC = '/yolo/signal'
NAMES = {0:'bump', 1:'crosswalk', 2:'left', 3:'parking', 4:'right', 5:'slow', 6:'export_label_6', 7:'export_label_7'}


def detector():
    d = ns['Detector']()
    d.names = NAMES
    d.direction_ocr = None
    d.get_logger = Mock(return_value=Mock())
    return d


@pytest.mark.parametrize('ident,name', [(6,'red'), (7,'green')])
@pytest.mark.parametrize('area', [2999, 3000, 3001])
def test_signal_ids_and_exact_original_pixel_area_boundary(ident, name, area):
    row = [0., 0., 50., area/50., .9, float(ident)]
    for width,height in [(640,480),(1280,720)]:
        detected = ns['filter_detections']([row], NAMES, width, height)
        assert (name in detected) == (area >= 3000)
    d = detector()
    result = d.classify_detections([row], np.zeros((480,640,3),np.uint8))
    assert (name in result[0]) == (area >= 3000)
    assert d.box_details[0]['class'] == name


def test_low_confidence_offscreen_and_invalid_ids_do_not_trigger_signal():
    filter_boxes = ns['filter_detections']
    assert not filter_boxes([[0,0,100,100,.59,6]],NAMES,640,480)
    assert filter_boxes([[0,0,60,50,.60,6]],NAMES,640,480) == {'red':.60}
    assert not filter_boxes([[-100,0,20,50,.9,7]],NAMES,640,480)
    assert not filter_boxes([[0,0,100,100,.9,6.5]],NAMES,640,480)
    assert not filter_boxes([[0,0,100,100,.9,0]],{0:'red'},640,480)


def test_red_false_green_true_and_no_signal_on_absence_or_ambiguity():
    state = ns['DetectionState']()
    assert TOPIC not in state.update({'red':.9})
    assert state.update({'red':.9})[TOPIC] is False
    assert TOPIC not in state.update({})
    for _ in range(3):
        assert TOPIC not in state.update({'red':.9,'green':.9})
    assert TOPIC not in state.update({'green':.9})
    assert state.update({'green':.9})[TOPIC] is True
    state.reset()
    assert TOPIC not in state.update({'green':.9})


def test_red_false_is_highlighted_and_logged_as_an_actual_publication():
    published = ns['bbox_is_published']
    assert published({'class':'red','eligible':True}, {TOPIC:False})
    assert not published({'class':'green','eligible':True}, {TOPIC:False})
    d = detector()
    d.log_detections({'red':.9},{TOPIC:False})
    assert 'output_topic=/yolo/signal' in d.get_logger().info.call_args.args[0]
    assert 'published=False' in d.get_logger().info.call_args.args[0]


def test_repeated_or_expired_camera_frames_never_republish_signal():
    d = detector()
    d.state = ns['DetectionState']()
    message = SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=9,nanosec=900_000_000)))
    d.latest_frame = (message, 9.9)
    d.last_stamp = 9_900_000_000
    d.last_processed_time = 9.95
    d.get_clock = lambda: SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=10_000_000_000))
    d.presence_values = dict.fromkeys(ns['PRESENCE_TOPICS'],False)
    d.publishers_by_topic = {topic:Mock() for topic in [*ns['PRESENCE_TOPICS'],TOPIC]}
    d.detect()  # Same acquisition must not extend a red-light stop.
    d.latest_frame = (message, 1.)
    d.detect()  # Expired image must not be treated as red=False.
    d.latest_frame = (None, 0.)
    d.detect()
    d.publishers_by_topic[TOPIC].publish.assert_not_called()


def test_yolo_signal_drives_existing_stop_and_green_release():
    root = Path(__file__).resolve().parents[2]
    spec = importlib.util.spec_from_file_location('signal_core', root/'signal_ws/src/autorace_signal/autorace_signal/core.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    signal = module.SignalState(2,5.,.5)
    state = ns['DetectionState']()
    for index,name in enumerate(['red','red','red','green','green','green']):
        values = state.update({name:.9})
        if TOPIC in values:
            signal.receive(values[TOPIC],index*.1)
        if index == 2:
            assert signal.stopping
    assert not signal.stopping and signal.release_reason == 'green confirmed'
