"""Threshold boundaries must use original-image pixel area; A/B comes from the YOLO class."""
import ast
from pathlib import Path
import numpy as np
import pytest

SOURCE = Path(__file__).resolve().parents[1] / 'src/yolo_detect.py'
tree = ast.parse(SOURCE.read_text())
selected = [n for n in tree.body if
    isinstance(n, ast.Assign) and any(isinstance(t, ast.Name) and t.id in {'CLASS_THRESHOLDS', 'SIGNAL_CLASS_IDS'} for t in n.targets)
    or isinstance(n, ast.FunctionDef) and n.name in {'filter_detections', 'detection_class_name'}]
klass = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'YoloDetect')
method = next(n for n in klass.body if isinstance(n, ast.FunctionDef) and n.name == 'classify_detections')
selected.append(ast.ClassDef(name='Classifier', bases=[], keywords=[], body=[method], decorator_list=[]))
ns = {'np': np, 'LR_TOPIC': '/yolo/LR_sign'}
exec(compile(ast.fix_missing_locations(ast.Module(body=selected, type_ignores=[])), str(SOURCE), 'exec'), ns)
EXPECTED = {'slow': 1500, 'bump': 1800, 'parking': 2000, 'crosswalk': 2000, 'left': 2000, 'right': 2000}

@pytest.mark.parametrize('name', EXPECTED)
@pytest.mark.parametrize('offset', [-1, 0, 1])
def test_pixel_area_boundary_is_independent_of_image_resolution(name, offset):
    area = EXPECTED[name] + offset
    row = [0., 0., 80., area / 80., .99, 0.]
    assert ns['CLASS_THRESHOLDS'][name]['min_area_pixels'] == EXPECTED[name]
    for w, h in [(640, 480), (1280, 720)]:
        assert (name in ns['filter_detections']([row], {0: name}, w, h)) == (offset >= 0)

def test_low_confidence_and_off_image_area_do_not_pass():
    f = ns['filter_detections']
    assert not f([[0, 0, 100, 100, .1, 0]], {0: 'slow'}, 640, 480)
    assert not f([[-100, 0, 10, 100, .99, 0]], {0: 'slow'}, 640, 480)

@pytest.mark.parametrize('name', ['left', 'right'])
@pytest.mark.parametrize('area', [1999, 2000, 2001])
def test_direction_is_the_yolo_class_with_pixel_threshold(name, area):
    classifier = ns['Classifier']()
    classifier.names = {0: name}
    detected, _ = classifier.classify_detections([[0, 0, area / 50., 50, .9, 0]], np.zeros((100, 100, 3), dtype=np.uint8))
    assert (name in detected) == (area >= 2000)
    assert classifier.box_details[0]['eligible'] == (area >= 2000)


def test_default_input_uses_compressed_camera_messages():
    t = ast.parse(SOURCE.read_text())
    topic = next(n.value.value for n in t.body if isinstance(n, ast.Assign)
                 and any(isinstance(x, ast.Name) and x.id == 'IMAGE_TOPIC' for x in n.targets))
    assert topic == '/camera/color/image_raw/compressed'
    transport = next(n.args[1].value for n in ast.walk(t) if isinstance(n, ast.Call)
                     and isinstance(n.func, ast.Attribute) and n.func.attr == 'declare_parameter'
                     and len(n.args) >= 2 and isinstance(n.args[0], ast.Constant) and n.args[0].value == 'image_transport')
    assert transport == 'compressed'
