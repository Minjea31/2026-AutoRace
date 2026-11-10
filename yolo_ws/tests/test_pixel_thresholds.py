"""Threshold boundaries and OCR overrides must use original-image pixel area."""
import ast
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock
import numpy as np
import pytest

SOURCE = Path(__file__).resolve().parents[1] / 'src/yolo_detect.py'
tree = ast.parse(SOURCE.read_text())
selected = [n for n in tree.body if
    isinstance(n, ast.Assign) and any(isinstance(t, ast.Name) and t.id == 'CLASS_THRESHOLDS' for t in n.targets)
    or isinstance(n, ast.FunctionDef) and n.name == 'filter_detections']
klass = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'YoloDetect')
method = next(n for n in klass.body if isinstance(n, ast.FunctionDef) and n.name == 'classify_detections')
selected.append(ast.ClassDef(name='Classifier', bases=[], keywords=[], body=[method], decorator_list=[]))
ns = {'np': np, 'LR_TOPIC': '/yolo/LR_sign'}
exec(compile(ast.fix_missing_locations(ast.Module(body=selected, type_ignores=[])), str(SOURCE), 'exec'), ns)
EXPECTED = {'slow': 3200, 'bump': 3200, 'parking': 3200, 'crosswalk': 3200, 'left': 3400, 'right': 3400}

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

@pytest.mark.parametrize('letter,name', [('A', 'left'), ('B', 'right')])
@pytest.mark.parametrize('area', [3399, 3400, 3401])
def test_ocr_direction_cannot_bypass_pixel_threshold(letter, name, area):
    classifier = ns['Classifier']()
    classifier.names = {0: 'parking'}
    classifier.get_logger = Mock(return_value=Mock())
    classifier.direction_ocr = SimpleNamespace(read=lambda image: SimpleNamespace(letter=letter, score=.99))
    detected, _, _ = classifier.classify_detections([[0, 0, area / 50., 50, .1, 0]], np.zeros((100,100,3), dtype=np.uint8))
    assert (name in detected) == (area >= 3400)


def test_default_input_uses_compressed_camera_messages():
    t = ast.parse(SOURCE.read_text())
    topic = next(n.value.value for n in t.body if isinstance(n, ast.Assign)
                 and any(isinstance(x, ast.Name) and x.id == 'IMAGE_TOPIC' for x in n.targets))
    assert topic == '/camera/color/image_raw/compressed'
    transport = next(n.args[1].value for n in ast.walk(t) if isinstance(n, ast.Call)
                     and isinstance(n.func, ast.Attribute) and n.func.attr == 'declare_parameter'
                     and len(n.args) >= 2 and isinstance(n.args[0], ast.Constant) and n.args[0].value == 'image_transport')
    assert transport == 'compressed'
