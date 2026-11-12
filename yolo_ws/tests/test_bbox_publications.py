"""BBox appearance follows the real detection publication, including LEFT=False."""
import ast
from pathlib import Path

import cv2
import numpy as np
import pytest

SOURCE = Path(__file__).resolve().parents[1] / 'src/yolo_detect.py'
tree = ast.parse(SOURCE.read_text())
constants = {'PRESENCE_TOPICS', 'LR_TOPIC', 'SIGNAL_TOPIC', 'BBOX_COLOURS', 'UNPUBLISHED_COLOUR'}
functions = {'bbox_is_published', 'draw_bbox_publications'}
nodes = [node for node in tree.body if
         (isinstance(node, ast.Assign) and any(isinstance(t, ast.Name) and t.id in constants for t in node.targets))
         or (isinstance(node, ast.FunctionDef) and node.name in functions)]
namespace = {'cv2': cv2}
exec(compile(ast.Module(body=nodes, type_ignores=[]), str(SOURCE), 'exec'), namespace)


def box(name='slow', eligible=True):
    return {'class': name, 'eligible': eligible, 'xyxy': (20, 20, 70, 40),
            'width': 50, 'height': 20, 'area': 1000}


@pytest.mark.parametrize('values', [{}, {'/yolo/slow_sign': False}])
def test_pending_or_negative_presence_is_gray(values):
    image = np.zeros((100, 260, 3), np.uint8)
    preview = namespace['draw_bbox_publications'](image, [box()], values)
    assert tuple(preview[20, 20]) == namespace['UNPUBLISHED_COLOUR']
    assert not np.any(image)
    assert np.any(preview[41:65])  # Label is beneath the box, not in a separate footer.


def test_published_box_is_class_coloured_but_rejected_box_stays_gray():
    image = np.zeros((100, 260, 3), np.uint8)
    render = namespace['draw_bbox_publications']
    values = {'/yolo/slow_sign': True}
    assert tuple(render(image, [box()], values)[20, 20]) == namespace['BBOX_COLOURS']['slow']
    assert tuple(render(image, [box(eligible=False)], values)[20, 20]) == namespace['UNPUBLISHED_COLOUR']


def test_left_false_is_an_actual_direction_publication():
    published = namespace['bbox_is_published']
    assert published(box('left'), {'/yolo/LR_sign': False})
    assert not published(box('right'), {'/yolo/LR_sign': False})
    assert not published(box('left'), {})
    assert not published(box('unconfigured'), {'/yolo/LR_sign': False})
