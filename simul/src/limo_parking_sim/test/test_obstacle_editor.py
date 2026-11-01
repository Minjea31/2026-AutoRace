import copy
import math

import numpy as np
import pytest
import yaml

from limo_parking_sim.config import load_config, share_directory
from limo_parking_sim.model import Simulation
from limo_parking_sim.obstacle_editor import Placement, read_layout


@pytest.fixture
def editor(tmp_path):
    cfg = load_config()
    model = Simulation(cfg)
    return Placement(cfg, model, path=tmp_path/'layout.yaml')


def test_grid_place_move_delete_undo_and_restart(editor, tmp_path):
    before = copy.deepcopy(editor.obstacles)
    cell = editor.grid_cell(4.4, 1.5)
    assert cell[:2] == ('upper', 1)
    assert editor.click(4.4, 1.5)
    added = editor.obstacles[-1]
    assert added['pose'] == pytest.approx([cell[2], cell[3], math.pi])
    assert added['grid_column'] == 1
    assert editor.model.collisions_at(*added['pose']) == [added['name']]
    assert not editor.click(*cell[2:])  # select the existing obstacle
    assert editor.selected == added['name']
    destination = editor.grid_cell(4.9, 1.5)
    assert editor.click(4.9, 1.5)
    assert editor.obstacles[-1]['pose'][:2] == pytest.approx(destination[2:])
    assert not editor.model.collisions_at(cell[2], cell[3], math.pi)
    assert editor.click(*destination[2:], button=3)
    assert editor.obstacles == before
    editor.undo()
    assert editor.obstacles[-1]['name'] == added['name']
    assert read_layout(editor.path, editor.cfg) == editor.obstacles
    # A fresh normal config load uses the saved explicit list after expanding
    # defaults, so deleted/moved avoid_* vehicles cannot reappear.
    world_path = tmp_path/'track.yaml'
    world_path.write_text((share_directory()/'config/parking_map.yaml').read_text())
    world_path.with_suffix('.obstacles.yaml').write_text(editor.path.read_text())
    assert load_config(world=world_path)['world']['obstacles'] == editor.obstacles


def test_remove_existing_generated_obstacle_persists(editor):
    old = next(o for o in editor.obstacles if o['name'].startswith('avoid_'))
    assert editor.click(*old['pose'][:2], button=3)
    assert not any(o['name'] == old['name'] for o in read_layout(editor.path, editor.cfg))
    assert not editor.model.collisions_at(*old['pose'])


def test_grid_placement_updates_scan_and_collision(editor):
    m = editor.model
    m.x, m.y, m.yaw = 3.9, 1.46, 0.
    mid = len(m.angles)//2
    initial = m.scan()[mid]
    cell = editor.grid_cell(4.4, 1.46)
    assert editor.click(4.4, 1.46)
    expected = cell[2]-editor.cfg['vehicle']['length']/2-(m.x+editor.cfg['lidar']['xyz'][0])
    assert m.scan()[mid] == pytest.approx(expected, abs=1e-6)
    assert m.scan()[mid] < initial
    assert m.collisions_at(cell[2], cell[3], 0.)
    assert editor.click(cell[2], cell[3], button=3)
    assert m.scan()[mid] == initial


def test_invalid_placements_do_not_change_world(editor):
    before = copy.deepcopy(editor.obstacles)
    m = editor.model
    m.x, m.y = editor.grid_cell(4.4, 1.46)[2:]
    assert not editor.click(m.x, m.y)
    assert '주행 차량' in editor.message
    assert not editor.click(.01, 3.)
    assert not editor.click(-1., 3.)
    assert not editor.click(float('nan'), 3.)
    parked = next(o for o in editor.obstacles if o['name'].startswith('parked_'))
    assert not editor.click(*parked['pose'][:2], button=3)
    assert editor.obstacles == before and not editor.history


def test_save_failure_keeps_live_edit_and_allows_retry(editor, tmp_path):
    blocker = tmp_path/'file'
    blocker.write_text('not a directory')
    editor.path = blocker/'layout.yaml'
    assert editor.click(4.4, 1.46)
    assert not editor.saved and '저장 실패' in editor.message
    assert editor.model.collisions_at(*editor.obstacles[-1]['pose'])
    editor.path = tmp_path/'fixed.yaml'
    editor.save()
    assert editor.saved


@pytest.mark.parametrize('pose', [[float('nan'), 1, 0], [0, 0, 0], [1, 2]])
def test_invalid_saved_pose_is_rejected(editor, pose):
    editor.path.write_text(yaml.safe_dump({'version': 1, 'obstacles': [{'name': 'bad', 'pose': pose}]}))
    with pytest.raises(ValueError):
        read_layout(editor.path, editor.cfg)
