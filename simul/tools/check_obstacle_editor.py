#!/usr/bin/env python3
"""Exercise real Tk mouse bindings and render the resulting physical scene.

Requires a desktop DISPLAY. Saves screenshots and a report, and uses a temporary
layout so the user's saved obstacle arrangement is never replaced.
"""
import argparse
import json
import math
from pathlib import Path
import tempfile
import time

import numpy as np
from PIL import Image, ImageGrab

from limo_parking_sim.config import load_config
from limo_parking_sim.model import Simulation
from limo_parking_sim.obstacle_editor import read_layout
from limo_parking_sim.scene import Scene


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', default='simul/captures/editor-check')
    options = parser.parse_args()
    out = Path(options.output).resolve()
    out.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='autorace-editor-') as temporary:
        cfg = load_config()
        cfg['_obstacle_layout_path'] = str(Path(temporary)/'layout.yaml')
        cfg['simulation']['spawn'] = [3., 3., 0.]
        model = Simulation(cfg)
        scene = Scene(cfg, model, headless=True)
        try:
            scene.toggle_editor()
            editor = scene.editor
            assert editor is not None
            # E toggles the independent window without closing the simulator.
            scene.toggle_editor()
            assert editor.closed and not scene.closed
            scene.toggle_editor()
            editor = scene.editor
            scene.view = 'free'
            scene.orbit.set_pose((6.2,-1.3,3.5), (6.2,.95,0))

            def settle():
                for _ in range(3):
                    scene.render_frame({'color', 'depth'})
                    time.sleep(.04)

            def click(x, y, button=1):
                px, py = editor.screen(x, y)
                editor.canvas.event_generate(f'<Button-{button}>', x=round(px), y=round(py))
                editor.canvas.event_generate(f'<ButtonRelease-{button}>', x=round(px), y=round(py))
                settle()

            frames = []

            def capture(name):
                settle()
                root = editor.root
                bbox = (root.winfo_rootx(), root.winfo_rooty(),
                        root.winfo_rootx()+root.winfo_width(), root.winfo_rooty()+root.winfo_height())
                shot = ImageGrab.grab(bbox=bbox)
                shot.save(out/f'{name}-editor.png')
                frames.append(shot)
                scene.snapshot(out/name)

            settle()
            count = len(cfg['world']['obstacles'])
            capture('01-existing')
            click(4.4, 1.5)
            assert len(cfg['world']['obstacles']) == count+1
            added = cfg['world']['obstacles'][-1]['name']
            assert not scene.render.find('**/'+added).isEmpty()
            capture('02-added')
            old_cell = editor.placement.grid_cell(4.4,1.5)
            click(*old_cell[2:])
            assert editor.placement.selected == added
            click(4.9, 1.5)
            assert len(cfg['world']['obstacles']) == count+1
            assert cfg['world']['obstacles'][-1]['grid_column'] == 2
            assert not model.collisions_at(old_cell[2],old_cell[3],math.pi)
            capture('03-moved')
            destination = editor.placement.grid_cell(4.9,1.5)
            click(*destination[2:], button=3)
            assert len(cfg['world']['obstacles']) == count
            assert scene.render.find('**/'+added).isEmpty()
            capture('04-deleted')
            editor.undo()
            assert not scene.render.find('**/'+added).isEmpty()
            assert read_layout(editor.placement.path,cfg) == cfg['world']['obstacles']
            capture('05-undo')
            # Keep every edit within the three lanes. Move ego to look at the
            # first empty upper cell, then verify the same physical sensors.
            model.x, model.y, model.yaw = 3.9,1.46,0.
            settle()
            before_rgb, before_depth, before_scan = scene.color_image(), scene.depth_image(), model.scan()
            click(4.4,1.46)
            placed = cfg['world']['obstacles'][-1]
            assert placed['name'] != added
            px,py,_ = placed['pose']
            assert abs(px-old_cell[2]) < .01 and abs(py-1.46) < .01
            assert np.count_nonzero(np.max(np.abs(scene.color_image().astype(float)-before_rgb),axis=2)>20)>100
            assert np.count_nonzero(scene.depth_image()!=before_depth)>100
            mid = len(model.angles)//2
            assert model.scan()[mid] < 1. and model.scan()[mid] < before_scan[mid]
            assert placed['name'] in model.collisions_at(px,py,0.)
            scene.set_view('follow')
            capture('06-camera-and-map')
            click(px,py,button=3)
            assert np.array_equal(model.scan(),before_scan)
            assert np.array_equal(scene.color_image(),before_rgb)
            assert np.array_equal(scene.depth_image(),before_depth)
            # Window resizing preserves the inverse mouse/map transform.
            editor.root.geometry('900x580')
            settle()
            assert np.allclose(editor.world(*editor.screen(5.,1.)), (5.,1.))
            unchanged = len(cfg['world']['obstacles'])
            assert not editor.placement.click(3.,3.)
            assert len(cfg['world']['obstacles']) == unchanged
            report = dict(result='PASS', checks=[
                'separate window open/close/reopen', 'existing vehicles displayed',
                'left click installs at grid centre', 'select then click moves without duplication',
                'right click deletes visual and collision geometry', 'undo restores',
                'atomic save/load preserves edited list', 'edits restricted to three lanes',
                'RGB and depth pixels change', 'lidar distance and collision update',
                'deletion restores original RGB/depth/lidar', 'resized mouse coordinates',
            ])
            frames[0].save(out/'editing.gif',save_all=True,append_images=frames[1:],duration=1000,loop=0)
            (out/'report.json').write_text(json.dumps(report,indent=2)+'\n')
            print(json.dumps(report,indent=2))
        finally:
            scene.close()


if __name__ == '__main__':
    main()
