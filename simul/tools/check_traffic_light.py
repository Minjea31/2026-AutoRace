#!/usr/bin/env python3
"""Save actual signal close-ups and vehicle RGB/depth for all three colours."""
import argparse
import json
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw

from limo_parking_sim.config import load_config
from limo_parking_sim.model import Simulation
from limo_parking_sim.scene import Scene


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', default='simul/captures/traffic-light-check')
    options = parser.parse_args()
    out = Path(options.output)
    out.mkdir(parents=True, exist_ok=True)
    cfg = load_config()
    cfg['simulation']['spawn'] = cfg['world']['spawns']['signal']
    model = Simulation(cfg)
    scene = Scene(cfg, model, headless=True)
    try:
        root = scene.render.find('**/traffic_light')
        lo, hi = root.getTightBounds(root)
        body_lo, body_hi = root.find('housing').getTightBounds(root)
        assert abs(hi.y-lo.y-.12) < 1e-6
        assert abs(body_hi.z-body_lo.z-.205) < 1e-6
        assert abs(body_lo.z-.05) < 1e-6
        assert abs(hi.z-.255) < 1e-6
        report = {
            'renderer': scene.base.win.getGsg().getDriverRenderer(),
            'overall_width_m': round(hi.y-lo.y, 6),
            'body_height_m': round(body_hi.z-body_lo.z, 6),
            'stand_height_m': round(body_lo.z, 6),
            'total_height_m': round(hi.z, 6),
            'vehicle_pose': [model.x, model.y, model.yaw],
            'colour_switch': {},
        }
        scene.set_view('free')
        x, y, _ = cfg['world']['traffic_light']['pose']
        scene.orbit.set_pose((x-.48, y-.20, .30), (x, y, .128))
        rows, closeups, images = [], [], {}
        for state in ('red', 'yellow', 'green'):
            scene.set_traffic(state)
            scene.render_frame({'color', 'depth'})
            scene.snapshot(out/state)
            images[state] = scene.color_image()
            closeup = Image.open(out/state/'scene.png').convert('RGB')
            closeups.append(closeup)
            row = Image.new('RGB', (1280, 480), (16, 23, 30))
            row.paste(closeup.resize((640, 400)), (0, 50))
            row.paste(Image.fromarray(images[state]), (640, 0))
            draw = ImageDraw.Draw(row)
            draw.text((16, 16), f'{state.upper()} | BODY 12 x 20.5 cm | STAND 5 cm | TOTAL 25.5 cm', fill='white')
            draw.text((656, 16), 'VEHICLE CAMERA / 640 x 480', fill='white')
            rows.append(row)
        for before, after in (('red', 'yellow'), ('yellow', 'green'), ('red', 'green')):
            changed = int(np.count_nonzero(np.max(np.abs(images[before].astype(float)-images[after]), axis=2) > 32))
            assert changed > 20, f'{before}/{after} not visible to vehicle camera'
            report['colour_switch'][before+'_'+after] = {'changed_camera_pixels': changed, 'result': 'PASS'}
        sheet = Image.new('RGB', (1280, 480*len(rows)))
        for index, row in enumerate(rows):
            sheet.paste(row, (0, 480*index))
        sheet.save(out/'comparison.jpg', quality=95)
        closeups[0].save(out/'colour-switch.gif', save_all=True, append_images=closeups[1:], duration=1000, loop=0)
        report['result'] = 'PASS'
        (out/'report.json').write_text(json.dumps(report, indent=2)+'\n', encoding='utf-8')
        print(json.dumps(report, indent=2))
    finally:
        scene.close()


if __name__ == '__main__':
    main()
