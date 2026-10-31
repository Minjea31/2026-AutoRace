#!/usr/bin/env python3
"""Render and assert actual vehicle motion, then save a GIF/MP4 and a report.

This is a deterministic simulator regression scenario, not a sensor-based parking
algorithm. It drives the SAME command/model/render code as the application. ROS
DDS control is checked independently by check_ros_topics.py.
"""
import argparse
import json
import math
from pathlib import Path
import shutil
import subprocess

import numpy as np
from PIL import Image, ImageDraw

from limo_parking_sim.config import load_config
from limo_parking_sim.model import Simulation
from limo_parking_sim.scene import Scene


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',default='simul/docs/render-check')
    parser.add_argument('--camera-inset',action='store_true',help='Include sensor inset in the validation video')
    options = parser.parse_args()
    out = Path(options.output)
    out.mkdir(parents=True,exist_ok=True)
    cfg = load_config()
    cfg['display']['show_camera'] = options.camera_inset
    model = Simulation(cfg)
    scene = Scene(cfg,model,headless=True)
    frames,still,report = [],[],{}
    ticks = 0
    initial_rgb = scene.color_image()

    def capture(label):
        ranges = model.scan()
        scene.render_frame({'color','depth'})
        assert abs(scene.ego.getX()-model.x)<1e-5
        assert abs(scene.ego.getY()-model.y)<1e-5
        assert abs(scene.ego.getH()-math.degrees(model.yaw))<1e-4
        assert np.isfinite(ranges).all()
        texture = scene.base.win.getScreenshot()
        rgb = np.frombuffer(texture.getRamImageAs('RGB'),np.uint8).reshape(texture.getYSize(),texture.getXSize(),3)[::-1]
        image = Image.fromarray(rgb).resize((768,480),getattr(Image,'Resampling',Image).BILINEAR)
        draw = ImageDraw.Draw(image)
        draw.rectangle((0,0,768,40),fill=(13,24,34))
        draw.text((12,7),f'LIMO PRO | {label} | t={model.time:.2f}s',fill=(155,232,252))
        draw.text((12,23),f'x={model.x:.3f} y={model.y:.3f} yaw={math.degrees(model.yaw):+.1f}  v={model.speed:+.2f}  contact={model.collision}',fill=(245,245,245))
        # Actual on-vehicle sensor image, acquired from the same rendered pose.
        if scene.preview_visible:
            camera = Image.fromarray(scene.color_image()).resize((192,144))
            image.paste(camera,(564,48))
        frames.append(image)
        return image

    def step(v,w,label):
        nonlocal ticks
        model.command(v,w)
        model.step(1/120)
        ticks += 1
        if ticks%10==0:
            capture(label)

    def drive(seconds,v,w,label):
        for _ in range(round(seconds*120)):
            step(v,w,label)
        still.append(capture(label))

    def until(predicate,v,w,label,timeout):
        start = model.time
        while not predicate():
            if model.time-start>timeout:
                raise AssertionError(f'{label} timed out')
            step(v,w,label)
            assert not model.collision,f'{label}: unexpected collision {model.collision_names}'
        still.append(capture(label))

    try:
        capture('INITIAL')
        x0,y0 = model.x,model.y
        drive(1.2,.2,0,'FORWARD')
        assert model.x>x0+.15 and abs(model.y-y0)<1e-8
        drive(1.4,.2,.2*math.tan(.35)/.2,'LEFT STEER')
        assert model.yaw>.3
        left_pose = [model.x,model.y,model.yaw]
        drive(.3,0,0,'STOP')
        reverse_start = model.x
        drive(1.4,-.18,-.18*math.tan(.35)/.2,'REVERSE WITH STEER')
        assert model.x<reverse_start-.15 and model.yaw<left_pose[2]
        drive(.3,0,0,'STOP')
        drive(1.2,.2,-.2*math.tan(.35)/.2,'RIGHT STEER')
        assert model.yaw<-.2
        report['forward_reverse_left_right'] = 'PASS'
        assert np.mean(np.abs(scene.color_image().astype(float)-initial_rgb))>1

        # Move to a known free approach facing an occupied bay, then hit it.
        model.reset()
        model.x,model.y,model.yaw = 6.360085,4.84,-math.pi/2
        assert not model.collisions_at(model.x,model.y,model.yaw)
        drive(2.,.2,0,'OBSTACLE CONTACT / STOP')
        assert model.collision and model.speed==0
        assert 'parked_left' in model.collision_names
        contact_y = model.y
        drive(1.,-.2,0,'REVERSE OUT OF CONTACT')
        assert not model.collision and model.y>contact_y+.1
        report['collision_stop_and_reverse_escape'] = 'PASS'

        model.reset()
        assert (model.x,model.y,model.yaw) == tuple(cfg['simulation']['spawn'])
        assert model.speed==0 and model.collisions==0
        report['reset'] = 'PASS'
        capture('RESET / PARKING MOTION CHECK')

        # Known-geometry control scenario exercises a full 90-degree manoeuvre.
        # It deliberately uses simulator truth for validation only.
        until(lambda:model.x>=6.40,.15,0,'PARKING APPROACH',10.)
        until(lambda:model.yaw<=-math.pi/2,.12,-.12/model.min_radius,'RIGHT QUARTER TURN',10.)
        until(lambda:model.y<=4.27,.10,0,'ENTER EMPTY BAY',3.)
        drive(.5,0,0,'STOP IN EMPTY BAY')
        assert not model.collision
        assert abs(model.x-6.88)<.08 and abs(model.y-4.27)<.04
        assert abs(model.yaw+math.pi/2)<.08
        report['ninety_degree_manoeuvre'] = {'result':'PASS','pose':[model.x,model.y,model.yaw]}
        report['render_pose_matches_model'] = 'PASS'
        report['camera_pixels_change_with_motion'] = 'PASS'
        report['physics_ticks'] = ticks
        report['video_frames'] = len(frames)
        report['simulated_seconds'] = model.time
        report['result'] = 'PASS'
        gif = out/'motion.gif'
        # A shared palette avoids noisy per-frame colour changes and keeps the
        # review artifact small enough to retain beside the source.
        palette = frames[0].quantize(colors=64)
        indexed = [im.quantize(palette=palette,dither=0) for im in frames]
        indexed[0].save(gif,save_all=True,append_images=indexed[1:],duration=83,loop=0,optimize=True)
        if shutil.which('ffmpeg'):
            subprocess.run(['ffmpeg','-y','-loglevel','error','-i',str(gif),'-pix_fmt','yuv420p',
                            '-crf','24','-movflags','+faststart',str(out/'motion.mp4')],check=True)
        selected = [still[i] for i in (0,1,3,5,6,len(still)-1)]
        sheet = Image.new('RGB',(1536,1440))
        for i,image in enumerate(selected):
            sheet.paste(image,((i%2)*768,(i//2)*480))
        sheet.save(out/'contact-sheet.jpg',quality=90)
        Image.fromarray(scene.color_image()).save(out/'final-camera.png')
        Image.fromarray(scene.depth_image()).save(out/'final-depth-mm.png')
        (out/'report.json').write_text(json.dumps(report,indent=2)+'\n')
        print(json.dumps(report,indent=2))
    finally:
        scene.close()


if __name__ == '__main__':
    main()
