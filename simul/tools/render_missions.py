#!/usr/bin/env python3
"""Render all mission facilities and verify two collision-free manoeuvres.

Uses ground truth only as a simulator regression driver. This is not the user's
camera/lidar-based mission algorithm and is not launched with the simulator.
"""
import argparse
import json
import math
from pathlib import Path
import shutil
import subprocess

import numpy as np
from PIL import Image,ImageDraw
from panda3d.core import BitMask32

from limo_parking_sim.config import load_config
from limo_parking_sim.model import Simulation
from limo_parking_sim.scene import Scene
from limo_parking_sim.world import cone_course_shape,cone_reference_path,obstacle_reference_path


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',default='simul/docs/missions-check')
    options=parser.parse_args()
    out=Path(options.output)
    out.mkdir(parents=True,exist_ok=True)
    cfg=load_config()
    model=Simulation(cfg)
    scene=Scene(cfg,model,headless=True)
    frames=[]
    still=[]
    report={'vehicles':len(cfg['world']['obstacles']),'avoidance_vehicles':7,
            'parking_vehicles':5,'cones':len(model.cones)}
    course=cfg['world']['cone_course']
    length=course['end_line_y']-course['start_line_y']-course.get('cone',{}).get('base_size',.08)
    frequency=math.pi*course['half_waves']/length
    reference=cone_reference_path(course)
    layout_x,_=cone_course_shape(course,reference[:,1])
    delta=np.diff(layout_x)
    bends=int(np.count_nonzero(delta[:-1]*delta[1:]<0))
    assert bends==4,'Cone mission must contain two full S waves'
    report['cone_course']={'s_waves':bends//2,'interior_bends':bends,
                           'bend_order':['right','left','right','left'],
                           'phase_deg':course['phase_deg'],
                           'cone_peak_to_peak_m':2*course['amplitude'],
                           'reference_peak_to_peak_m':2*course['reference_amplitude'],
                           'horizontal_row_separation_m':2*course['half_width'],
                           'reference_min_turn_radius_m':1/(course['reference_amplitude']*frequency**2),
                           'vehicle_min_turn_radius_m':model.min_radius,
                           'painted_edge_crossing_allowed':True}

    def camera_pose(eye,target):
        scene.view='free'
        scene.orbit.set_pose(eye,target)

    def capture(name,save=False,video=False):
        scene.render_frame({'color','depth'})
        texture=scene.base.win.getScreenshot()
        rgb=np.frombuffer(texture.getRamImageAs('RGB'),np.uint8).reshape(texture.getYSize(),texture.getXSize(),3)[::-1]
        im=Image.fromarray(rgb)
        if save:
            im.save(out/(name+'.png'))
            Image.fromarray(scene.color_image()).save(out/(name+'-camera.png'))
        im=im.resize((768,480))
        draw=ImageDraw.Draw(im)
        draw.rectangle((0,0,768,34),fill=(13,24,34))
        draw.text((10,5),name.replace('-',' ').upper(),fill=(160,230,250))
        draw.text((10,19),f'LIMO PRO | x={model.x:.3f} y={model.y:.3f} yaw={math.degrees(model.yaw):+.1f} | collisions={model.collisions}',fill='white')
        if video:
            frames.append(im)
        return im

    def drive(name,path,speed,eye,target):
        model.reset()
        model.x,model.y,model.yaw=map(float,path[0])
        assert not model.collisions_at(model.x,model.y,model.yaw)
        camera_pose(eye,target)
        progress=0
        for ticks in range(15000):
            distances=np.linalg.norm(path[:,:2]-[model.x,model.y],axis=1)
            progress+=int(np.argmin(distances[progress:min(progress+45,len(path))]))
            if np.linalg.norm(path[-1,:2]-[model.x,model.y])<.035:
                break
            index=progress
            while index<len(path)-1 and distances[index]<.13:
                index+=1
            delta=path[index,:2]-[model.x,model.y]
            lateral=-math.sin(model.yaw)*delta[0]+math.cos(model.yaw)*delta[1]
            curvature=2*lateral/max(float(delta@delta),.005)
            model.command(speed,speed*curvature)
            model.step(1/120)
            assert not model.collision,f'{name}: collision {model.collision_names}'
            if ticks%15==0:
                capture(name,video=True)
        else:
            raise AssertionError(f'{name}: reference driver did not finish')
        report[name]={'result':'PASS','physics_ticks':ticks,
                      'final_pose':[model.x,model.y,model.yaw],'collisions':model.collisions}
        still.append(capture(name,save=True,video=True))

    try:
        scene.set_view('overview')
        still.append(capture('overview',save=True,video=True))
        drive('s-cone-mission',cone_reference_path(cfg['world']['cone_course']),.10,(-1.20,2.90,2.8),(.72,2.90,0))
        scene.ego.hide(BitMask32.bit(0))
        capture('cone-layout',save=True)
        scene.ego.show(BitMask32.bit(0))
        drive('seven-vehicle-avoidance',obstacle_reference_path(model.min_radius),.13,
              (6.1,-1.20,3.8),(6.1,.94,0))
        scene.ego.hide(BitMask32.bit(0))
        capture('obstacle-layout',save=True)
        scene.ego.show(BitMask32.bit(0))
        model.reset()
        model.x,model.y,model.yaw=cfg['world']['spawns']['fork']
        camera_pose((4.95,.02,1.20),(4.10,.94,.20))
        scene.set_fork('left')
        still.append(capture('fork-left',save=True))
        left=scene.color_image().copy()
        scene.set_fork('right')
        still.append(capture('fork-right',save=True))
        right=scene.color_image().copy()
        changed=int(np.count_nonzero(np.max(np.abs(left.astype(float)-right),axis=2)>32))
        assert changed>40
        report['fork_camera_switch']={'result':'PASS','changed_pixels':changed}
        scene.set_fork('left')
        model.x,model.y,model.yaw=cfg['world']['spawns']['signal']
        camera_pose((3.3,3.80,1.45),(3.1,4.84,0))
        scene.set_traffic('red')
        still.append(capture('signal-red-crosswalk',save=True))
        red=scene.color_image().copy()
        scene.set_traffic('green')
        capture('signal-green-crosswalk',save=True)
        green=scene.color_image().copy()
        changed=int(np.count_nonzero(np.max(np.abs(red.astype(float)-green),axis=2)>32))
        assert changed>20
        report['traffic_camera_switch']={'result':'PASS','changed_pixels':changed}
        scene.set_traffic('red')
        model.x,model.y,model.yaw=cfg['world']['spawns']['crosswalk']
        camera_pose((8.9,2.35,1.7),(9.58,2.35,0))
        still.append(capture('pedestrian-crosswalk',save=True))
        model.x,model.y,model.yaw=cfg['world']['spawns']['slow']
        camera_pose((3.20,4.70,1.6),(3.10,5.58,0))
        still.append(capture('slow-signs-and-bump',save=True))
        camera_pose((2.53,5.89,.155),(3.05,5.89,.155))
        capture('sign-size-closeup',save=True)
        sheet=Image.new('RGB',(1536,1920))
        for i,im in enumerate(still):
            sheet.paste(im,((i%2)*768,(i//2)*480))
        sheet.save(out/'all-missions.jpg',quality=92)
        palette=frames[0].quantize(colors=96)
        indexed=[im.quantize(palette=palette,dither=0) for im in frames]
        indexed[0].save(out/'motion.gif',save_all=True,append_images=indexed[1:],duration=125,loop=0,optimize=True)
        if shutil.which('ffmpeg'):
            subprocess.run(['ffmpeg','-y','-loglevel','error','-i',str(out/'motion.gif'),
                '-pix_fmt','yuv420p','-crf','24','-movflags','+faststart',str(out/'motion.mp4')],check=True)
        report['result']='PASS'
        report['video_frames']=len(frames)
        (out/'report.json').write_text(json.dumps(report,indent=2)+'\n')
        print(json.dumps(report,indent=2))
    finally:
        scene.close()


if __name__=='__main__':
    main()
