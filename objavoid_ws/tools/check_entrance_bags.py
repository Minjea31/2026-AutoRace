#!/usr/bin/env python3
"""Offline camera -> existing ONNX -> exact C++ entrance detector, with visual evidence."""
import argparse
import json
from pathlib import Path
import struct
import subprocess
import sys
import time

import cv2
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import CompressedImage

WS = Path(__file__).resolve().parents[1]
ROOT = WS.parent
sys.path.insert(0, str(ROOT / 'lane_fusion_ws/src/autorace_lane_fusion_deep'))
from autorace_lane_fusion_deep.runtime import create_backend, segment


def panel(jpeg, mask, result, title):
    camera = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
    camera = cv2.resize(camera, (640, 480))
    overlay = camera.copy()
    selected = cv2.resize(mask, (640, 480), interpolation=cv2.INTER_NEAREST) == 1
    overlay[selected] = (overlay[selected] * 0.35 + np.array([0, 255, 0]) * 0.65).astype(np.uint8)
    bev = np.full((480, 400, 3), 24, np.uint8)
    def xy(x, y):
        return round(200-y*145), round(450-x*180)
    for x in np.arange(0, 2.5, 0.5):
        cv2.line(bev, xy(x, -1.3), xy(x, 1.3), (60, 60, 60), 1)
    for x, y in result['points']:
        cv2.circle(bev, xy(x, y), 2, (0, 160, 255), -1)
    if result['ok']:
        c, s = np.cos(result['yaw']), np.sin(result['yaw'])
        for sign in (-1, 1):
            pts=[]
            for x in np.arange(max(0, result['x']-.3), result['x']+1.0, .02):
                y=result['center']+sign*(.25+np.clip(x-result['x'], 0, .52))
                pts.append(xy(c*x-s*y, s*x+c*y))
            cv2.polylines(bev,[np.asarray(pts,np.int32)],False,(80,255,80),2)
    canvas=np.zeros((520,1680,3),np.uint8)
    canvas[40:520,:640]=camera;canvas[40:520,640:1280]=overlay;canvas[40:520,1280:]=bev
    cv2.putText(canvas,title,(12,27),cv2.FONT_HERSHEY_SIMPLEX,.65,(255,255,255),2)
    return canvas


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('bags',nargs='+',type=Path)
    ap.add_argument('--hz',type=float,default=10)
    args=ap.parse_args()
    output=WS/'.replay/entrance_validation';output.mkdir(parents=True,exist_ok=True)
    backend=create_backend('onnx',str(ROOT/'lane_fusion_ws/src/autorace_lane_fusion_deep/model/lane_640_mask.onnx'),640,480,'cpu')
    summaries=[]
    for bag in args.bags:
        probe=subprocess.Popen([str(WS/'build/autorace_objavoid_v3_cpp/entrance_probe'),
            str(ROOT/'lane_fusion_ws/src/autorace_lane_deep_lite/config/real/mission_vision.yaml'),
            str(WS/'src/autorace_objavoid_v3/config/real_car04.yaml')],stdin=subprocess.PIPE,stdout=subprocess.PIPE,text=False)
        reader=rosbag2_py.SequentialReader()
        reader.open(rosbag2_py.StorageOptions(uri=str(bag),storage_id='sqlite3'),rosbag2_py.ConverterOptions('cdr','cdr'))
        reader.set_filter(rosbag2_py.StorageFilter(topics=['/camera/color/image_raw/compressed']))
        first=None;last=-1e9;rows=[];evidence=[];previous=None;hits=0;maximum=0;cost=[]
        try:
            while reader.has_next():
                _,data,recorded=reader.read_next();stamp=recorded/1e9
                if first is None:first=stamp
                if stamp-last < 1/args.hz:continue
                last=stamp;msg=deserialize_message(data,CompressedImage);jpeg=bytes(msg.data)
                bgr=cv2.imdecode(np.frombuffer(jpeg,np.uint8),cv2.IMREAD_COLOR)
                started=time.perf_counter();mask=segment(backend,bgr,640,480,.5)
                raw=mask.tobytes();probe.stdin.write(struct.pack('<IIII',640,480,len(jpeg),len(raw))+jpeg+raw);probe.stdin.flush()
                result=json.loads(probe.stdout.readline());cost.append((time.perf_counter()-started)*1000)
                coherent=result['ok'] and (previous is None or not previous['ok'] or
                    (abs(result['x']-previous['x'])<.2 and abs(result['center']-previous['center'])<.15 and
                     abs(result['yaw']-previous['yaw'])<.15))
                hits=hits+1 if coherent else 0;maximum=max(maximum,hits);previous=result
                row={k:v for k,v in result.items() if k!='points'};row.update(t=round(stamp-first,3),hits=hits);rows.append(row)
                if (result['ok'] and (not evidence or all(abs(row['t']-e[0]['t'])>1 for e in evidence))):
                    evidence.append((row,panel(jpeg,mask,result,f"{bag.name} t={row['t']:.2f}s detected L={result['left']} R={result['right']} support={result['support']}")))
                    evidence.sort(key=lambda e:e[0]['support'],reverse=True);evidence=evidence[:3]
                if len(rows)==1:
                    cv2.imwrite(str(output/(bag.name+'_first.jpg')),panel(jpeg,mask,result,f'{bag.name} first sampled frame'))
            probe.stdin.close();probe.wait(timeout=5)
            assert probe.returncode==0
        finally:
            if probe.poll() is None:probe.terminate();probe.wait(timeout=5)
        for i,(row,frame) in enumerate(evidence):cv2.imwrite(str(output/(bag.name+f'_detected_{i}.jpg')),frame)
        detected=[row for row in rows if row['ok']]
        summary={'bag':bag.name,'sampled':len(rows),'detections':len(detected),'first_detection':detected[0]['t'] if detected else None,
            'first_confirmed':next((row['t'] for row in rows if row['hits']>=3),None),'max_consecutive':maximum,'three_frame_confirmation':maximum>=3,'mean_pipeline_ms':round(float(np.mean(cost)),2),
            'evidence':[dict(row,image=bag.name+f'_detected_{i}.jpg') for i,(row,_) in enumerate(evidence)]}
        (output/(bag.name+'.json')).write_text(json.dumps({'summary':summary,'frames':rows},indent=2))
        summaries.append(summary);print(json.dumps(summary),flush=True)
    (output/'summary.json').write_text(json.dumps(summaries,indent=2))


if __name__=='__main__':main()
