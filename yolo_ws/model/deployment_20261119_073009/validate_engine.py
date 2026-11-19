#!/usr/bin/env python3
"""Compare exported TensorRT detections against the uploaded PyTorch checkpoint."""
import argparse
import hashlib
import json
from pathlib import Path
import statistics
import sys
import time

sys.path.insert(0,'/home/wego/2026-AutoRace/yolo_ws/yolo26')
import cv2
import numpy as np
import torch
from ultralytics import YOLO

NAMES=['slow','bump','crosswalk','left','right','parking','red','green']


def predict(model,image):
    return model.predict(image,imgsz=640,rect=False,conf=.25,iou=.7,
                         device=0,verbose=False,save=False,augment=False)[0]


def box_iou(a,b):
    left=np.maximum(a[:2],b[:2]);right=np.minimum(a[2:4],b[2:4])
    intersection=np.prod(np.maximum(0,right-left))
    area_a=np.prod(np.maximum(0,a[2:4]-a[:2]))
    area_b=np.prod(np.maximum(0,b[2:4]-b[:2]))
    return float(intersection/max(1e-9,area_a+area_b-intersection))


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--source',required=True,type=Path)
    parser.add_argument('--engine',required=True,type=Path)
    parser.add_argument('--samples',required=True,type=Path)
    parser.add_argument('--report',required=True,type=Path)
    args=parser.parse_args()
    pt=YOLO(str(args.source),task='detect')
    trt=YOLO(str(args.engine),task='detect')
    samples=sorted(args.samples.glob('*.jpg'))
    assert len(samples)>=6
    images=[cv2.imread(str(p)) for p in samples]
    assert all(im is not None and im.shape==(480,640,3) for im in images)
    # Initialize the lazy TensorRT backend before examining model names.
    predict(pt,images[0]);predict(trt,images[0])
    assert [pt.names[i] for i in range(8)]==NAMES
    assert [trt.names[i] for i in range(8)]==NAMES
    results=[];ious=[];confidence_diffs=[]
    for path,image in zip(samples,images):
        a=predict(pt,image).boxes.data.cpu().numpy()
        b=predict(trt,image).boxes.data.cpu().numpy()
        matches=[]
        for row in a:
            if row[4]<.4:
                continue
            same=b[b[:,5]==row[5]]
            assert len(same),f'Missing high-confidence class {int(row[5])}: {path.name}'
            paired=max(same,key=lambda candidate:box_iou(row,candidate))
            iou=box_iou(row,paired);difference=abs(float(row[4]-paired[4]))
            assert iou>=.98,(path.name,'IoU',iou)
            assert difference<=.03,(path.name,'confidence_difference',difference)
            ious.append(iou);confidence_diffs.append(difference)
            matches.append(dict(name=NAMES[int(row[5])],iou=iou,
                                pt_confidence=float(row[4]),engine_confidence=float(paired[4])))
        results.append(dict(image=path.name,pt_boxes=len(a),engine_boxes=len(b),matches=matches))
        print('SAMPLE '+json.dumps(results[-1]),flush=True)
    assert len(ious)>=6
    timings={}
    for name,model in [('pytorch',pt),('tensorrt_fp16',trt)]:
        for _ in range(3):predict(model,images[0])
        elapsed=[]
        for _ in range(20):
            torch.cuda.synchronize();start=time.perf_counter()
            predict(model,images[0]);torch.cuda.synchronize()
            elapsed.append((time.perf_counter()-start)*1000)
        timings[name]=dict(median_predict_ms=statistics.median(elapsed),iterations=20)
    report=dict(passed=True,source=str(args.source),engine=str(args.engine),
                source_sha256=hashlib.sha256(args.source.read_bytes()).hexdigest(),
                engine_sha256=hashlib.sha256(args.engine.read_bytes()).hexdigest(),
                classes=NAMES,sample_count=len(samples),matched_high_confidence_boxes=len(ious),
                minimum_iou=min(ious),maximum_confidence_difference=max(confidence_diffs),
                timings=timings,samples=results)
    args.report.write_text(json.dumps(report,indent=2)+'\n')
    print('VALIDATION_PASSED '+json.dumps({k:v for k,v in report.items() if k!='samples'}),flush=True)


if __name__=='__main__':
    main()
