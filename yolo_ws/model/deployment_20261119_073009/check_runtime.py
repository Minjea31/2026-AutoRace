#!/usr/bin/env python3
"""Check both detector entry points and engines without starting ROS nodes."""
import argparse
import hashlib
import importlib.util
import json
from pathlib import Path
import sys

sys.path.insert(0,'/home/wego/2026-AutoRace/yolo_ws/yolo26')
import cv2
from ultralytics import YOLO


def main():
    parser=argparse.ArgumentParser();parser.add_argument('config',type=Path)
    args=parser.parse_args();cfg=json.loads(args.config.read_text())
    workspace=Path(cfg['remote_workspace']);job=Path(cfg['remote_job'])
    validation=json.loads((job/'validation.json').read_text())
    results=[]
    for label,script in [('source',workspace/'src/yolo_detect.py'),
                         ('installed',workspace/'install/yolo_detect/lib/yolo_detect/yolo_detect')]:
        # Installed executable has no .py suffix, so use an explicit source loader.
        from importlib.machinery import SourceFileLoader
        loader=SourceFileLoader('deployment_detector_'+label,str(script))
        spec=importlib.util.spec_from_loader(loader.name,loader)
        module=importlib.util.module_from_spec(spec)
        sys.modules[loader.name]=module
        loader.exec_module(module)
        engine=module.default_model_path()
        digest=hashlib.sha256(engine.read_bytes()).hexdigest()
        assert digest==validation['engine_sha256']
        assert module.IMAGE_SIZE==640 and module.SIGNAL_CLASS_IDS=={6:'red',7:'green'}
        model=YOLO(str(engine),task='detect')
        image=cv2.imread(str(sorted((job/'samples').glob('*.jpg'))[0]))
        prediction=model.predict(image,imgsz=640,rect=False,device=0,conf=.25,
                                 verbose=False,save=False,augment=False)[0]
        assert [model.names[i] for i in range(8)]==validation['classes']
        results.append(dict(entry_point=label,script=str(script),default_model=str(engine),
                            engine_sha256=digest,class_mapping_matches=True,
                            real_image_boxes=len(prediction.boxes)))
    report=dict(passed=True,ros_nodes_started=False,entry_points=results)
    (job/'runtime_check.json').write_text(json.dumps(report,indent=2)+'\n')
    print('RUNTIME_CHECK_PASSED '+json.dumps(report),flush=True)


if __name__=='__main__':
    main()
