"""Run the lanedeep U-Net over a ROS 2 bag: overlay video and/or per-frame class masks.

usage (ROS sourced; python with torch for .pt, onnxruntime for .onnx):
  python tools/bag_masks.py BAG MODEL(.pt|.onnx) OUT.mp4|- [--topic T] [--threshold 0.5] [--masks DIR]
  e.g. MODEL = src/autorace_lane_fusion_deep/model/best.pt
OUT.mp4: mask painted on the dimmed original (yellow = orange_lane, white = white_dashed_lane).
DIR: NNNNNN.png class masks by bag frame index, for lane_fusion_eval/lane_fusion_tuner --deep-masks.
"""
import argparse
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import CompressedImage, Image


def to_bgr(msg):
    if isinstance(msg, CompressedImage):
        return cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
    img = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)[:, :msg.width * 3]
    img = img.reshape(msg.height, msg.width, 3)
    return cv2.cvtColor(img, cv2.COLOR_RGB2BGR) if msg.encoding == 'rgb8' else img.copy()


def main():
    p = argparse.ArgumentParser()
    p.add_argument('bag'), p.add_argument('model'), p.add_argument('out')
    p.add_argument('--topic', default='/camera/color/image_raw')
    p.add_argument('--threshold', type=float, default=0.5)
    p.add_argument('--masks', type=Path)
    a = p.parse_args()
    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src/autorace_lane_fusion_deep'))
    from autorace_lane_fusion_deep.runtime import create_backend, decode, preprocess
    size = (640, 480)
    if a.model.endswith('.pt'):
        import torch
        size = tuple(torch.load(a.model, map_location='cpu', weights_only=False)['size'])
        device = 'cuda' if torch.cuda.is_available() else 'cpu'
    else:
        device = 'cpu'
    backend = create_backend('auto', a.model, size[0], size[1], device)

    if a.masks:
        a.masks.mkdir(parents=True, exist_ok=False)
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=a.bag, storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    cls = CompressedImage if types[a.topic].endswith('CompressedImage') else Image
    reader.set_filter(rosbag2_py.StorageFilter(topics=[a.topic]))

    frames, stamps = [], []
    while reader.has_next():
        _, data, t = reader.read_next()
        frames.append(data), stamps.append(t)
    fps = (len(stamps) - 1) / ((stamps[-1] - stamps[0]) * 1e-9) if len(stamps) > 1 else 15
    writer, ms, lane = None, [], 0
    for i, data in enumerate(frames):
        bgr = to_bgr(deserialize_message(data, cls))
        t0 = time.perf_counter()
        mask = decode(backend.logits(preprocess(bgr, *size)), (bgr.shape[1], bgr.shape[0]), a.threshold)
        if getattr(backend, 'device', None) is not None and getattr(backend.device, 'type', '') == 'cuda':
            __import__('torch').cuda.synchronize()
        ms.append((time.perf_counter() - t0) * 1000)
        lane += bool(mask.any())
        if a.masks:
            cv2.imwrite(str(a.masks / f'{i:06d}.png'), mask)
        if a.out == '-':
            continue
        out = (bgr * .55).astype(np.uint8)
        out[mask == 1] = (0, 220, 255)
        out[mask == 2] = (255, 255, 255)
        label = f'lanedeep (U-Net {size[0]}x{size[1]})  #{i}'
        cv2.putText(out, label, (10, 28), cv2.FONT_HERSHEY_SIMPLEX, .7, (0, 0, 0), 4)
        cv2.putText(out, label, (10, 28), cv2.FONT_HERSHEY_SIMPLEX, .7, (0, 255, 0), 2)
        if writer is None:
            writer = cv2.VideoWriter(a.out, cv2.VideoWriter_fourcc(*'mp4v'), fps, (out.shape[1], out.shape[0]))
        writer.write(out)
    if writer is not None:
        writer.release()
    ms = np.array(ms[10:] or ms)
    print(f'frames {len(frames)} | any-lane {100 * lane / len(frames):.1f}% | '
          f'infer+decode {ms.mean():.1f} ms (p95 {np.percentile(ms, 95):.1f}) via {backend.name}')


if __name__ == '__main__':
    main()
