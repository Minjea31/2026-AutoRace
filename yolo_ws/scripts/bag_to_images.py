#!/usr/bin/env python3
"""Export images from a ROS 2 bag: bag_to_images.py BAG IMAGES_PER_SECOND."""
import argparse
import math
from pathlib import Path
import sys


def main():
    parser = argparse.ArgumentParser(description='ROS 2 bag에서 지정한 초당 장수로 이미지를 저장합니다.')
    parser.add_argument('bag', type=Path, help='bag 디렉터리 또는 .db3/.mcap 파일')
    parser.add_argument('fps', type=float, help='초당 저장 장수 (0.1초 간격은 10)')
    args = parser.parse_args()
    if not math.isfinite(args.fps) or not 0 < args.fps <= 1e9:
        parser.error('초당 장수는 0보다 크고 1,000,000,000 이하여야 합니다.')
    bag = args.bag.expanduser().resolve()
    if not bag.exists():
        parser.error(f'bag 경로가 없습니다: {bag}')

    try:
        import cv2
        import numpy as np
        import rosbag2_py
        from cv_bridge import CvBridge
        from rclpy.serialization import deserialize_message
        from sensor_msgs.msg import CompressedImage, Image
    except ImportError as exc:
        parser.exit(1, f'ROS Python 의존성을 불러오지 못했습니다: {exc}\n'
                       'source /opt/ros/humble/setup.bash 후 /usr/bin/python3으로 실행하세요.\n')

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id=''),
                rosbag2_py.ConverterOptions('', ''))
    image_types = {'sensor_msgs/msg/CompressedImage': CompressedImage,
                   'sensor_msgs/msg/Image': Image}
    candidates = [t for t in reader.get_all_topics_and_types() if t.type in image_types]
    if not candidates:
        parser.exit(1, 'bag에 Image 또는 CompressedImage 토픽이 없습니다.\n')
    # Prefer the robot's color camera; export only one topic to avoid duplicate images.
    candidates.sort(key=lambda t: (t.name not in (
        '/camera/color/image_raw/compressed', '/camera/color/image_raw'),
        'color' not in t.name, t.type != 'sensor_msgs/msg/CompressedImage', t.name))
    topic = candidates[0]
    reader.set_filter(rosbag2_py.StorageFilter(topics=[topic.name]))
    output = Path.cwd() / f'{bag.stem}_images_{args.fps:g}fps'
    # A second run must not silently overwrite a previous export.
    output.mkdir(parents=True, exist_ok=False)
    print(f'토픽: {topic.name}\n저장 간격: {1 / args.fps:g}초\n출력: {output}')
    bridge = CvBridge()
    start = None
    slot = 0
    saved = 0
    failed = 0
    while reader.has_next():
        _, data, stamp = reader.read_next()
        if start is None:
            start = stamp
        elapsed = (stamp - start) / 1e9
        if elapsed + 1e-9 < slot / args.fps:
            continue
        msg = deserialize_message(data, image_types[topic.type])
        try:
            if topic.type == 'sensor_msgs/msg/CompressedImage':
                if 'compressedDepth' in msg.format:
                    raise ValueError('compressedDepth 대신 원본 Image 토픽을 사용하세요.')
                frame = cv2.imdecode(np.frombuffer(msg.data, dtype=np.uint8), cv2.IMREAD_UNCHANGED)
                if frame is None:
                    raise ValueError('이미지 디코딩 실패')
            else:
                frame = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
                if msg.encoding in ('rgb8', 'rgba8'):
                    frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR if msg.encoding == 'rgb8' else cv2.COLOR_RGBA2BGRA)
            filename = output / f'{saved:06d}_{elapsed:010.3f}s.png'
            if not cv2.imwrite(str(filename), frame):
                raise ValueError(f'이미지 저장 실패: {filename}')
        except (ValueError, cv2.error) as exc:
            failed += 1
            print(f'건너뜀 ({elapsed:.3f}초): {exc}', file=sys.stderr)
            continue
        saved += 1
        slot = max(slot + 1, math.floor(elapsed * args.fps + 1e-9) + 1)
    print(f'완료: {saved}장 저장, {failed}장 실패')
    return 1 if failed else 0


if __name__ == '__main__':
    try:
        sys.exit(main())
    except (RuntimeError, OSError) as exc:
        sys.exit(f'오류: {exc}')
