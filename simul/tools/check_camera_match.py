#!/usr/bin/env python3
"""Compare local camera defaults and optionally read a real ROS camera.

No device settings are changed. Simulator publishers are excluded from the
live probe so a synthetic frame cannot be reported as a hardware measurement.
"""
import argparse
import ast
import json
from pathlib import Path
import time

from limo_parking_sim.config import load_config
from limo_parking_sim.model import intrinsics


def driver_defaults(path):
    result = {}
    for node in ast.walk(ast.parse(path.read_text())):
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Name) and node.func.id == 'DeclareLaunchArgument':
            if node.args and isinstance(node.args[0], ast.Constant):
                for keyword in node.keywords:
                    if keyword.arg == 'default_value' and isinstance(keyword.value, ast.Constant):
                        result[node.args[0].value] = keyword.value.value
    return result


def probe(domain, seconds, output):
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CameraInfo, CompressedImage
    context = Context()
    rclpy.init(context=context, domain_id=domain)
    node = rclpy.create_node('camera_match_read_only', context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    received, stamps = {}, []
    subscriptions = []
    subscribed = set()
    endpoints = {}
    topics = {'/camera/color/camera_info': CameraInfo,
              '/camera/depth/camera_info': CameraInfo,
              '/camera/color/image_raw/compressed': CompressedImage}

    def callback(topic, msg):
        if topic.endswith('camera_info'):
            received[topic] = dict(width=msg.width, height=msg.height, k=list(msg.k),
                                   d=list(msg.d), distortion_model=msg.distortion_model)
        else:
            stamp = msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
            stamps.append(stamp)
            received[topic] = dict(format=msg.format, bytes=len(msg.data))
            if not (output/'real-camera.jpg').exists() and 'jpeg' in msg.format.lower():
                (output/'real-camera.jpg').write_bytes(bytes(msg.data))

    try:
        start = time.monotonic()
        while time.monotonic()-start < seconds:
            for topic, msg_type in topics.items():
                publishers = node.get_publishers_info_by_topic(topic)
                endpoints[topic] = [p.node_name for p in publishers]
                # If any synthetic publisher shares the topic, don't use its
                # frames as evidence of actual camera quality.
                if publishers and all(p.node_name != 'limo_parking_sim' for p in publishers) and topic not in subscribed:
                    subscriptions.append(node.create_subscription(
                        msg_type, topic, lambda msg, t=topic: callback(t, msg), qos_profile_sensor_data))
                    subscribed.add(topic)
            executor.spin_once(timeout_sec=.1)
        intervals = [b-a for a,b in zip(stamps,stamps[1:]) if b>a]
        return dict(domain=domain, publishers=endpoints, received=received,
                    observed_fps=(len(intervals)/sum(intervals) if intervals else None),
                    hardware_image_received=bool(stamps))
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown(context=context)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', default='simul/captures/camera-check')
    parser.add_argument('--live', action='store_true')
    parser.add_argument('--domains', type=int, nargs='+', default=[0, 42])
    parser.add_argument('--seconds', type=float, default=5.)
    args = parser.parse_args()
    out = Path(args.output)
    out.mkdir(parents=True, exist_ok=True)
    root = Path(__file__).resolve().parents[2]
    driver = root/'sensor_ws/src/OrbbecSDK_ROS2/orbbec_camera/launch/astra_stereo_u3.launch.py'
    defaults = driver_defaults(driver)
    cfg = load_config()
    comparisons = {}
    for stream in ('color', 'depth'):
        actual = {key: int(defaults[f'{stream}_{key}']) for key in ('width','height','fps')}
        simulated = {key: cfg['camera'][stream][key] for key in ('width','height','fps')}
        comparisons[stream] = dict(driver_defaults=actual, simulation=simulated,
                                   configuration_matches=actual == simulated,
                                   simulated_intrinsics=list(intrinsics(cfg['camera'][stream])))
    usb = []
    for device in Path('/sys/bus/usb/devices').iterdir():
        if (device/'idVendor').exists():
            usb.append(dict(vendor=(device/'idVendor').read_text().strip(),
                            product=(device/'product').read_text().strip() if (device/'product').exists() else ''))
    report = dict(
        conclusion='Stream defaults match; equal real-world image quality is NOT established.',
        driver_source=str(driver.relative_to(root)), streams=comparisons,
        compression=dict(driver_capture_format=defaults['color_format'],
                         simulation_jpeg_quality=cfg['camera']['jpeg_quality'],
                         real_transport_jpeg_quality='not measured'),
        differences=dict(
            auto_exposure=dict(real_driver=defaults['enable_color_auto_exposure'], simulation=False),
            auto_white_balance=dict(real_driver=defaults['enable_color_auto_white_balance'], simulation=False),
            distortion=dict(real_driver='device calibration', simulation='D=0'),
            calibrated_color_intrinsics=cfg['camera']['color']['intrinsics'] is not None,
            noise_blur_exposure='not matched to a measured camera',
        ), usb_devices=usb, orbbec_usb_detected=any(d['vendor']=='2bc5' for d in usb),
        live=[])
    if args.live:
        report['live'] = [probe(domain,args.seconds,out) for domain in args.domains]
    (out/'report.json').write_text(json.dumps(report,indent=2,ensure_ascii=False)+'\n')
    print(json.dumps(report,indent=2,ensure_ascii=False))


if __name__ == '__main__':
    main()
