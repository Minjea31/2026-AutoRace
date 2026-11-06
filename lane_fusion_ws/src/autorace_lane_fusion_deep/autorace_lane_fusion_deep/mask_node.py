"""Segmentation-only node for lane_fusion: camera -> /lane_fusion/deep_mask.

Publishes the network class mask (mono8: 0 background, 1 orange/yellow lane,
2 white dashed lane) at the network input size, with the CAMERA header copied
unchanged so lane_fusion_node can pair mask and frame by stamp. No tracking, no
control output and no OpenCV fallback: if the model is unavailable nothing is
published and lane_fusion_node keeps running on its own HSV features.
"""
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image

from .runtime import create_backend, decode, preprocess


class LaneDeepMaskNode(Node):
    def __init__(self):
        super().__init__('lane_fusion_deep_mask')
        p = self.declare_parameter
        self.backend_name = p('backend', 'auto').value  # auto (by suffix) | tensorrt | onnx | torch
        self.model_path = p('model_path', '').value
        self.device = p('device', 'auto').value  # auto | cuda | cpu
        self.width = p('input_width', 640).value
        self.height = p('input_height', 480).value
        self.threshold = p('confidence_threshold', 0.5).value
        self.retry = p('retry_seconds', 5.0).value
        camera = p('camera_topic', '/camera/color/image_raw/compressed').value
        output = p('mask_topic', '/lane_fusion/deep_mask').value
        if not self.model_path:
            # Installed model: the TensorRT engine built on this machine
            # (model/lane_640.engine before colcon build) if present, else the ONNX.
            share = Path(get_package_share_directory('autorace_lane_fusion_deep')) / 'model'
            engine = share / 'lane_640.engine'
            self.model_path = str(engine if engine.is_file() else share / 'lane_640.onnx')
        if self.device == 'auto':
            self.device = 'cuda'
            if self.model_path.endswith('.onnx'):
                import onnxruntime
                if 'CUDAExecutionProvider' not in onnxruntime.get_available_providers():
                    self.device = 'cpu'
        self.backend, self.retry_after = None, 0.0
        self.latest, self.processed = None, None
        self.count, self.ms, self.report = 0, 0.0, time.monotonic()
        self.pub = self.create_publisher(Image, output, qos_profile_sensor_data)
        self.create_subscription(CompressedImage, camera, self._image, qos_profile_sensor_data)
        self.create_timer(0.005, self._tick)
        self._load()

    def _load(self):
        try:
            self.backend = create_backend(self.backend_name, self.model_path,
                                          self.width, self.height, self.device)
            self.get_logger().info(f'{self.backend_name} {self.width}x{self.height}: {self.model_path}')
        except Exception as error:
            self.backend, self.retry_after = None, time.monotonic() + self.retry
            self.get_logger().error(f'model load failed ({error}); retry in {self.retry:.0f}s')

    def _image(self, message):
        self.latest = message

    def _tick(self):
        message = self.latest
        if message is None or message is self.processed:
            return
        if self.backend is None:
            if time.monotonic() >= self.retry_after:
                self._load()
            return
        self.processed = message  # newest frame only; older ones are skipped
        start = time.perf_counter()
        try:
            bgr = cv2.imdecode(np.frombuffer(message.data, np.uint8), cv2.IMREAD_COLOR)
            if bgr is None:
                raise ValueError('camera decode failed')
            mask = decode(self.backend.logits(preprocess(bgr, self.width, self.height)),
                          (self.width, self.height), self.threshold)
        except Exception as error:
            self.get_logger().warning(f'inference failed: {error}', throttle_duration_sec=2.0)
            return
        out = Image()
        out.header = message.header
        out.height, out.width = mask.shape
        out.encoding, out.step = 'mono8', mask.shape[1]
        out.data = mask.tobytes()
        self.pub.publish(out)
        self.count += 1
        self.ms += (time.perf_counter() - start) * 1000
        now = time.monotonic()
        if now - self.report >= 5.0:
            self.get_logger().info(f'{self.count / (now - self.report):.1f} masks/s, '
                                   f'{self.ms / max(1, self.count):.1f} ms/frame')
            self.count, self.ms, self.report = 0, 0.0, now

    def destroy_node(self):
        if self.backend is not None and hasattr(self.backend, 'close'):
            self.backend.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = LaneDeepMaskNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        if rclpy.ok():  # a real error, not the SIGTERM shutdown race
            raise
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
