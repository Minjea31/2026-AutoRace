"""Publish a visual-only LIMO Pro URDF for the parking RViz view.

All five visuals belong to base_link, so no joint states or extra TF publishers
are needed. Reuse the same source meshes as the simulator, including materials.
"""
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String


def mesh_directory():
    # Resolve symlink installs back to the repository; no sensor build required.
    for parent in Path(__file__).resolve().parents:
        candidate = parent / 'sensor_ws/src/limo_ros2/limo_description/meshes'
        if (candidate / 'limo_base.dae').is_file() and (candidate / 'limo_wheel.dae').is_file():
            return candidate
    from ament_index_python.packages import get_package_share_directory
    return Path(get_package_share_directory('limo_description')) / 'meshes'


def robot_description():
    meshes = mesh_directory()
    for name in ('limo_base.dae', 'limo_wheel.dae'):
        if not (meshes / name).is_file():
            raise FileNotFoundError(meshes / name)
    robot = ET.Element('robot', name='limo_pro_parking_visual')
    link = ET.SubElement(robot, 'link', name='base_link')
    def visual(name, mesh, xyz, rpy, scale):
        element = ET.SubElement(link, 'visual', name=name)
        ET.SubElement(element, 'origin', xyz=' '.join(map(str, xyz)), rpy=' '.join(map(str, rpy)))
        geometry = ET.SubElement(element, 'geometry')
        ET.SubElement(geometry, 'mesh', filename=(meshes / mesh).as_uri(), scale=' '.join(map(str, scale)))

    # Source bounds in metres; match build_limo_model.py's LIMO Pro envelope:
    # 322 mm length, body z=24..251 mm, 200 mm wheelbase, 175 mm track.
    body_y_scale = .322 / (.1630476 + .1585)
    body_z_scale = .227 / (.2535344 - .02437367)
    visual('body', 'limo_base.dae',
           ((.1630476 - .1585) / 2 * body_y_scale, 0., .024 - .02437367 * body_z_scale - .15),
           (0., 0., math.pi / 2), (1., body_y_scale, body_z_scale))
    radius_scale = .045 / .04814471
    width_scale = .045 / (.04366261 + .031)
    wheel_center_y = (.04366261 - .031) / 2 * width_scale
    for x in (-.1, .1):
        for side in (-1, 1):
            visual(f'wheel_{x}_{side}', 'limo_wheel.dae',
                   (x, side * (.175 / 2 - wheel_center_y), .045 - .15),
                   (0. if side > 0 else math.pi, 0., 0.),
                   (radius_scale, width_scale, radius_scale))
    return ET.tostring(robot, encoding='unicode')


class RobotVisual(Node):
    def __init__(self):
        super().__init__('parking_robot_visual')
        self.publisher = self.create_publisher(
            String, '/parking/robot_description',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.publisher.publish(String(data=robot_description()))


def main(args=None):
    rclpy.init(args=args)
    node = RobotVisual()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
