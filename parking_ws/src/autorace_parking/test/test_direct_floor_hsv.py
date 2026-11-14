"""Parking uses camera pixels only, isolated from the vehicle DDS domain."""
import time
from types import SimpleNamespace

import cv2
import numpy as np
import pytest
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool, Float64MultiArray

from test_start_and_wheel_departure import node


@pytest.fixture
def camera(node):
    parking,_=node
    driver=Node('parking_direct_hsv_test_camera',context=parking.context)
    executor=SingleThreadedExecutor(context=parking.context)
    executor.add_node(parking);executor.add_node(driver)
    images=driver.create_publisher(CompressedImage,'/camera/color/image_raw/compressed',10)
    old_floor=driver.create_publisher(Float64MultiArray,'/parking/floor_observation',10)
    starts=[]
    gate=parking.floor_gate
    gate.start=lambda:starts.append(True)

    def spin(duration):
        deadline=time.monotonic()+duration
        while time.monotonic()<deadline:executor.spin_once(timeout_sec=.005)

    def image(frame,age=0.):
        ok,encoded=cv2.imencode('.jpg',frame)
        assert ok
        message=CompressedImage(format='jpeg',data=encoded.tobytes())
        stamp=driver.get_clock().now().nanoseconds-int(age*1e9)
        message.header.stamp.sec,message.header.stamp.nanosec=divmod(stamp,1_000_000_000)
        images.publish(message)
        spin(.14)

    deadline=time.monotonic()+3.
    while not images.get_subscription_count():
        assert time.monotonic()<deadline
        spin(.02)
    try:
        yield SimpleNamespace(parking=parking,gate=gate,old_floor=old_floor,
                              starts=starts,spin=spin,image=image)
    finally:
        executor.remove_node(parking);executor.remove_node(driver)
        executor.shutdown();driver.destroy_node()


def arm(camera):
    camera.gate.on_sign(Bool(data=True));camera.gate.on_sign(Bool(data=True))
    assert camera.gate.armed


def test_external_white_ratio_cannot_start_or_delay_camera_detection(camera):
    arm(camera)
    stamp=camera.gate.clock()
    sec,nanosec=divmod(stamp,1_000_000_000)
    camera.old_floor.publish(Float64MultiArray(data=[float(sec),float(nanosec),1.,0.,0.]))
    camera.spin(.12)
    assert not camera.starts and camera.gate.armed
    assert camera.parking.count_subscribers('/parking/floor_observation')==0
    camera.image(np.zeros((480,640,3),np.uint8))
    assert not camera.starts and camera.gate.ratio==0.
    # A recent external observation must not suppress parking's own image.
    camera.image(np.full((480,640,3),255,np.uint8))
    assert camera.starts==[True] and not camera.gate.armed


def test_only_fresh_white_pixels_inside_parking_roi_start_the_mission(camera):
    # White images captured before sign confirmation are never cached as a trigger.
    camera.image(np.full((480,640,3),255,np.uint8))
    assert not camera.starts
    arm(camera)
    roi=camera.gate.roi
    x0,x1=int(640*roi.left),int(np.ceil(640*roi.right))
    y0,y1=int(480*roi.top),int(np.ceil(480*roi.bottom))
    frame=np.full((480,640,3),255,np.uint8)
    frame[y0:y1,x0:x1]=0
    camera.image(frame)
    assert camera.gate.ratio==0. and not camera.starts  # white outside the ROI
    frame[y0:y1,x0:x1]=(0,255,255)
    camera.image(frame)
    assert camera.gate.ratio==0. and not camera.starts  # bright yellow is not white
    frame[y0:y1,x0:x1]=255
    camera.image(frame,age=1.)
    camera.image(frame,age=-1.)
    assert not camera.starts
    camera.image(frame)
    assert camera.starts==[True]
