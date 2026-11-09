import importlib.util
from pathlib import Path
import cv2
import numpy as np
from sensor_msgs.msg import Image

spec=importlib.util.spec_from_file_location('viewer_server',Path(__file__).parents[1]/'server.py')
viewer=importlib.util.module_from_spec(spec)
spec.loader.exec_module(viewer)


def test_class_ids_are_visible_without_modifying_input():
    pixels=np.zeros((48,96),np.uint8)
    pixels[:,32:64]=1;pixels[:,64:]=2
    m=Image();m.height=48;m.width=96;m.step=96;m.encoding='mono8';m.data=pixels.tobytes()
    before=bytes(m.data)
    jpeg,w,h=viewer.raw_jpeg(m,class_mask=True)
    image=cv2.imdecode(np.frombuffer(jpeg,np.uint8),cv2.IMREAD_COLOR)
    assert (w,h)==(96,48)
    assert image[24,16].max()<10
    assert image[24,48,1]>180 and image[24,48,2]>220
    assert image[24,80].min()>220
    assert bytes(m.data)==before


def test_plain_mono_keeps_intensity():
    m=Image();m.height=32;m.width=32;m.step=32;m.encoding='mono8';m.data=bytes([120])*1024
    jpeg,_,_=viewer.raw_jpeg(m)
    image=cv2.imdecode(np.frombuffer(jpeg,np.uint8),cv2.IMREAD_GRAYSCALE)
    assert abs(float(image.mean())-120)<2
