from glob import glob

from setuptools import find_packages, setup

setup(
    name='autorace_lane_fusion_deep', version='0.1.0', packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/autorace_lane_fusion_deep']),
        ('share/autorace_lane_fusion_deep', ['package.xml']),
        ('share/autorace_lane_fusion_deep/launch', ['launch/deep_mask.launch.py']),
        ('share/autorace_lane_fusion_deep/model', glob('model/*.onnx') + glob('model/*.json')),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='ms-c-h', maintainer_email='alstjd5571@cbnu.ac.kr',
    description='lanedeep U-Net segmentation-only node for lane_fusion gated fusion.',
    license='Apache-2.0',
    entry_points={'console_scripts': ['lane_fusion_deep_mask = autorace_lane_fusion_deep.mask_node:main']},
)
