from setuptools import find_packages, setup

setup(
    name='cone_lane_follower', version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/cone_lane_follower']),
        ('share/cone_lane_follower', ['package.xml']),
        ('share/cone_lane_follower/launch', ['launch/roi.launch.py']),
        ('share/cone_lane_follower/rviz', ['rviz/roi.rviz']),
        ('share/cone_lane_follower/config', ['config/clustering.yaml']),
    ],
    install_requires=['setuptools'], zip_safe=True,
    tests_require=['pytest'],
    maintainer='Minjea31', maintainer_email='kimminjea31@naver.com',
    description='LiDAR ROI filtering, scan clustering, and cone candidate visualization.',
    license='Apache-2.0',
    entry_points={'console_scripts': ['roi_node = cone_lane_follower.node:main']},
)
