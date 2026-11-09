from setuptools import setup

setup(
    name='autorace_objavoid', version='0.1.0',
    packages=['autorace_objavoid'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/autorace_objavoid']),
        ('share/autorace_objavoid', ['package.xml']),
        ('share/autorace_objavoid/config', ['config/objavoid.yaml', 'config/objavoid.rviz', 'config/camera_projection.yaml']),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='Minjea31', maintainer_email='kimminjea31@naver.com',
    description='Two-obstacle LiDAR avoidance with deep lane corridor guidance.',
    license='Apache-2.0',
    entry_points={'console_scripts': ['objavoid_node = autorace_objavoid.node:main']},
)
