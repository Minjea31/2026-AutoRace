from glob import glob
from setuptools import find_packages, setup

setup(
    name='autorace_parking', version='0.1.0', packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/autorace_parking']),
        ('share/autorace_parking', ['package.xml']),
        ('share/autorace_parking/config', glob('config/*')),
        ('share/autorace_parking/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools', 'numpy'], zip_safe=True,
    maintainer='AutoRace Team', maintainer_email='autorace@example.com',
    description='Parking activation and LiDAR obstacle/path visualization',
    license='Apache-2.0',
    entry_points={'console_scripts': ['parking_node = autorace_parking.node:main']},
)
