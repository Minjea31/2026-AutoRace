from glob import glob
from setuptools import find_packages, setup

setup(
    name='limo_parking_sim',version='0.1.0',packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',['resource/limo_parking_sim']),
        ('share/limo_parking_sim',['package.xml']),
        ('share/limo_parking_sim/config',glob('config/*')),
        ('share/limo_parking_sim/assets',glob('assets/*')),
        ('share/limo_parking_sim/launch',glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools','numpy<2','PyYAML','Pillow','panda3d==1.10.16'],
    zip_safe=False,maintainer='AutoRace Team',maintainer_email='autorace@example.com',
    description='AutoRace LIMO Pro Panda3D mission simulator',license='Apache-2.0',
    entry_points={'console_scripts':[
        'limo_parking_sim = limo_parking_sim.app:main',
        'parking_sim_adapter = limo_parking_sim.parking_adapter:main',
        'mission_sim_adapter = limo_parking_sim.mission_adapter:main',
    ]},
)
