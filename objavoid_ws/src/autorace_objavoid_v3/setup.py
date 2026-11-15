from glob import glob
from setuptools import setup

package_name = 'autorace_objavoid_v3'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ms-c-h',
    description='AutoRace obstacle avoidance V3',
    license='MIT',
    tests_require=['pytest'],
    entry_points={'console_scripts': ['objavoid_v3_node = autorace_objavoid_v3.node:main']},
)
