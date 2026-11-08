from setuptools import find_packages, setup

setup(
    name='autorace_slow', version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/autorace_slow']),
        ('share/autorace_slow', ['package.xml']),
        ('share/autorace_slow/launch', ['launch/slow.launch.py']),
        ('share/autorace_slow/config', ['config/slow.yaml']),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='Minjea31', maintainer_email='kimminjea31@naver.com',
    description='Straight slow missions and lane-following speed bump control.',
    license='Apache-2.0',
    entry_points={'console_scripts': ['slow_node = autorace_slow.node:main']},
)
