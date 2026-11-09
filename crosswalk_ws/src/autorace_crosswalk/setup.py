from setuptools import find_packages, setup

setup(
    name='autorace_crosswalk', version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/autorace_crosswalk']),
        ('share/autorace_crosswalk', ['package.xml']),
        ('share/autorace_crosswalk/launch', ['launch/crosswalk.launch.py']),
        ('share/autorace_crosswalk/config', ['config/crosswalk.yaml']),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='Minjea31', maintainer_email='kimminjea31@naver.com',
    description='Crosswalk stops with a ten-second cooldown, preserving lane steering.',
    license='Apache-2.0',
    entry_points={'console_scripts': ['crosswalk_node = autorace_crosswalk.node:main']},
)
