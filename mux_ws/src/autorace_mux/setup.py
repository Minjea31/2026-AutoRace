from glob import glob

from setuptools import find_packages, setup


setup(
    name='autorace_mux',
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/autorace_mux']),
        ('share/autorace_mux', ['package.xml']),
        ('share/autorace_mux/launch', glob('launch/*.launch.py')),
        ('share/autorace_mux/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wego',
    maintainer_email='wego@example.com',
    description='Priority mux with lane fallback and /cmd_vel output.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={'console_scripts': ['mux_node = autorace_mux.node:main']},
)
