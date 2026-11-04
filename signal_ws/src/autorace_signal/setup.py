from setuptools import find_packages, setup

setup(
    name='autorace_signal', version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/autorace_signal']),
        ('share/autorace_signal', ['package.xml']),
        ('share/autorace_signal/launch', ['launch/signal.launch.py']),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='Minjea31', maintainer_email='kimminjea31@naver.com',
    description='Confirmed traffic light stops with lane command preservation.',
    license='Apache-2.0',
    entry_points={'console_scripts': ['signal_node = autorace_signal.node:main']},
)
