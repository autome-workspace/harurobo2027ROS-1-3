from glob import glob
from setuptools import setup

package = 'camera_lidar_calibration'
executables = ('calibration_node', 'calibration_gui', 'lidar_projection_node', 'calibration_tf_publisher')
setup(
    name=package, version='0.1.0', packages=[package],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package]),
        ('share/' + package, ['package.xml', 'README.md']),
        ('share/' + package + '/launch', glob('launch/*.launch.py')),
        ('share/' + package + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    entry_points={'console_scripts': [f'{name} = {package}.{name}:main' for name in executables]},
)
