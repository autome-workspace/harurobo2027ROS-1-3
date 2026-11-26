from glob import glob
from setuptools import setup

package_name = 'ball_auto_approach'
nodes = ('usb_camera', 'ball_detector', 'ball_localizer', 'approach_controller')
setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*')),
    ],
    install_requires=['setuptools'],
    entry_points={'console_scripts': [f'{name} = {package_name}.{name}:main' for name in nodes]},
)
