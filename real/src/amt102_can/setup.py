from glob import glob
from setuptools import setup

setup(name='amt102_can',version='0.1.0',packages=['amt102_can'],
      data_files=[('share/ament_index/resource_index/packages',['resource/amt102_can']),
                  ('share/amt102_can',['package.xml','README.md']),
                  ('share/amt102_can/config',glob('config/*')),
                  ('share/amt102_can/launch',glob('launch/*.launch.py'))],
      install_requires=['setuptools'],tests_require=['pytest'],
      entry_points={'console_scripts':['encoder_node = amt102_can.encoder_node:main',
                                      'encoder_visualizer = amt102_can.encoder_visualizer:main']})
