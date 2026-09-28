from setuptools import setup
from glob import glob
setup(name='robot_base', version='0.1.0', packages=['robot_base'],
      tests_require=['pytest'],
      data_files=[('share/ament_index/resource_index/packages', ['resource/robot_base']),
                  ('share/robot_base', ['package.xml']),
                  ('share/robot_base/launch', glob('launch/*')),
                  ('share/robot_base/description', glob('description/*')),
                  ('share/robot_base/config', glob('config/*')),
                  ('share/robot_base/web', glob('web/*'))],
      entry_points={'console_scripts': ['base = robot_base.node:main']})
