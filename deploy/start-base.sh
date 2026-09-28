#!/bin/bash
set -eo pipefail
source /opt/ros/jazzy/setup.bash
source /home/taras/robot/ros2_ws/install/setup.bash
exec ros2 launch robot_base base.launch.py config:=/home/taras/robot/ros2_ws/src/robot_base/config/base.yaml
