#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/jazzy/setup.bash
source /home/taras/robot/mrbobus_ws/install/setup.bash
exec ros2 launch /home/taras/robot/mrbobus_ws/deploy/navigation.launch.py
