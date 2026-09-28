#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/jazzy/setup.bash
source /home/taras/robot/mrbobus_ws/install/setup.bash
exec ros2 launch mrbobus_bringup control.launch.py can:=can0 current_limit:="${DRIVE_CURRENT_LIMIT:-15.0}" profile:=floor gain_profile:="${DRIVE_GAIN_PROFILE:-baseline}"
