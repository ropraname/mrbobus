#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/jazzy/setup.bash
source /home/taras/robot/mrbobus_ws/install/setup.bash
source /home/taras/robot/lio_ws/install/setup.bash
export PYTHONPATH=/home/taras/robot/mrbobus_ws/src/mrbobus_console:${PYTHONPATH:-}
# Restore the verified overview-camera controls after reconnect/reboot.
v4l2-ctl -d /dev/video0 --set-ctrl=auto_exposure=3,gamma=300,contrast=52,sharpness=50 >/dev/null 2>&1 || true
exec python3 -m mrbobus_console.server
