#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/jazzy/setup.bash
source /home/taras/robot/mrbobus_ws/install/setup.bash
source /home/taras/robot/lio_ws/install/setup.bash
export PYTHONPATH=/home/taras/robot/mrbobus_ws/src/mrbobus_console:${PYTHONPATH:-}
exec python3 -m mrbobus_console.server
