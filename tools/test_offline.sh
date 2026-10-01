#!/usr/bin/env bash
# No ROS service, real CAN, SSH or motion is started here.
set -euo pipefail
cd "$(dirname "$0")/.."
export PYTHONPATH="$PWD/src/robot_base:$PWD/src/mrbobus_console:$PWD/tools${PYTHONPATH:+:$PYTHONPATH}"
python3 -m unittest discover -s src/robot_base/test -v
for suite in control field_map ground mission radio routing; do
  python3 -m unittest discover -s src/mrbobus_console/tests -p "test_${suite}.py" -v
done
python3 -m unittest discover -s tools -p 'test_crsf*.py' -v
python3 -m unittest discover -s tools -p 'test_radio_policy.py' -v
