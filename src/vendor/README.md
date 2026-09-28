# ODrive upstream subset

`odrive_base` and `odrive_ros2_control` are vendored from
https://github.com/odriverobotics/ros_odrive at
`efe8f775f1c1ef1aec4dd23f6f8b2ccdd5239652` (MIT; see LICENSE.odrive).
Only the hardware plugin and its shared CAN transport are included.

Local changes target ODrive v3.6 firmware 0.5.6, velocity mode:
physical-forward joint sign, legacy heartbeat and feedback freshness,
all-axis Idle on faults/deactivation, CAN send/read error propagation,
Hall position continuity across Idle/re-arm, configurable current limit (bench default1A),
no interpretation of legacy ADC 0x1C as torque, one CAN owner lock.
Differential kinematics/odometry remain in the standard diff_drive_controller.
