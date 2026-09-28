"""Standard four-wheel controller test. No motor driver is instantiated."""
from pathlib import Path
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    # Keep even virtual cmd_vel and TF away from the running robot (domain 42).
    if os.environ.get('ROS_DOMAIN_ID') != '43':
        raise RuntimeError('Run the mock test with ROS_DOMAIN_ID=43 (robot uses 42).')
    share = Path(get_package_share_directory('mrbobus_bringup'))
    description = xacro.process_file(str(share / 'description/control_mock.urdf.xacro')).toxml()
    state = Node(package='robot_state_publisher', executable='robot_state_publisher',
                 parameters=[{'robot_description': description}])
    control = Node(package='controller_manager', executable='ros2_control_node',
                   parameters=[str(share / 'config/controllers.yaml')], output='screen')
    joints = Node(package='controller_manager', executable='spawner',
                  arguments=['joint_state_broadcaster'])
    drive = Node(package='controller_manager', executable='spawner',
                 arguments=['diff_drive_controller'])
    return LaunchDescription([
        state, control, joints,
        RegisterEventHandler(OnProcessExit(target_action=joints, on_exit=[drive])),
    ])
