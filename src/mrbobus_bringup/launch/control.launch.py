"""Standard ODrive/diff-drive stack. Drive controller starts INACTIVE.

can0 requires stopping the old robot-base first; its CAN lock is respected.
"""
from pathlib import Path
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def launch_nodes(context):
    interface = LaunchConfiguration('can').perform(context)
    if interface not in ('can0', 'vcan0'):
        raise RuntimeError('Choose can0 (robot) or vcan0 (test).')
    required_domain = '42' if interface == 'can0' else '43'
    if os.environ.get('ROS_DOMAIN_ID') != required_domain:
        raise RuntimeError(f'{interface} requires ROS_DOMAIN_ID={required_domain}')
    lock = f'/run/robot-base/robot-base-{interface}.lock' if interface == 'can0' else '/tmp/robot-base-vcan0.lock'
    share = Path(get_package_share_directory('mrbobus_bringup'))
    description = xacro.process_file(str(share / 'description/control.urdf.xacro'),
                                    mappings={'can': interface, 'lock_path': lock}).toxml()
    state = Node(package='robot_state_publisher', executable='robot_state_publisher',
                 parameters=[{'robot_description': description}])
    control = Node(package='controller_manager', executable='ros2_control_node',
                   parameters=[str(share / 'config/controllers.yaml'), str(share / 'config/bench.yaml')],
                   remappings=[('/diff_drive_controller/odom', '/odom')], output='screen')
    joints = Node(package='controller_manager', executable='spawner',
                  arguments=['joint_state_broadcaster'])
    drive = Node(package='controller_manager', executable='spawner',
                 arguments=['diff_drive_controller', '--inactive'])
    return [state, control, joints, RegisterEventHandler(OnProcessExit(target_action=joints, on_exit=[drive]))]


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('can', default_value='vcan0'), OpaqueFunction(function=launch_nodes)])
