"""Standard ODrive/diff-drive stack. Drive controller starts INACTIVE.

can0 requires stopping the old robot-base first; its CAN lock is respected.
"""
from pathlib import Path
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro
import yaml


def launch_nodes(context):
    interface = LaunchConfiguration('can').perform(context)
    if interface not in ('can0', 'vcan0'):
        raise RuntimeError('Choose can0 (robot) or vcan0 (test).')
    required_domain = '42' if interface == 'can0' else '43'
    if os.environ.get('ROS_DOMAIN_ID') != required_domain:
        raise RuntimeError(f'{interface} requires ROS_DOMAIN_ID={required_domain}')
    gain_profile = LaunchConfiguration('gain_profile').perform(context)
    profile = LaunchConfiguration('profile').perform(context)
    if profile not in ('bench', 'floor'): raise RuntimeError('Unknown control profile')
    lock = f'/run/robot-base/robot-base-{interface}.lock' if interface == 'can0' else '/tmp/robot-base-vcan0.lock'
    share = Path(get_package_share_directory('mrbobus_bringup'))
    if not (share / ('config/'+profile+'.yaml')).is_file():
        raise RuntimeError('Control profile is missing; rebuild mrbobus_bringup')
    profiles = yaml.safe_load((share / 'config/tuning.yaml').read_text())
    if gain_profile != 'existing' and gain_profile not in profiles:
        raise RuntimeError('Unknown gain profile')
    gains = {} if gain_profile == 'existing' else {k:str(v) for k,v in profiles[gain_profile].items()}
    description = xacro.process_file(str(share / 'description/control.urdf.xacro'),
                                    mappings={**gains, 'set_gains': 'false' if gain_profile == 'existing' else 'true', 'can': interface, 'lock_path': lock,
                                              'stop_file': '/run/mrbobus-stop/enabled' if interface == 'can0' else LaunchConfiguration('stop_file').perform(context),
                                              'max_velocity_rad_s': '4.5' if profile == 'floor' else '0.6283185307179586',
                                              'current_limit': LaunchConfiguration('current_limit').perform(context)}).toxml()
    state = Node(package='robot_state_publisher', executable='robot_state_publisher',
                 parameters=[{'robot_description': description}])
    control = Node(package='controller_manager', executable='ros2_control_node',
                   parameters=[str(share / 'config/controllers.yaml'), str(share / ('config/'+profile+'.yaml'))],
                   remappings=[('/diff_drive_controller/odom', '/odom')], output='screen')
    joints = Node(package='controller_manager', executable='spawner',
                  arguments=['joint_state_broadcaster'])
    drive = Node(package='controller_manager', executable='spawner',
                 arguments=['diff_drive_controller', '--inactive'])
    return [state, control, RegisterEventHandler(OnProcessExit(target_action=control, on_exit=[EmitEvent(event=Shutdown(reason='controller_manager exited'))])), joints, RegisterEventHandler(OnProcessExit(target_action=joints, on_exit=[drive]))]


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('can', default_value='vcan0'), DeclareLaunchArgument('gain_profile', default_value='existing'), DeclareLaunchArgument('profile', default_value='bench'), DeclareLaunchArgument('stop_file', default_value=''), DeclareLaunchArgument('current_limit', default_value='1.0'), OpaqueFunction(function=launch_nodes)])
