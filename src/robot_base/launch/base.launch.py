from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    share = Path(get_package_share_directory('robot_base'))
    return LaunchDescription([
        DeclareLaunchArgument('config',default_value=str(share/'config/base.yaml')),
        Node(package='robot_base',executable='base',output='screen',
             parameters=[LaunchConfiguration('config')],
             on_exit=[EmitEvent(event=Shutdown(reason='Base process exited'))]),
        Node(package='robot_state_publisher',executable='robot_state_publisher',
             parameters=[{'robot_description':(share/'description/robot.urdf').read_text()}],
             on_exit=[EmitEvent(event=Shutdown(reason='Model publisher exited'))]),
    ])
