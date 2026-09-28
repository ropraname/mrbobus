"""Consume the existing L2 driver; independent experimental LIO TF tree."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=PathJoinSubstitution([
            FindPackageShare('mrbobus_lio'), 'config', 'l2_pi.yaml'])),
        Node(package='point_lio', executable='pointlio_mapping', name='lio',
             output='screen', parameters=[LaunchConfiguration('config')],
             remappings=[('/aft_mapped_to_init', '/lio/odometry'),
                         ('/cloud_registered', '/lio/cloud_registered'),
                         ('/cloud_registered_body', '/lio/cloud_body'),
                         ('/cloud_effected', '/lio/cloud_effected'),
                         ('/Laser_map', '/lio/map_seed'), ('/path', '/lio/path')]),
    ])
