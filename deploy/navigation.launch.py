"""Nav2 servers only. Commands go through the console's STOP-aware arbiter."""
import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    params='/home/taras/robot/mrbobus_ws/src/mrbobus_console/config/navigation.yaml'
    specs=[('nav2_map_server','map_server'),('nav2_planner','planner_server'),('nav2_controller','controller_server'),('nav2_bt_navigator','bt_navigator')]
    map_root=os.environ.get('MRBOBUS_MAP_DIR','/home/taras/robot/records/maps/field-20260928')
    nodes=[Node(package=p,executable=n,name=n,parameters=[params]+([{'yaml_filename':os.environ.get('MRBOBUS_NAV_MAP',os.path.join(map_root,'field.yaml'))}] if n=='map_server' else []),remappings=[('cmd_vel','/navigation/cmd_vel')],output='screen') for p,n in specs]
    nodes.append(Node(package='nav2_lifecycle_manager',executable='lifecycle_manager',name='lifecycle_manager_field',parameters=[{'autostart':True,'node_names':[n for _,n in specs]}],output='screen'))
    return LaunchDescription(nodes)
