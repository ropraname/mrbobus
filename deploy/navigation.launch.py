"""Nav2 servers only. Commands go through the console's STOP-aware arbiter."""
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    params='/home/taras/robot/mrbobus_ws/src/mrbobus_console/config/navigation.yaml'
    specs=[('nav2_map_server','map_server'),('nav2_planner','planner_server'),('nav2_controller','controller_server'),('nav2_bt_navigator','bt_navigator')]
    nodes=[Node(package=p,executable=n,name=n,parameters=[params],remappings=[('cmd_vel','/navigation/cmd_vel')],output='screen') for p,n in specs]
    nodes.append(Node(package='nav2_lifecycle_manager',executable='lifecycle_manager',name='lifecycle_manager_field',parameters=[{'autostart':True,'node_names':[n for _,n in specs]}],output='screen'))
    return LaunchDescription(nodes)
