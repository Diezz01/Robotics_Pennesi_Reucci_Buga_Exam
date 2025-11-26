from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import GroupAction
from launch_ros.actions import PushRosNamespace

def generate_launch_description():
    return LaunchDescription([
        
        GroupAction([
            PushRosNamespace('robot_0'),
            Node(
                package='space_project',
                executable='astar_navigation_node',
                name='controller'
            )
        ]),

        GroupAction([
            PushRosNamespace('robot_1'),
            Node(
                package='space_project',
                executable='astar_navigation_node',
                name='controller'
            )
        ]),
    ])
