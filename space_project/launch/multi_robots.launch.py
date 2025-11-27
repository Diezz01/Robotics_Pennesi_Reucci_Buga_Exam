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
                name='controller',
                parameters=[
                    {'src_x': 49.0},
                    {'src_y': 49.0},
                    {'dest_x': 40.0},
                    {'dest_y': 40.0}
                ]
            )
        ]),

        GroupAction([
            PushRosNamespace('robot_1'),
            Node(
                package='space_project',
                executable='astar_navigation_node',
                name='controller',
                parameters=[
                    {'src_x': 20.0},
                    {'src_y': 20.0},
                    {'dest_x': 10.0},
                    {'dest_y': 10.0}
                ]
            )
        ]),
    ])