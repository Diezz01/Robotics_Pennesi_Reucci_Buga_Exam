"""
Battery Management System Launch File
Launches all necessary nodes for the battery-aware navigation system:
- A* navigation node
- Battery manager
- ROS-TCP-Endpoint for Unity communication
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # Launch arguments
    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        #default_value='tb3_0',
        description='Name of the robot (used for topic namespacing)'
    )

    low_battery_threshold_arg = DeclareLaunchArgument(
        'low_battery_threshold',
        default_value='30.0'
    )

    critical_battery_threshold_arg = DeclareLaunchArgument(
        'critical_battery_threshold',
        default_value='15.0'
    )

    charged_threshold_arg = DeclareLaunchArgument(
        'charged_threshold',
        default_value='95.0'
    )

    metrics_interval_arg = DeclareLaunchArgument(
        'publish_metrics_interval',
        default_value='5.0'
    )

    # Launch configurations
    robot_name = LaunchConfiguration('robot_name')
    low_battery_threshold = LaunchConfiguration('low_battery_threshold')
    critical_battery_threshold = LaunchConfiguration('critical_battery_threshold')
    charged_threshold = LaunchConfiguration('charged_threshold')
    metrics_interval = LaunchConfiguration('publish_metrics_interval')

    # A* Navigation Node
    astar_navigation_node = Node(
        package='space_project',
        executable='astar_navigation_node',
        name='astar_navigation',
        namespace=robot_name,
        output='screen'
    )

    # Battery Manager Node
    battery_manager_node = Node(
        package='space_project',
        executable='battery_manager',
        name='battery_manager',
        namespace=robot_name,
        output='screen',
        parameters=[{
            'robot_name': robot_name,
            'low_battery_threshold': low_battery_threshold,
            'critical_battery_threshold': critical_battery_threshold,
            'charged_threshold': charged_threshold,
            'publish_metrics_interval': metrics_interval,
        }]
    )

    # ROS TCP Endpoint
    ros_tcp_endpoint = Node(
        package='ros_tcp_endpoint',
        executable='default_server_endpoint',
        name='ros_tcp_endpoint',
        output='screen',
        parameters=[{
            'ROS_IP': '0.0.0.0',
            'ROS_TCP_PORT': 10000,
        }]
    )

    # Launch info
    launch_info = LogInfo(msg=[
        '',
        '=' * 60,
        'Battery Management System Launched',
        '=' * 60,
        'Robot Name: ', robot_name,
        'Low Battery Threshold: ', low_battery_threshold, '%',
        'Critical Battery Threshold: ', critical_battery_threshold, '%',
        'Charged Threshold: ', charged_threshold, '%',
        'Metrics Interval: ', metrics_interval, ' s',
        '',
    ])

    return LaunchDescription([
        robot_name_arg,
        low_battery_threshold_arg,
        critical_battery_threshold_arg,
        charged_threshold_arg,
        metrics_interval_arg,
        ros_tcp_endpoint,
        astar_navigation_node,
        battery_manager_node,
        launch_info,
    ])
