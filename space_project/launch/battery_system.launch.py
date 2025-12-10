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
    # Declare launch arguments
    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='tb3_0',
        description='Name of the robot (used for topic namespacing)'
    )

    low_battery_threshold_arg = DeclareLaunchArgument(
        'low_battery_threshold',
        default_value='30.0',
        description='Battery level (%) to trigger low battery warning'
    )

    critical_battery_threshold_arg = DeclareLaunchArgument(
        'critical_battery_threshold',
        default_value='15.0',
        description='Battery level (%) to trigger critical battery alert'
    )

    charged_threshold_arg = DeclareLaunchArgument(
        'charged_threshold',
        default_value='95.0',
        description='Battery level (%) to consider charging complete'
    )

    metrics_interval_arg = DeclareLaunchArgument(
        'publish_metrics_interval',
        default_value='5.0',
        description='Interval (seconds) for publishing battery health metrics'
    )

    # Get launch configuration values
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
        output='screen',
        parameters=[],
        remappings=[
            # Add remappings if needed
        ]
    )

    # Battery Manager Node
    battery_manager_node = Node(
        package='space_project',
        executable='battery_manager',
        name='battery_manager',
        output='screen',
        parameters=[{
            'robot_name': robot_name,
            'low_battery_threshold': low_battery_threshold,
            'critical_battery_threshold': critical_battery_threshold,
            'charged_threshold': charged_threshold,
            'publish_metrics_interval': metrics_interval
        }]
    )

    # ROS-TCP-Endpoint for Unity communication
    ros_tcp_endpoint = Node(
        package='ros_tcp_endpoint',
        executable='default_server_endpoint',
        name='ros_tcp_endpoint',
        output='screen',
        parameters=[{
            'ROS_IP': '0.0.0.0',  # Listen on all interfaces
            #'ROS_IP': '192.168.1.13',  # Listen on all interfaces
            'ROS_TCP_PORT': 10000
        }]
    )

    # Launch info
    launch_info = LogInfo(
        msg=[
            '\n',
            '='*60, '\n',
            'Battery Management System Launched\n',
            '='*60, '\n',
            'Robot Name: ', robot_name, '\n',
            'Low Battery Threshold: ', low_battery_threshold, '%\n',
            'Critical Battery Threshold: ', critical_battery_threshold, '%\n',
            'Charged Threshold: ', charged_threshold, '%\n',
            'Metrics Interval: ', metrics_interval, 's\n',
            '='*60, '\n',
            '\nMonitor battery with:\n',
            '  ros2 topic echo /', robot_name, '/battery_state\n',
            '  ros2 topic echo /', robot_name, '/battery_health\n',
            '\n'
        ]
    )

    return LaunchDescription([
        # Arguments
        robot_name_arg,
        low_battery_threshold_arg,
        critical_battery_threshold_arg,
        charged_threshold_arg,
        metrics_interval_arg,

        # Info message
        launch_info,

        # Nodes
        ros_tcp_endpoint,
        astar_navigation_node,
        battery_manager_node,
    ])
