#!/usr/bin/env python3
"""
Battery Manager Node
Monitors battery level from Unity and manages charging behavior:
- Subscribes to battery state from Unity
- Detects low battery (< 20%)
- Navigates robot to charging station
- Resumes mission when fully charged
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
import math


class BatteryManager(Node):
    def __init__(self):
        super().__init__('battery_manager')

        # Parameters - adjust these for your setup
        self.declare_parameter('robot_name', 'tb3_0')
        self.declare_parameter('low_battery_threshold', 20.0)
        self.declare_parameter('charged_threshold', 95.0)
        self.declare_parameter('charging_station_x', 0.0)  # Set your charging station coordinates
        self.declare_parameter('charging_station_y', 0.0)
        self.declare_parameter('charging_station_z', 0.0)

        robot_name = self.get_parameter('robot_name').value
        self.low_battery_threshold = self.get_parameter('low_battery_threshold').value
        self.charged_threshold = self.get_parameter('charged_threshold').value

        # Charging station position
        self.charging_station = (
            self.get_parameter('charging_station_x').value,
            self.get_parameter('charging_station_y').value,
            self.get_parameter('charging_station_z').value
        )

        # State management
        self.battery_level = 100.0
        self.is_charging = False
        self.is_low_battery = False
        self.saved_goal = None  # Store interrupted goal to resume later
        self.current_position = None

        # Subscribers
        self.battery_sub = self.create_subscription(
            Float32,
            f'/{robot_name}/battery_state',
            self.battery_callback,
            10
        )

        self.pose_sub = self.create_subscription(
            Odometry,
            f'/{robot_name}/odom',
            self.pose_callback,
            10
        )

        # Publishers
        self.goal_pub = self.create_publisher(
            PoseStamped,
            f'/{robot_name}/goal_pose',
            10
        )

        self.cmd_vel_pub = self.create_publisher(
            Twist,
            f'/{robot_name}/cmd_vel',
            10
        )

        # Timer for state management
        self.timer = self.create_timer(1.0, self.check_battery_state)

        self.get_logger().info(f'Battery Manager initialized for {robot_name}')
        self.get_logger().info(f'Charging station at: {self.charging_station}')
        self.get_logger().info(f'Low battery threshold: {self.low_battery_threshold}%')

    def battery_callback(self, msg):
        """Receive battery level from Unity"""
        self.battery_level = msg.data

    def pose_callback(self, msg):
        """Track current robot position"""
        self.current_position = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        )

    def check_battery_state(self):
        """Main state machine for battery management"""

        # Check if battery is low and not already heading to charge
        if self.battery_level <= self.low_battery_threshold and not self.is_low_battery:
            self.get_logger().warn(
                f'⚠️ Low battery detected: {self.battery_level:.1f}%'
            )
            self.is_low_battery = True
            self.navigate_to_charging_station()

        # Check if at charging station and battery low
        elif self.is_low_battery and not self.is_charging:
            if self.is_at_charging_station():
                self.get_logger().info('🔌 Arrived at charging station, waiting for charge...')
                self.is_charging = True
                self.stop_robot()

        # Check if battery is charged
        elif self.is_charging and self.battery_level >= self.charged_threshold:
            self.get_logger().info(
                f'🔋 Battery charged to {self.battery_level:.1f}%, resuming mission'
            )
            self.is_charging = False
            self.is_low_battery = False
            self.resume_mission()

        # Log battery status periodically
        if int(self.battery_level) % 10 == 0:  # Log every 10%
            status = "🔌 Charging" if self.is_charging else "🔋 Operating"
            self.get_logger().info(
                f'{status} - Battery: {self.battery_level:.1f}%'
            )

    def navigate_to_charging_station(self):
        """Send navigation goal to charging station"""
        self.get_logger().info(
            f'🚀 Navigating to charging station at {self.charging_station}'
        )

        goal_msg = PoseStamped()
        goal_msg.header.frame_id = 'map'
        goal_msg.header.stamp = self.get_clock().now().to_msg()

        goal_msg.pose.position.x = self.charging_station[0]
        goal_msg.pose.position.y = self.charging_station[1]
        goal_msg.pose.position.z = self.charging_station[2]
        goal_msg.pose.orientation.w = 1.0

        self.goal_pub.publish(goal_msg)

    def is_at_charging_station(self):
        """Check if robot is at charging station"""
        if self.current_position is None:
            return False

        dx = self.current_position[0] - self.charging_station[0]
        dy = self.current_position[1] - self.charging_station[1]
        dz = self.current_position[2] - self.charging_station[2]

        distance = math.sqrt(dx**2 + dy**2 + dz**2)
        return distance < 0.5  # Within 0.5 meters

    def stop_robot(self):
        """Stop robot movement"""
        stop_msg = Twist()
        self.cmd_vel_pub.publish(stop_msg)

    def resume_mission(self):
        """Resume previous mission after charging"""
        if self.saved_goal is not None:
            self.get_logger().info('Resuming previous mission goal')
            self.goal_pub.publish(self.saved_goal)
            self.saved_goal = None
        else:
            self.get_logger().info('No saved goal, waiting for new commands')

    def save_current_goal(self, goal):
        """Save current goal before interrupting for charging"""
        self.saved_goal = goal


def main(args=None):
    rclpy.init(args=args)
    node = BatteryManager()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
