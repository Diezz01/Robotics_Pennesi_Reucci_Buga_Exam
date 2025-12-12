#!/usr/bin/env python3
"""
Battery Manager Node - Monitoring and Health Metrics
Monitors battery level from Unity and publishes health metrics.
Does NOT control navigation (that's ExplorerController's job).
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool, String
import json
from datetime import datetime


class BatteryManager(Node):
    def __init__(self):
        super().__init__('battery_manager')

        # Parameters
        self.declare_parameter('robot_name', 'tb3_0')
        self.declare_parameter('low_battery_threshold', 30.0)
        self.declare_parameter('critical_battery_threshold', 15.0)
        self.declare_parameter('charged_threshold', 95.0)
        self.declare_parameter('publish_metrics_interval', 5.0)  # seconds

        robot_name = self.get_parameter('robot_name').value
        self.low_battery_threshold = self.get_parameter('low_battery_threshold').value
        self.critical_battery_threshold = self.get_parameter('critical_battery_threshold').value
        self.charged_threshold = self.get_parameter('charged_threshold').value
        metrics_interval = self.get_parameter('publish_metrics_interval').value

        # State tracking
        self.battery_level = 100.0
        self.is_charging = False
        self.battery_history = []  # For calculating drain rate
        self.max_history_size = 100

        # Alert states
        self.low_battery_alert_sent = False
        self.critical_battery_alert_sent = False
        self.charged_alert_sent = False

        # Subscribers
        self.battery_sub = self.create_subscription(
            Float32,
            f'/{robot_name}/battery_state',
            self.battery_callback,
            10
        )

        self.charging_status_sub = self.create_subscription(
            Bool,
            f'/{robot_name}/charging_status',
            self.charging_status_callback,
            10
        )

        # Publishers
        self.battery_health_pub = self.create_publisher(
            String,
            f'/{robot_name}/battery_health',
            10
        )

        self.battery_alert_pub = self.create_publisher(
            String,
            f'/{robot_name}/battery_alert',
            10
        )

        # Timer for periodic metrics
        self.metrics_timer = self.create_timer(
            metrics_interval,
            self.publish_battery_metrics
        )

        self.get_logger().info(f'Battery Manager initialized for {robot_name}')
        self.get_logger().info(f'Monitoring mode: Low threshold={self.low_battery_threshold}%, Critical={self.critical_battery_threshold}%')

    def battery_callback(self, msg):
        """Receive battery level from Unity"""
        self.battery_level = msg.data

        # Track battery history with timestamp
        current_time = self.get_clock().now().nanoseconds / 1e9
        self.battery_history.append({
            'time': current_time,
            'level': self.battery_level,
            'charging': self.is_charging
        })

        # Limit history size
        if len(self.battery_history) > self.max_history_size:
            self.battery_history.pop(0)

        # Check for alert conditions
        self.check_battery_alerts()

    def charging_status_callback(self, msg):
        """Receive charging status from Unity"""
        was_charging = self.is_charging
        self.is_charging = msg.data

        if not was_charging and self.is_charging:
            self.get_logger().info(f'🔌 Charging started at {self.battery_level:.1f}%')
            self.publish_alert('INFO', 'Charging started')
        elif was_charging and not self.is_charging:
            self.get_logger().info(f'🔋 Charging stopped at {self.battery_level:.1f}%')
            self.publish_alert('INFO', 'Charging stopped')
            # Allow announcing "charged" again on the next charging cycle.
            self.charged_alert_sent = False

    def check_battery_alerts(self):
        """Check battery level and send alerts"""

        # Critical battery alert
        if self.battery_level <= self.critical_battery_threshold and not self.is_charging:
            if not self.critical_battery_alert_sent:
                self.get_logger().error(
                    f'🚨 CRITICAL BATTERY: {self.battery_level:.1f}%'
                )
                self.publish_alert('CRITICAL', f'Battery critically low: {self.battery_level:.1f}%')
                self.critical_battery_alert_sent = True
        else:
            self.critical_battery_alert_sent = False

        # Low battery warning
        if self.battery_level <= self.low_battery_threshold and not self.is_charging:
            if not self.low_battery_alert_sent:
                self.get_logger().warn(
                    f'⚠️ LOW BATTERY: {self.battery_level:.1f}%'
                )
                self.publish_alert('WARNING', f'Battery low: {self.battery_level:.1f}%')
                self.low_battery_alert_sent = True
        else:
            self.low_battery_alert_sent = False

        # Charged notification
        if self.battery_level >= self.charged_threshold and self.is_charging:
            if not self.charged_alert_sent:
                self.get_logger().info(f'✅ Battery charged to {self.battery_level:.1f}%')
                self.charged_alert_sent = True
        else:
            # Reset if we drop below the threshold (or stop charging) so we can
            # announce again after re-reaching the charged threshold.
            self.charged_alert_sent = False

    def publish_battery_metrics(self):
        """Publish battery health metrics periodically"""

        drain_rate = self.calculate_drain_rate()
        estimated_time = self.estimate_remaining_time()

        metrics = {
            'timestamp': datetime.now().isoformat(),
            'battery_level': round(self.battery_level, 2),
            'is_charging': self.is_charging,
            'drain_rate_percent_per_second': round(drain_rate, 4),
            'estimated_remaining_seconds': estimated_time,
            'status': self.get_battery_status()
        }

        metrics_msg = String()
        metrics_msg.data = json.dumps(metrics)
        self.battery_health_pub.publish(metrics_msg)

        # Log metrics periodically
        self.get_logger().info(
            f'📊 Battery: {self.battery_level:.1f}% | '
            f'Status: {metrics["status"]} | '
            f'Drain: {drain_rate:.4f}%/s'
        )

    def calculate_drain_rate(self):
        """Calculate battery drain rate from history"""
        if len(self.battery_history) < 2:
            return 0.0

        # Only calculate drain rate when not charging
        non_charging_samples = [
            h for h in self.battery_history if not h['charging']
        ]

        if len(non_charging_samples) < 2:
            return 0.0

        # Calculate average drain over recent history
        time_diff = non_charging_samples[-1]['time'] - non_charging_samples[0]['time']
        level_diff = non_charging_samples[0]['level'] - non_charging_samples[-1]['level']

        if time_diff > 0:
            return level_diff / time_diff
        return 0.0

    def estimate_remaining_time(self):
        """Estimate remaining battery time in seconds"""
        if self.is_charging:
            return -1  # Infinite when charging

        drain_rate = self.calculate_drain_rate()
        if drain_rate <= 0:
            return -1

        return self.battery_level / drain_rate

    def get_battery_status(self):
        """Get human-readable battery status"""
        if self.is_charging:
            return "CHARGING"
        elif self.battery_level <= self.critical_battery_threshold:
            return "CRITICAL"
        elif self.battery_level <= self.low_battery_threshold:
            return "LOW"
        elif self.battery_level >= self.charged_threshold:
            return "FULL"
        else:
            return "NORMAL"

    def publish_alert(self, level, message):
        """Publish battery alert"""
        alert = {
            'timestamp': datetime.now().isoformat(),
            'level': level,
            'message': message,
            'battery_level': round(self.battery_level, 2)
        }

        alert_msg = String()
        alert_msg.data = json.dumps(alert)
        self.battery_alert_pub.publish(alert_msg)


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
