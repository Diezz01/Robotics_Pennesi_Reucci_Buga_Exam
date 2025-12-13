#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from space_project.srv import PathApproval
import math

class CollisionCoordinatorNode(Node):
    def __init__(self):
        super().__init__('collision_coordinator_node')

        # Parameters
        self.declare_parameter('num_robots', 6)
        self.declare_parameter('safe_distance', 4.0)  # Minimum distance between robots (increased for better safety)
        self.declare_parameter('path_buffer', 3.0)    # Buffer zone for path planning (increased)
        self.declare_parameter('collision_warning_distance', 6.0)  # Early warning distance

        self.num_robots = self.get_parameter('num_robots').value
        self.safe_distance = self.get_parameter('safe_distance').value
        self.path_buffer = self.get_parameter('path_buffer').value
        self.warning_distance = self.get_parameter('collision_warning_distance').value

        # Robot state tracking
        self.robot_positions = {}  # {robot_id: (x, y, timestamp)}
        self.robot_velocities = {}  # {robot_id: (vx, vy)} for predictive collision
        self.robot_goals = {}      # {robot_id: (goal_x, goal_y)}
        self.collision_states = {}  # {robot_id: bool}
        self.robot_priorities = {}  # {robot_id: priority_score}
        self.previous_positions = {}  # For velocity calculation

        # Deadlock detection
        self.robot_stopped_times = {}  # Track how long each robot has been stopped
        self.deadlock_timeout = 8.0    # Seconds before declaring deadlock (synced with Unity)

        # === TOPICS: Real-time position sharing ===
        self.pose_subs = []
        for i in range(self.num_robots):
            robot_name = f'tb3_{i}'
            pose_sub = self.create_subscription(
                PoseStamped,
                f'/{robot_name}/pose',
                lambda msg, idx=i: self.pose_callback(msg, idx),
                10
            )
            self.pose_subs.append(pose_sub)
            self.collision_states[i] = False
            # Note: Priorities now calculated dynamically based on distance-to-goal
            # (see update_robot_priorities method)
            self.get_logger().info(f'Subscribed to /{robot_name}/pose')

        # Publishers for emergency collision alerts
        self.collision_pubs = []
        for i in range(self.num_robots):
            robot_name = f'tb3_{i}'
            collision_pub = self.create_publisher(
                Bool,
                f'/{robot_name}/collision_detected',
                10
            )
            self.collision_pubs.append(collision_pub)

        # === SERVICE: Path approval coordination ===
        self.path_approval_service = self.create_service(
            PathApproval,
            '/request_path_approval',
            self.path_approval_callback
        )
        self.get_logger().info('Service /request_path_approval ready')

        # Periodic collision checking (10 Hz)
        self.create_timer(0.1, self.check_real_time_collisions)

        self.get_logger().info(
            f'Collision Coordinator started: {self.num_robots} robots, '
            f'safe_distance={self.safe_distance}m'
        )

    def pose_callback(self, msg, robot_id):
        """Store robot position from continuous updates"""
        x = msg.pose.position.x
        y = msg.pose.position.y
        timestamp = self.get_clock().now()
        
        # Calculate velocity from position change
        if robot_id in self.robot_positions:
            prev_pos = self.robot_positions[robot_id]
            prev_time = prev_pos[2]
            dt = (timestamp - prev_time).nanoseconds / 1e9
            if dt > 0.01:  # Avoid division by near-zero
                vx = (x - prev_pos[0]) / dt
                vy = (y - prev_pos[1]) / dt
                self.robot_velocities[robot_id] = (vx, vy)

        self.robot_positions[robot_id] = (x, y, timestamp)

        # Update priorities every position update
        self.update_robot_priorities()

    def check_real_time_collisions(self):
        """REACTIVE: Check for imminent collisions between moving robots with predictive detection"""
        robot_ids = list(self.robot_positions.keys())
        new_collision_states = {idx: False for idx in robot_ids}

        # Prediction time horizon (seconds)
        prediction_time = 1.0

        # Check each robot pair
        for i in range(len(robot_ids)):
            for j in range(i + 1, len(robot_ids)):
                id1, id2 = robot_ids[i], robot_ids[j]

                if id1 not in self.robot_positions or id2 not in self.robot_positions:
                    continue

                pos1 = self.robot_positions[id1]
                pos2 = self.robot_positions[id2]

                # Current 2D distance
                current_distance = math.sqrt((pos1[0] - pos2[0])**2 + (pos1[1] - pos2[1])**2)

                # Predictive collision check using velocities
                predicted_distance = current_distance
                if id1 in self.robot_velocities and id2 in self.robot_velocities:
                    v1 = self.robot_velocities[id1]
                    v2 = self.robot_velocities[id2]
                    
                    # Predict positions
                    pred_x1 = pos1[0] + v1[0] * prediction_time
                    pred_y1 = pos1[1] + v1[1] * prediction_time
                    pred_x2 = pos2[0] + v2[0] * prediction_time
                    pred_y2 = pos2[1] + v2[1] * prediction_time
                    
                    predicted_distance = math.sqrt((pred_x1 - pred_x2)**2 + (pred_y1 - pred_y2)**2)

                # Check both current and predicted distance
                min_distance = min(current_distance, predicted_distance)
                
                if min_distance < self.safe_distance:
                    # Collision imminent! Determine who should stop
                    # Strict priority: ONLY lower priority stops
                    # Lower value = higher priority (closer to goal)
                    if id1 in self.robot_priorities and id2 in self.robot_priorities:
                        if self.robot_priorities[id1] > self.robot_priorities[id2]:
                            # id1 has lower priority (farther from goal) - it stops
                            new_collision_states[id1] = True
                        elif self.robot_priorities[id1] < self.robot_priorities[id2]:
                            # id2 has lower priority - it stops
                            new_collision_states[id2] = True
                        else:
                            # Exactly equal priorities (rare with distance-based + ID tiebreaker)
                            if id1 < id2:
                                new_collision_states[id2] = True
                            else:
                                new_collision_states[id1] = True
                    else:
                        # Fallback if priorities not yet assigned: use ID-based tiebreaker
                        if id1 < id2:
                            new_collision_states[id2] = True
                        else:
                            new_collision_states[id1] = True

                    if not (self.collision_states.get(id1) and self.collision_states.get(id2)):
                        self.get_logger().warning(
                            f'COLLISION ALERT: tb3_{id1} and tb3_{id2} '
                            f'current: {current_distance:.2f}m, predicted: {predicted_distance:.2f}m '
                            f'(min safe: {self.safe_distance}m)'
                        )
                elif min_distance < self.warning_distance:
                    # Early warning - robots getting close
                    self.get_logger().debug(
                        f'WARNING: tb3_{id1} and tb3_{id2} approaching: {min_distance:.2f}m'
                    )

        # Check for deadlock: robots stopped for too long
        for robot_id in robot_ids:
            if new_collision_states.get(robot_id, False):
                # Robot is being stopped - track time
                if robot_id not in self.robot_stopped_times:
                    self.robot_stopped_times[robot_id] = self.get_clock().now()
                else:
                    # Check how long this robot has been stopped
                    stopped_duration = (self.get_clock().now() - self.robot_stopped_times[robot_id]).nanoseconds / 1e9

                    if stopped_duration > self.deadlock_timeout:
                        # DEADLOCK DETECTED - clear the stop flag to allow movement
                        self.get_logger().warning(
                            f'⚠️ DEADLOCK: tb3_{robot_id} stopped for {stopped_duration:.1f}s - clearing collision flag'
                        )
                        new_collision_states[robot_id] = False
                        del self.robot_stopped_times[robot_id]
            else:
                # Robot is moving - clear stopped timer
                if robot_id in self.robot_stopped_times:
                    del self.robot_stopped_times[robot_id]

        # Publish collision states
        for robot_id in robot_ids:
            msg = Bool()
            msg.data = new_collision_states[robot_id]

            # Publish if state changed
            if msg.data != self.collision_states.get(robot_id, False):
                self.collision_pubs[robot_id].publish(msg)
                status = "STOP" if msg.data else "RESUME"
                self.get_logger().info(f'tb3_{robot_id}: {status}')

            self.collision_states[robot_id] = msg.data

    def update_robot_priorities(self):
        """Update priorities based on distance to goal - closer robots have higher priority"""
        for robot_id, goal in self.robot_goals.items():
            if robot_id in self.robot_positions:
                pos = self.robot_positions[robot_id]
                distance = math.sqrt((goal[0] - pos[0])**2 + (goal[1] - pos[1])**2)
                # Lower distance = higher priority (lower value)
                # Add small robot_id tiebreaker to ensure uniqueness
                self.robot_priorities[robot_id] = distance + (robot_id * 0.01)

    def path_approval_callback(self, request, response):
        """PREDICTIVE: Approve/reject path before robot starts moving"""
        robot_id = request.robot_id
        start_x, start_y = request.start_x, request.start_y
        goal_x, goal_y = request.goal_x, request.goal_y

        self.get_logger().info(
            f'Path approval request from tb3_{robot_id}: '
            f'({start_x:.1f}, {start_y:.1f}) → ({goal_x:.1f}, {goal_y:.1f})'
        )

        # Store goal for tracking
        self.robot_goals[robot_id] = (goal_x, goal_y)

        # Check if path conflicts with other robots' positions or goals
        conflict_robot = None
        min_distance = float('inf')

        for other_id, other_pos in self.robot_positions.items():
            if other_id == robot_id:
                continue

            # Check distance from goal to other robot's current position
            distance = math.sqrt(
                (goal_x - other_pos[0])**2 + (goal_y - other_pos[1])**2
            )

            if distance < self.path_buffer:
                if distance < min_distance:
                    min_distance = distance
                    conflict_robot = other_id

        # Check if goal conflicts with other robots' goals
        for other_id, other_goal in self.robot_goals.items():
            if other_id == robot_id:
                continue

            distance = math.sqrt(
                (goal_x - other_goal[0])**2 + (goal_y - other_goal[1])**2
            )

            if distance < self.path_buffer:
                if distance < min_distance:
                    min_distance = distance
                    conflict_robot = other_id

        # Approve or reject based on priority
        if conflict_robot is not None:
            # Use priority to decide
            if self.robot_priorities[robot_id] > self.robot_priorities[conflict_robot]:
                # Higher priority - approved
                response.approved = True
                response.wait_time = 0.0
                response.reason = f'Approved (higher priority than tb3_{conflict_robot})'
                self.get_logger().info(
                    f'✓ tb3_{robot_id} APPROVED (priority over tb3_{conflict_robot})'
                )
            else:
                # Lower priority - wait
                response.approved = False
                response.wait_time = 3.0  # Suggest 3 second wait
                response.reason = f'Wait for tb3_{conflict_robot} ({min_distance:.1f}m away)'
                self.get_logger().warning(
                    f'✗ tb3_{robot_id} REJECTED - conflict with tb3_{conflict_robot}'
                )
        else:
            # No conflict - approved immediately
            response.approved = True
            response.wait_time = 0.0
            response.reason = 'Path clear'
            self.get_logger().info(f'✓ tb3_{robot_id} APPROVED - path clear')

        return response

def main(args=None):
    rclpy.init(args=args)
    node = CollisionCoordinatorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
