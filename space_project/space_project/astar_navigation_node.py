#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, PoseArray
from space_project.srv import PathApproval
import heapq
import time

# Define the Cell class
class Cell:
    def __init__(self):
        self.parent_i = 0
        self.parent_j = 0
        self.f = float('inf')
        self.g = float('inf')
        self.h = 0

class UnityAStarController(Node):
    def __init__(self):
        super().__init__('unity_astar_controller')

        # Declare parameter for number of robots
        self.declare_parameter('num_robots', 4)

        num_robots = self.get_parameter('num_robots').value

        # Subscribe to shared map (all robots use same map)
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)

        # Create per-robot subscriptions and publishers
        self.robot_subs = []
        self.robot_pubs = []

        for i in range(num_robots):
            robot_name = f'tb3_{i}'

            # Subscribe to this robot's target requests
            target_sub = self.create_subscription(
                PoseArray,
                f'/{robot_name}/target',
                lambda msg, idx=i: self.target_callback(msg, idx),
                10
            )
            self.robot_subs.append(target_sub)

            # Publisher for this robot's path
            path_pub = self.create_publisher(
                Path,
                f'/{robot_name}/astar_path',
                10
            )
            self.robot_pubs.append(path_pub)

            self.get_logger().info(f'Subscribed to /{robot_name}/target')
            self.get_logger().info(f'Publishing to /{robot_name}/astar_path')

        # Initialize map data
        self.map_data = []
        self.map_row = 0
        self.map_col = 0
        self.resolution = 0.0

        # Service client for path approval (hybrid coordination)
        self.path_approval_client = self.create_client(PathApproval, '/request_path_approval')

        # Non-blocking check for service availability
        # Don't block initialization - will check when needed
        if not self.path_approval_client.wait_for_service(timeout_sec=0.1):
            self.get_logger().warning('Path approval service not immediately available - will plan without approval if needed')
        else:
            self.get_logger().info('Path approval service connected!')

    # Unity coordinates (-50:+50) to Grid indices
    def unity_to_grid(self, x, y):
        cell_size = 1
        local_x = x - (-50)
        local_y = y - (-50)

        # indici della cella
        grid_x = int(local_x // cell_size)
        grid_y = int(local_y // cell_size)

        # inverti l'asse Y per adattarsi alla griglia
        grid_y = self.map_col - 1 - grid_y

        # clamp per non uscire dai limiti
        grid_x = max(0, min(self.map_row - 1, grid_x))
        grid_y = max(0, min(self.map_col - 1, grid_y))

        return (grid_y, grid_x)

        # Converts grid coordinates (i, j) to Unity coordinates (-50 : 50).
    def grid_to_unity(self, grid_y, grid_x):
        # Inverti l'asse Y
        cell_size = 1
        local_y = (self.map_col - 1 - grid_y) * cell_size
        local_x = grid_x * cell_size

        # Centro della cella
        x_world = (-50) + local_x + cell_size / 2
        y_world = (-50) + local_y + cell_size / 2

        return (x_world, y_world)

    def is_valid(self, row, col):
        return 0 <= row < self.map_row and 0 <= col < self.map_col
    
    def is_unblocked(self, grid, row, col):
        return grid[row][col] == 0  # 0 = free, 1 = obstacle (CORRECTED)

    def is_destination(self, row, col, dest):
        return row == dest[0] and col == dest[1]

    def calculate_h_value(self, row, col, dest):
        return ((row - dest[0]) ** 2 + (col - dest[1]) ** 2) ** 0.5

    def trace_path(self, cell_details, dest):
        #print("The Path is:")
        path = []
        row, col = dest
        while not (cell_details[row][col].parent_i == row and cell_details[row][col].parent_j == col):
            path.append((row, col))
            row, col = cell_details[row][col].parent_i, cell_details[row][col].parent_j
        path.append((row, col))
        path.reverse()
        self.path = path
        '''for p in path:
            print("->", p, end=" ")
        print()'''

        return path

    def a_star_search(self, src, dest):
        # Convert src and dest to grid indices
        path = []
        src = self.unity_to_grid(src[0], src[1])
        dest = self.unity_to_grid(dest[0], dest[1])
        print("NORMALIZED SRC: ", src)
        print("NORMALIZED dest: ", dest)

        if not self.is_valid(src[0], src[1]) or not self.is_valid(dest[0], dest[1]):
            print("Source or destination is invalid")
            return

        if self.is_destination(src[0], src[1], dest):
            print("We are already at the destination")
            return

        closed_list = [[False for _ in range(self.map_col)] for _ in range(self.map_row)]
        cell_details = [[Cell() for _ in range(self.map_col)] for _ in range(self.map_row)]

        i, j = src
        cell_details[i][j].f = cell_details[i][j].g = cell_details[i][j].h = 0
        cell_details[i][j].parent_i = i
        cell_details[i][j].parent_j = j

        open_list = []
        heapq.heappush(open_list, (0.0, i, j))
        found_dest = False

        directions = [(0, 1), (0, -1), (1, 0), (-1, 0), (1, 1), (1, -1), (-1, 1), (-1, -1)]

        while open_list:
            _, i, j = heapq.heappop(open_list)
            closed_list[i][j] = True

            for d in directions:
                new_i, new_j = i + d[0], j + d[1]
                if self.is_valid(new_i, new_j) and self.is_unblocked(self.map_data, new_i, new_j) and not closed_list[new_i][new_j]:
                    if self.is_destination(new_i, new_j, dest):
                        cell_details[new_i][new_j].parent_i = i
                        cell_details[new_i][new_j].parent_j = j
                        print("The destination cell is found")
                        path = self.trace_path(cell_details, dest)
                        found_dest = True
                        break
                    g_new = cell_details[i][j].g + 1.0
                    h_new = self.calculate_h_value(new_i, new_j, dest)
                    f_new = g_new + h_new
                    if cell_details[new_i][new_j].f == float('inf') or cell_details[new_i][new_j].f > f_new:
                        heapq.heappush(open_list, (f_new, new_i, new_j))
                        cell_details[new_i][new_j].f = f_new
                        cell_details[new_i][new_j].g = g_new
                        cell_details[new_i][new_j].h = h_new
                        cell_details[new_i][new_j].parent_i = i
                        cell_details[new_i][new_j].parent_j = j

        if not found_dest:
            print("Failed to find the destination cell")

        return path

    def publish_path(self, path, robot_idx):
        """Publish path to specific robot's topic"""
        robot_name = f'tb3_{robot_idx}'

        if not path:
            self.get_logger().error(f'{robot_name}: Failed to publish path (path is empty)')
            return

        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = "map"

        for cell in path:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            app_coord = self.grid_to_unity(cell[0], cell[1])
            pose.pose.position.x = app_coord[0]
            pose.pose.position.y = app_coord[1]
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

        # Use robot-specific publisher
        self.robot_pubs[robot_idx].publish(path_msg)
        self.get_logger().info(f'{robot_name}: Published path with {len(path)} waypoints')

    def map_callback(self, msg):
        self.map_row = msg.info.width
        self.map_col = msg.info.height
        self.resolution = msg.info.resolution

        # Convert map to list of lists
        grid = []
        obstacle_count = 0
        for row in range(self.map_col):
            row_data = []
            for col in range(self.map_row):
                index = row * self.map_row + col
                if msg.data[index] == -1 or msg.data[index] > 0:
                    row_data.append(1)  # obstacle
                    obstacle_count += 1
                else:
                    row_data.append(0)  # free
            grid.append(row_data)

        # Store grid data
        self.map_data = grid

        # Enhanced logging
        total_cells = self.map_row * self.map_col
        obstacle_percentage = (obstacle_count / total_cells * 100) if total_cells > 0 else 0

        self.get_logger().info('='*60)
        self.get_logger().info(f'Map received: {self.map_row}x{self.map_col}')
        self.get_logger().info(f'Resolution: {self.resolution}m per cell')
        self.get_logger().info(f'Origin: ({msg.info.origin.position.x:.2f}, {msg.info.origin.position.y:.2f})')
        self.get_logger().info(f'Obstacles: {obstacle_count}/{total_cells} cells ({obstacle_percentage:.1f}%)')
        self.get_logger().info('='*60)

        # Print grid sample if obstacles detected
        if obstacle_count > 0:
            self.get_logger().info('Grid sample (first 100x100, X=obstacle, .=free):')
            for r in range(min(100, len(grid))):
                print(''.join('X' if c == 1 else '.' for c in grid[r][:100]))
        else:
            self.get_logger().warning('NO OBSTACLES DETECTED! Check Unity obstacle layer configuration.')

        # Start A*
        #self.a_star_search()

        #self.publish_path()

    def robots_callback(self, msg):
        self.robots_list = [(p.position.x, p.position.z, p.position.y) for p in msg.poses]
        self.get_logger().info(f"Received {len(self.robots_list)} robots at points: {self.robots_list}")

    def targets_callback(self, msg):
        self.targets_list = [(p.position.x, p.position.z, p.position.y) for p in msg.poses]
        self.get_logger().info(f"Received {len(self.targets_list)} targets at points: {self.targets_list}")
    
    def target_callback(self, msg, robot_idx):
        """Handle path request from specific robot"""
        robot_name = f'tb3_{robot_idx}'

        if len(msg.poses) < 2:
            self.get_logger().warning(f'{robot_name}: Invalid target message (need 2 poses)')
            return

        # Extract source and destination
        src = (msg.poses[0].position.x, msg.poses[0].position.z, msg.poses[0].position.y)
        dest = (msg.poses[1].position.x, msg.poses[1].position.z, msg.poses[1].position.y)

        self.get_logger().info(f'{robot_name}: Path request from {src} to {dest}')

        # === HYBRID APPROACH: Request path approval before planning (NON-BLOCKING) ===
        # Check if service is ready (non-blocking)
        if not self.path_approval_client.service_is_ready():
            self.get_logger().warning(f'{robot_name}: Path approval service not ready - planning without approval')
            # Service not available - plan path immediately as fallback
            path = self.a_star_search(src, dest)
            if path:
                self.publish_path(path, robot_idx)
            else:
                self.get_logger().error(f'{robot_name}: Failed to find path')
            return

        # Service is ready - call it asynchronously with callback
        request = PathApproval.Request()
        request.robot_id = robot_idx
        request.start_x = float(src[0])
        request.start_y = float(src[1])
        request.goal_x = float(dest[0])
        request.goal_y = float(dest[1])

        # Async call with callback - doesn't block
        future = self.path_approval_client.call_async(request)
        future.add_done_callback(lambda f: self.handle_path_approval_response(f, robot_idx, src, dest))

    def handle_path_approval_response(self, future, robot_idx, src, dest):
        """Handle async path approval response"""
        robot_name = f'tb3_{robot_idx}'

        try:
            response = future.result()
            if response.approved:
                self.get_logger().info(f'{robot_name}: Path APPROVED - {response.reason}')
                # Compute and publish path
                path = self.a_star_search(src, dest)
                if path:
                    self.publish_path(path, robot_idx)
                else:
                    self.get_logger().error(f'{robot_name}: Failed to find path')
            else:
                self.get_logger().warning(
                    f'{robot_name}: Path NOT approved - {response.reason}. '
                    f'Suggested wait: {response.wait_time}s'
                )
                # Path rejected - robot will retry later
        except Exception as e:
            # Service call failed - plan anyway as fallback
            self.get_logger().warning(f'{robot_name}: Service call failed ({e}) - planning without approval')
            path = self.a_star_search(src, dest)
            if path:
                self.publish_path(path, robot_idx)
            else:
                self.get_logger().error(f'{robot_name}: Failed to find path')

    # OLD SINGLE-ROBOT CALLBACK - Kept for reference but not used
    # def target_explorer_callback(self, msg):
    #     explorer_src_dest = [(p.position.x, p.position.z, p.position.y) for p in msg.poses]
    #     self.get_logger().info(f"Received from explorer: src {explorer_src_dest[0]} dest {explorer_src_dest[1]}")
    #     path = self.a_star_search(explorer_src_dest[0], explorer_src_dest[1])
    #     if path:
    #         self.publish_path(path, 0)  # Would need robot index


def main(args=None):
    rclpy.init(args=args)
    node = UnityAStarController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
