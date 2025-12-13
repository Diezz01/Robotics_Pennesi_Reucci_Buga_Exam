import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, PoseArray
import heapq

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

        self.robots_sub = self.create_subscription(PoseArray, '/robots', self.robots_callback,10)
        #self.targets_sub = self.create_subscription(PoseArray, '/targets', self.targets_callback,10)

        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.target_explorer_sub = self.create_subscription(PoseArray, 'target', self.target_explorer_callback,10)
        self.path_pub = self.create_publisher(Path, 'astar_path', 10)

        self.target_subs = {}
        self.path_pubs = {}

        self.map_data = None
        self.start = None
        self.robots_list = None
        self.targets_list = None

        self.path = []
        self.current_index = 0
        self.resolution = None
        self.map_row = None
        self.map_col = None

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
        return grid[row][col] == 0  # 0 = obstacle, 1 = free

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

    def a_star_search(self, src, dest, robot_id):
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

        self.publish_path(path, robot_id)

    def publish_path(self, path, robot_id):
        if not path:
            print("Failed to publish path")
            return
        path_msg = Path()
        path_msg.header.frame_id = "map"
        for cell in path:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            app_coord = self.grid_to_unity(cell[0],cell[1])
            pose.pose.position.x = app_coord[0] 
            pose.pose.position.y = app_coord[1] 
            pose.pose.position.z = 0.0 
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)
        print("Publishing path")
        print("\nPATH PUBLISHED (Unity coordinates):")
        for i, cell in enumerate(path):
            x_u, y_u = self.grid_to_unity(cell[0], cell[1])
            print(f"Step {i}: GRID=({cell[0]}, {cell[1]}) -> UNITY=({x_u:.2f}, {y_u:.2f})")
        self.path_pubs[robot_id].publish(path_msg)
        #self.path_pub.publish(path_msg)
        
    def map_callback(self, msg):
        self.map_row = msg.info.width
        self.map_col = msg.info.height
        self.resolution = msg.info.resolution

        # Convert map to list of lists
        grid = []
        for row in range(self.map_col):
            row_data = []
            for col in range(self.map_row):
                index = row * self.map_row + col
                if msg.data[index] == -1 or msg.data[index] > 0:
                    row_data.append(1)  # obstacle
                else:
                    row_data.append(0)  # free
            grid.append(row_data)

        self.map_data = grid

        # Print the grid
        print("Occupancy Grid:")
        for r in grid:
            print(''.join(str(c) for c in r))

        # Start A*
        #self.a_star_search()

        #self.publish_path()

    def robots_callback(self, msg):
        self.robots_list = [(p.position.x, p.position.z, p.position.y) for p in msg.poses]
        self.get_logger().info(f"Received {len(self.robots_list)} robots at points: {self.robots_list}")
        num_robot = len(self.robots_list)
        for n in range(0, num_robot):
            robot_id = f"/tb3_{n}"
            astar_topic = f"{robot_id}/astar_path"
            target_topic = f"{robot_id}/target"
            target_sub = self.create_subscription(PoseArray, target_topic, self.target_explorer_callback,10)
            path_pub = self.create_publisher(Path, astar_topic, 10)
            print("creating topic",astar_topic)
            print("creating topic",target_topic)
            self.target_subs[robot_id] = target_sub
            self.path_pubs[robot_id] = path_pub

    def targets_callback(self, msg):
        self.targets_list = [(p.position.x, p.position.z, p.position.y) for p in msg.poses]
        self.get_logger().info(f"Received {len(self.targets_list)} targets at points: {self.targets_list}")
    
    def target_explorer_callback(self, msg):
        robot_id = f"/{msg.header.frame_id}" # Obtaining robot id from the topic
        self.target_explorer_sub.topic_name
        explorer_src_dest = [(p.position.x, p.position.z, p.position.y) for p in msg.poses]
        self.get_logger().info(f"Received from explorer: src {explorer_src_dest[0]} dest {explorer_src_dest[1]}")
        self.a_star_search(explorer_src_dest[0],explorer_src_dest[1], robot_id) # Specifing also the robot id

def main(args=None):
    rclpy.init(args=args)
    node = UnityAStarController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
