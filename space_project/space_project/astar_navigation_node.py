import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
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

        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.path_pub = self.create_publisher(Path, 'astar_path', 10)

        self.map_data = None
        self.start = None
        #self.src = (49, 49)  # Unity coordinates in meters
        #self.dest = (40, 40)   # Unity coordinates in meters

        self.declare_parameter('src_x', 49.0)
        self.declare_parameter('src_y', 49.0)
        self.declare_parameter('dest_x', 40.0)
        self.declare_parameter('dest_y', 40.0)

        self.src = (
            float(self.get_parameter('src_x').value),
            float(self.get_parameter('src_y').value)
        )
        self.dest = (
            float(self.get_parameter('dest_x').value),
            float(self.get_parameter('dest_y').value)
        )

        self.get_logger().info(f"Robot starting at src={self.src} going to dest={self.dest}")

        self.path = []
        self.current_index = 0
        self.resolution = None
        self.map_row = None
        self.map_col = None

    # Conversione Unity -> Grid
    def unity_to_grid(self, x, y):
        """Converte coordinate Unity (-50:+50) in indici griglia"""
        grid_x = int((x + 50) * self.map_row / 100)
        grid_y = int((y + 50) * self.map_col / 100)
        # Clamp per sicurezza
        grid_x = max(0, min(self.map_row - 1, grid_x))
        grid_y = max(0, min(self.map_col - 1, grid_y))
        return grid_x, grid_y

    def grid_to_unity(self, i, j):
        """
        Converte coordinate della griglia (i, j) in coordinate Unity (-50 : 50).
        """
        x_unity = (i / self.map_row) * 100 - 50
        y_unity = (j / self.map_col) * 100 - 50
        return x_unity, y_unity


    def is_valid(self, row, col):
        return 0 <= row < self.map_row and 0 <= col < self.map_col
    
    def is_unblocked(self, grid, row, col):
        # 0 = ostacolo, 1 = libero
        return grid[row][col] == 0

    def is_destination(self, row, col, dest):
        return row == dest[0] and col == dest[1]

    def calculate_h_value(self, row, col, dest):
        return ((row - dest[0]) ** 2 + (col - dest[1]) ** 2) ** 0.5

    def trace_path(self, cell_details, dest):
        print("The Path is:")
        path = []
        row, col = dest
        while not (cell_details[row][col].parent_i == row and cell_details[row][col].parent_j == col):
            path.append((row, col))
            row, col = cell_details[row][col].parent_i, cell_details[row][col].parent_j
        path.append((row, col))
        path.reverse()
        self.path = path
        for p in path:
            print("->", p, end=" ")
        print()

    def a_star_search(self):
        # Convertiamo src e dest in indici di griglia
        self.src = self.unity_to_grid(self.src[0], self.src[1])
        self.dest = self.unity_to_grid(self.dest[0], self.dest[1])
        print("NORMALIZED SRC: ", self.src)
        print("NORMALIZED dest: ", self.dest)

        if not self.is_valid(self.src[0], self.src[1]) or not self.is_valid(self.dest[0], self.dest[1]):
            print("Source or destination is invalid")
            return

        if self.is_destination(self.src[0], self.src[1], self.dest):
            print("We are already at the destination")
            return

        closed_list = [[False for _ in range(self.map_col)] for _ in range(self.map_row)]
        cell_details = [[Cell() for _ in range(self.map_col)] for _ in range(self.map_row)]

        i, j = self.src
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
                    if self.is_destination(new_i, new_j, self.dest):
                        cell_details[new_i][new_j].parent_i = i
                        cell_details[new_i][new_j].parent_j = j
                        print("The destination cell is found")
                        self.trace_path(cell_details, self.dest)
                        found_dest = True
                        return
                    g_new = cell_details[i][j].g + 1.0
                    h_new = self.calculate_h_value(new_i, new_j, self.dest)
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

    def publish_path(self):
        if not self.path:
            print("Fallita pubblicazione path")
            return
        path_msg = Path()
        path_msg.header.frame_id = "map"

        for cell in self.path:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            app_coord = self.grid_to_unity(cell[0],cell[1])
            pose.pose.position.x = app_coord[0] 
            pose.pose.position.y = app_coord[1] 
            pose.pose.position.z = 0.0 
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)
        print("Pubblico il path")
        print("\nPATH PUBBLICATO (Unity coordinates):")
        for i, cell in enumerate(self.path):
            x_u, y_u = self.grid_to_unity(cell[0], cell[1])
            print(f"Step {i}: GRID=({cell[0]}, {cell[1]}) -> UNITY=({x_u:.2f}, {y_u:.2f})")
        self.path_pub.publish(path_msg)

    def map_callback(self, msg):
        self.map_row = msg.info.width
        self.map_col = msg.info.height
        self.resolution = msg.info.resolution

        # Convertiamo la mappa in lista di liste
        grid = []
        for row in range(self.map_col):
            row_data = []
            for col in range(self.map_row):
                index = row * self.map_row + col
                if msg.data[index] == -1 or msg.data[index] > 0:
                    row_data.append(1)  # ostacolo
                else:
                    row_data.append(0)  # libero
            grid.append(row_data)

        self.map_data = grid

        # Stampa la griglia
        print("Occupancy Grid:")
        for r in grid:
            print(''.join(str(c) for c in r))

        # Avvia A*
        self.a_star_search()

        self.publish_path()

def main(args=None):
    rclpy.init(args=args)
    node = UnityAStarController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
