#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from geometry_msgs.msg import Twist, PoseStamped

class UnityAStarController(Node):
    def __init__(self):
        super().__init__('unity_astar_controller')


        # Subscriber all'odometria
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Subscriber alla mappa
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        
        # Publisher dei comandi veloci
        #self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.path_pub = self.create_publisher(Path, '/astar_path', 10)


        # Parametri
        self.map_data = None
        self.robot_pos = (49.5,49.5)  # Posizione reale in metri
        self.start = None
        self.goal = (40.0, 40.0)  # (x, y) in coordinate griglia
        self.path = []
        self.current_index = 0

        # Controllo velocità
        self.max_linear = 3.0   # m/s (Unity)
        self.max_angular = 1.0  # rad/s

        # Imposta obiettivo finale nella scena Unity (metri)
        self.goal_world = (40.0, 40.0)  # modifica secondo scena

    def publish_path(self):
        if not self.path:
            print("Fallita pubblicazione path")
            return
        path_msg = Path()
        path_msg.header.frame_id = "map"

        for cell in self.path:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.pose.position.x = cell[0] * 1.0  # moltiplica per resolution
            pose.pose.position.y = cell[1] * 1.0
            pose.pose.position.z = 0.0 
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)
        print("Pubblico il path")
        self.path_pub.publish(path_msg)


    # Riceve mappa da Unity
    def map_callback(self, msg):
        print("mappa ricevuta")
        width = msg.info.width
        height = msg.info.height
        resolution = msg.info.resolution

        # 1 = ostacolo, 0 = libero
        data = np.array(msg.data).reshape((height, width))
        self.map_data = (data == 1).astype(np.int8)
        print("Mappa ricevuta ({} x {}):".format(width, height))
        for row in data:
            line = ''.join(['#' if x==1 else '.' for x in row])
            print(line)
        print("\n" + "="*width + "\n") 

        # Converte goal mondo -> coordinate griglia
        gx = int(self.goal_world[0] / resolution)
        gy = int(self.goal_world[1] / resolution)
        self.goal = (gx, gy)

        # Se start non definito e robot_pos già letta
        print("start: ", self.start, "Robot pos: ", self.robot_pos)
        if self.start is None and self.robot_pos is not None:
            print("Devo calcoalre la posizione")
            self.start = self.get_robot_grid_pos(msg, self.robot_pos)
            self.path = self.a_star(self.map_data, self.start, self.goal)
            print("Percorso generato (celle):")
            for p in self.path:
                print(p)
            self.current_index = 0
            self.get_logger().info(f'Percorso generato: {len(self.path)} punti')
            self.publish_path()

    # Riceve odometria
    def odom_callback(self, msg):
        #print("Ricevo posizione del robot")
        self.robot_pos = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        #print(self.robot_pos)
        self.follow_path()

    # Converte posizione reale in coordinate griglia
    def get_robot_grid_pos(self, map_msg, robot_pos):
        rx, ry = robot_pos
        mx = int(rx / map_msg.info.resolution)
        my = int(ry / map_msg.info.resolution)
        return (mx, my)

    # A* semplice (grid 2D)
    def a_star(self, grid, start, goal):
        from queue import PriorityQueue
        width, height = grid.shape[1], grid.shape[0]
        open_set = PriorityQueue()
        open_set.put((0, start))
        came_from = {}
        g_score = {start:0}
        f_score = {start:self.heuristic(start, goal)}

        while not open_set.empty():
            _, current = open_set.get()
            if current == goal:
                return self.reconstruct_path(came_from, current)

            neighbors = self.get_neighbors(current, width, height)
            for n in neighbors:
                if grid[n[1], n[0]] != 0:
                    continue  # ostacolo
                tentative_g = g_score[current] + 1
                if n not in g_score or tentative_g < g_score[n]:
                    came_from[n] = current
                    g_score[n] = tentative_g
                    f_score[n] = tentative_g + self.heuristic(n, goal)
                    open_set.put((f_score[n], n))
        return []

    def heuristic(self, a, b):
        return abs(a[0]-b[0]) + abs(a[1]-b[1])

    def reconstruct_path(self, came_from, current):
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path

    def get_neighbors(self, node, width, height):
        x, y = node
        neighbors = []
        for dx, dy in [(-1,0),(1,0),(0,-1),(0,1)]:
            nx, ny = x+dx, y+dy
            if 0 <= nx < width and 0 <= ny < height:
                neighbors.append((nx, ny))
        return neighbors

    # Controllo per seguire il percorso
    def follow_path(self):
        if not self.path or self.current_index >= len(self.path) or self.robot_pos is None:
            # stop robot
            twist = Twist()
            #self.cmd_pub.publish(twist)
            return

        target = self.path[self.current_index]
        tx, ty = target
        resolution = 1  # modifica con la risoluzione reale della mappa in Unity
        tx_world = tx * resolution
        ty_world = ty * resolution

        rx, ry = self.robot_pos
        dx = tx_world - rx
        dy = ty_world - ry
        distance = np.hypot(dx, dy)

        if distance < 0.1:
            self.current_index += 1
            return

        angle_to_goal = np.arctan2(dy, dx)
        twist = Twist()
        twist.linear.x = min(self.max_linear, distance)
        twist.angular.z = max(-self.max_angular, min(self.max_angular, angle_to_goal))
       # self.cmd_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = UnityAStarController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
