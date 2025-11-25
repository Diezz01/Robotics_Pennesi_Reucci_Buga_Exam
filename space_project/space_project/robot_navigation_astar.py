import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
import heapq
import math

class AStarPlanner(Node):
    def __init__(self, robot_name="robot1"):
        super().__init__(f'{robot_name}_astar')
        self.robot = robot_name
        self.map_data = None

        self.create_subscription(OccupancyGrid, "/map", self.map_callback, 10)
        self.create_subscription(PoseStamped, f"/{self.robot}/replan", self.replan_callback, 10)

        self.path_pub = self.create_publisher(Path, f"/{self.robot}/astar_path", 10)

        self.start = (2, 2)
        self.goal = (15, 15)

    def map_callback(self, msg):
        self.map_data = msg

    def replan_callback(self, msg):
        self.get_logger().info("Replanning requested...")
        self.plan_and_publish()

    def heuristic(self, a, b):
        return math.hypot(b[0]-a[0], b[1]-a[1])

    def neighbors(self, x, y):
        dirs = [(1,0),(-1,0),(0,1),(0,-1)]
        for dx, dy in dirs:
            nx, ny = x+dx, y+dy
            if 0 <= nx < self.map_data.info.width and 0 <= ny < self.map_data.info.height:
                idx = ny*self.map_data.info.width + nx
                if self.map_data.data[idx] == 0:  # free cell
                    yield nx, ny

    def a_star(self, start, goal):
        open_set = [(0, start)]
        came = {}
        g = {start: 0}

        while open_set:
            _, curr = heapq.heappop(open_set)
            if curr == goal:
                path = []
                while curr in came:
                    path.append(curr)
                    curr = came[curr]
                path.append(start)
                return path[::-1]

            for n in self.neighbors(*curr):
                tentative = g[curr] + 1
                if n not in g or tentative < g[n]:
                    g[n] = tentative
                    came[n] = curr
                    f = tentative + self.heuristic(n, goal)
                    heapq.heappush(open_set, (f, n))
        return []

    def publish_path(self, pts):
        path = Path()
        path.header.frame_id = "map"
        res = self.map_data.info.resolution
        ox = self.map_data.info.origin.position.x
        oy = self.map_data.info.origin.position.y

        for x, y in pts:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.pose.position.x = x * res + ox
            pose.pose.position.y = y * res + oy
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        self.path_pub.publish(path)

    def plan_and_publish(self):
        if self.map_data:
            pts = self.a_star(self.start, self.goal)
            if pts:
                self.publish_path(pts)

def main():
    rclpy.init()
    node = AStarPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
