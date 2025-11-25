import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist,Point, Pose
from nav_msgs.msg import Odometry
import math

class MoveToTarget(Node):
    def __init__(self, target):
        super().__init__('move_to_target')
        
        # Target finale
        self.target = target  # (x, y, z)

        # Publisher per il comando di velocità
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Subscriber per la posizione corrente
        self.odom_sub = self.create_subscription(Pose, '/robot_pose', self.odom_callback, 10)


        # Timer per il loop di controllo
        self.timer = self.create_timer(0.1, self.control_loop)

        # Stato corrente del robot
        
        self.x = -5.0
        self.y = -7.0
        self.z = 0.0
        self.yaw = 0.0
        self.state = 'rotate'  # stati: 'rotate' → 'move'
        self.start_position = (self.x, self.y, self.z) 
        self.moving_to_target = True

        # --- Gestione batteria ---
        self.battery_level = 100.0        # percentuale iniziale
        self.consumption_rate = 0.5       # % di batteria per metro percorso
        self.prev_x = None
        self.prev_y = None
        self.prev_z = None
        self.low_battery_threshold = 10.0 

    def update_battery(self):
        """Aggiorna il livello della batteria in base alla distanza percorsa."""
        if self.prev_x is not None:
            dx = self.x - self.prev_x
            dy = self.y - self.prev_y
            dz = self.z - self.prev_z
            distance = math.sqrt(dx**2 + dy**2 + dz**2)
            self.battery_level -= distance * self.consumption_rate
            self.battery_level = max(self.battery_level, 0.0)
        self.prev_x, self.prev_y, self.prev_z = self.x, self.y, self.z

    def odom_callback(self, msg):
        # Aggiorna posizione
        self.x = msg.position.x
        self.y = msg.position.y
        self.z = msg.position.z

        # Estrae yaw dalla quaternion
        q = msg.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)
        # print("YAW: ",self.yaw)

    def control_loop(self):
        # Aggiorna batteria
        self.update_battery()

        # Se batteria scarica, ferma tutto
        if self.battery_level <= self.low_battery_threshold:
            msg = Twist()
            self.cmd_pub.publish(msg)
            self.get_logger().warn(f"Batteria bassa ({self.battery_level:.1f}%) - arresto robot!")
            return

        # Differenze verso il target
        dx = self.target[0] - self.x
        dy = self.target[1] - self.y
        dz = self.target[2] - self.z

        # Distanza euclidea
        dist = math.sqrt(dx**2 + dy**2 + dz**2)

        # Calcola l'angolo verso il target e l'errore angolare normalizzato
        theta_target = math.atan2(dy, dx)
        theta_error = theta_target - self.yaw
        theta_error = (theta_error + math.pi) % (2 * math.pi) - math.pi  # normalizzazione tra -pi e pi

        # Comando Twist
        msg = Twist()

        # Parametri di controllo proporzionale
        K_linear = 0.5       # velocità massima lineare
        K_angular = 1.0      # guadagno angolare

        # Riduce la velocità lineare se l'errore angolare è grande
        linear_speed = K_linear * min(dist, 1.0) * max(0.0, 1 - abs(theta_error)/math.pi)
        msg.linear.x = linear_speed
        msg.angular.z = K_angular * theta_error
        msg.linear.z = 0.5 * dz  # movimento verticale proporzionale

        # Se il robot è vicino al target, fermalo e inverti la direzione
        if dist < 0.1:
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            msg.linear.z = 0.0
            self.get_logger().info('Target raggiunto!')

            # Inverti direzione
            if getattr(self, 'moving_to_target', True):
                # Salva la posizione di partenza se non già fatto
                if not hasattr(self, 'start_position'):
                    self.start_position = (self.x, self.y, self.z)
                self.target = self.start_position
                self.moving_to_target = False
            else:
                self.target = (-5.0, -7.0, 0.0)  # target originale
                self.moving_to_target = True

        # Pubblica il comando
        self.cmd_pub.publish(msg)

        # Log utile per debug
        self.get_logger().info(f"Distanza: {dist:.2f}, Theta error: {theta_error:.2f}, Batteria: {self.battery_level:.2f}%")



def main(args=None):
    rclpy.init(args=args)
    target = (5.0, 5.0, 0.0)  # esempio: x, y, z
    node = MoveToTarget(target)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
