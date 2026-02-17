import rclpy
from rclpy.node import Node
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class StanleyController(Node):
    def __init__(self):
        super().__init__('stanley_controller')

        
        self.declare_parameter('k', 0.8)                    
        self.declare_parameter('softening', 0.2)
        self.declare_parameter('linear_speed_max', 0.35)
        self.declare_parameter('waypoint_threshold', 0.25)   
        self.declare_parameter('wheel_base', 0.3)           

        self.k = self.get_parameter('k').value
        self.soft = self.get_parameter('softening').value
        self.v_max = self.get_parameter('linear_speed_max').value
        self.threshold = self.get_parameter('waypoint_threshold').value
        self.L = self.get_parameter('wheel_base').value

        
        self.waypoints = [
            (3.0, 0.0), (6.0, 4.0), (3.0, 4.0), (3.0, 1.0), (0.0, 3.0)
        ]
        self.current_idx = 0

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.05, self.control_loop)   

        self.get_logger().info("=== Stanley Controller (улучшенная версия) запущен ===")
        self.get_logger().info(f"Waypoints: {self.waypoints}")

    def odom_callback(self, msg: Odometry):
        pose = msg.pose.pose
        self.x = pose.position.x
        self.y = pose.position.y

        
        q = pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny, cosy)

    def control_loop(self):
        if self.current_idx >= len(self.waypoints):
            self.stop()
            self.get_logger().info("Все waypoints достигнуты!")
            return

        
        tx, ty = self.waypoints[self.current_idx]
        dx = tx - self.x
        dy = ty - self.y
        dist_to_goal = math.hypot(dx, dy)

        
        if dist_to_goal < self.threshold:
            self.current_idx += 1
            self.get_logger().info(f"Waypoint {self.current_idx} достигнут (dist = {dist_to_goal:.3f})")
            return

        
        
        path_yaw = math.atan2(dy, dx)

        
        heading_error = path_yaw - self.yaw
        heading_error = math.atan2(math.sin(heading_error), math.cos(heading_error))

        
        
        front_axle_x = self.x + self.L * math.cos(self.yaw)
        front_axle_y = self.y + self.L * math.sin(self.yaw)
        cte = (tx - front_axle_x) * math.sin(path_yaw) - (ty - front_axle_y) * math.cos(path_yaw)

        
        delta = heading_error + math.atan2(self.k * cte, self.v_max + self.soft)

        
        speed = self.v_max * (1.0 - min(abs(heading_error) / (math.pi * 0.6), 1.0))

        
        omega = delta / self.L
        omega = max(min(omega, 1.8), -1.8)

        
        cmd = Twist()
        cmd.linear.x = speed
        cmd.angular.z = omega
        self.cmd_pub.publish(cmd)

        
        if self.get_clock().now().nanoseconds % 500_000_000 == 0:  
            self.get_logger().info(
                f"dist={dist_to_goal:.2f}  cte={cte:.3f}  head_err={math.degrees(heading_error):.1f}°  "
                f"v={speed:.2f}  ω={omega:.2f}"
            )

    def stop(self):
        cmd = Twist()
        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = StanleyController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()