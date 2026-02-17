import rclpy
from rclpy.node import Node

import math
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry

from time import sleep

def quaternion_to_yaw(qx, qy, qz, qw):
    """
    Вычисляет только yaw из quaternion (xyzw порядок, как в ROS)
    Возвращает угол в радианах, в диапазоне [-pi, pi]
    """
    t3 = +2.0 * (qw * qz + qx * qy)
    t4 = +1.0 - 2.0 * (qy * qy + qz * qz)
    
    t4 = max(min(t4, 1.0), -1.0)
    
    yaw = math.atan2(t3, t4)
    return yaw



class PurePursuitController(Node):
    def __init__(self):
        super().__init__('pure_pursuit_controller')

        self.declare_parameter('look_ahead_distance', 0.5)
        self.declare_parameter('linear_speed', 0.5)
        self.declare_parameter('waypoint_threshold', 0.1)     
        self.declare_parameter('wheel_base', 0.4)

        self.L = self.get_parameter('look_ahead_distance').value
        self.v = self.get_parameter('linear_speed').value
        self.threshold = self.get_parameter('waypoint_threshold').value
        self.L_wheel = self.get_parameter('wheel_base').value

        self.waypoints = [
            (3.0, 0.0),
            (6.0, 4.0),
            (3.0, 4.0),
            (3.0, 1.0),
            (0.0, 3.0)
        ]

        self.current_waypoint_idx = 0
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info("Pure Pursuit controller started")
        self.get_logger().info(f"Waypoints: {self.waypoints}")

    def odom_callback(self, msg: Odometry):
        pose = msg.pose.pose
        self.current_x = pose.position.x
        self.current_y = pose.position.y

        orientation_q = pose.orientation
        self.current_yaw = quaternion_to_yaw(orientation_q.x, 
                                             orientation_q.y, 
                                             orientation_q.z, 
                                             orientation_q.w)

    def get_lookahead_point(self):
        """Находим точку на траектории на расстоянии L впереди"""
        target = None
        min_dist = float('inf')

        for i in range(self.current_waypoint_idx, len(self.waypoints)):
            wx, wy = self.waypoints[i]
            dx = wx - self.current_x
            dy = wy - self.current_y
            dist = math.hypot(dx, dy)

            if dist < min_dist and dist >= self.L * 0.5:
                min_dist = dist
                target = (wx, wy)

            if dist > self.L:
                break

        if target is None and self.current_waypoint_idx < len(self.waypoints):
            target = self.waypoints[-1]

        return target

    def pure_pursuit_step(self):
        if self.current_waypoint_idx >= len(self.waypoints):
            cmd = Twist()
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.cmd_pub.publish(cmd)
            self.get_logger().info("Goal reached!")
            return

        target_x, target_y = self.waypoints[self.current_waypoint_idx]

        dist_to_target = math.hypot(target_x - self.current_x, target_y - self.current_y)
        if dist_to_target < self.threshold:
            self.current_waypoint_idx += 1
            self.get_logger().info(f"Waypoint {self.current_waypoint_idx} reached")
            if self.current_waypoint_idx >= len(self.waypoints):
                self.get_logger().info("All waypoints completed!")
            return

        lookahead = self.get_lookahead_point()
        if lookahead is None:
            self.get_logger().warn("No valid lookahead point found")
            return

        lx, ly = lookahead

        dx = lx - self.current_x
        dy = ly - self.current_y
        angle_to_lookahead = math.atan2(dy, dx)

        alpha = angle_to_lookahead - self.current_yaw
        alpha = math.atan2(math.sin(alpha), math.cos(alpha))

        curvature = 2.0 * math.sin(alpha) / self.L
        omega = self.v * curvature

        omega = max(min(omega, 1.5), -1.5)

        cmd = Twist()
        cmd.linear.x = self.v
        cmd.angular.z = omega

        self.cmd_pub.publish(cmd)

    def control_loop(self):
        if abs(self.current_x) < 1e-6 and abs(self.current_y) < 1e-6 and abs(self.current_yaw) < 1e-6:
            self.get_logger().warn("cant start, no odom info")
            sleep(1)
            return

        self.pure_pursuit_step()


def main(args=None):
    rclpy.init(args=args)
    node = PurePursuitController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cmd = Twist()
        node.cmd_pub.publish(cmd)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()