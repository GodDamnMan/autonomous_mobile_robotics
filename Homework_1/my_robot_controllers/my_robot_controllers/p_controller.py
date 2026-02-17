#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class PController(Node):
    def __init__(self):
        super().__init__('p_controller')

        self.declare_parameter('kp_linear', 1.2)
        self.declare_parameter('kp_angular', 2.5)
        self.declare_parameter('waypoint_threshold', 0.1)
        self.declare_parameter('max_linear_speed', 0.5)
        self.declare_parameter('max_angular_speed', 1.8)

        self.kp_lin = self.get_parameter('kp_linear').value
        self.kp_ang = self.get_parameter('kp_angular').value
        self.threshold = self.get_parameter('waypoint_threshold').value
        self.max_v = self.get_parameter('max_linear_speed').value
        self.max_w = self.get_parameter('max_angular_speed').value

        self.waypoints = [
            (3.0, 0.0), (6.0, 4.0), (3.0, 4.0), (3.0, 1.0), (0.0, 3.0)
        ]
        self.current_idx = 0

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0

        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info("P-Controller started")

    def odom_callback(self, msg):
        pose = msg.pose.pose
        self.current_x = pose.position.x
        self.current_y = pose.position.y
        q = pose.orientation
        t3 = 2.0 * (q.w * q.z + q.x * q.y)
        t4 = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        t4 = max(min(t4, 1.0), -1.0)
        self.current_yaw = math.atan2(t3, t4)

    def control_loop(self):
        if self.current_idx >= len(self.waypoints):
            self.stop()
            return

        tx, ty = self.waypoints[self.current_idx]
        dx = tx - self.current_x
        dy = ty - self.current_y
        dist = math.hypot(dx, dy)

        if dist < self.threshold:
            self.current_idx += 1
            self.get_logger().info(f"Waypoint {self.current_idx} reached")
            if self.current_idx >= len(self.waypoints):
                self.get_logger().info("All waypoints done!")
                self.stop()
            return

        desired_yaw = math.atan2(dy, dx)
        yaw_error = desired_yaw - self.current_yaw
        yaw_error = math.atan2(math.sin(yaw_error), math.cos(yaw_error))

        angular = self.kp_ang * yaw_error
        angular = max(min(angular, self.max_w), -self.max_w)

        linear = self.kp_lin * dist
        linear = min(linear, self.max_v)

        cmd = Twist()
        cmd.linear.x = linear
        cmd.angular.z = angular
        self.cmd_pub.publish(cmd)

    def stop(self):
        cmd = Twist()
        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = PController()
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