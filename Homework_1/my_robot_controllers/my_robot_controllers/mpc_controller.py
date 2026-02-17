

import rclpy
from rclpy.node import Node
import math
import numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class MPCController(Node):
    def __init__(self):
        super().__init__('mpc_controller')

        
        self.declare_parameter('horizon', 12)               
        self.declare_parameter('dt', 0.08)                  
        self.declare_parameter('v_ref', 0.32)               
        self.declare_parameter('waypoint_threshold', 0.22)  
        self.declare_parameter('max_omega', 1.6)            

        self.N = self.get_parameter('horizon').value
        self.dt = self.get_parameter('dt').value
        self.v_ref = self.get_parameter('v_ref').value
        self.threshold = self.get_parameter('waypoint_threshold').value
        self.max_omega = self.get_parameter('max_omega').value

        self.waypoints = np.array([
            [3.0, 0.0],
            [6.0, 4.0],
            [3.0, 4.0],
            [3.0, 1.0],
            [0.0, 3.0]
        ])
        self.current_idx = 0

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.08, self.control_loop)  

        self.get_logger().info("=== MPC Controller (стабилизированная версия) запущен ===")

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
        dist = math.hypot(dx, dy)

        if dist < self.threshold:
            self.current_idx += 1
            self.get_logger().info(f"Waypoint {self.current_idx} достигнут (dist = {dist:.3f})")
            return

        
        goal_yaw = math.atan2(dy, dx)
        yaw_error = goal_yaw - self.yaw
        yaw_error = math.atan2(math.sin(yaw_error), math.cos(yaw_error))

        
        v_max_now = self.v_ref * (1.0 - min(abs(yaw_error) / (math.pi * 0.7), 1.0))

        
        best_v = 0.0
        best_w = 0.0
        best_cost = float('inf')

        
        v_candidates = np.linspace(0.05, v_max_now, 6)
        w_candidates = np.linspace(-self.max_omega, self.max_omega, 11)

        for v in v_candidates:
            for w in w_candidates:
                cost = 0.0
                px, py, pth = self.x, self.y, self.yaw

                for _ in range(self.N):
                    px += v * math.cos(pth) * self.dt
                    py += v * math.sin(pth) * self.dt
                    pth += w * self.dt
                    pth = math.atan2(math.sin(pth), math.cos(pth))

                    
                    dist_err = math.hypot(px - tx, py - ty)
                    heading_err = math.atan2(ty - py, tx - px) - pth
                    heading_err = math.atan2(math.sin(heading_err), math.cos(heading_err))

                    cost += (
                        4.0 * dist_err**2 +           
                        1.5 * heading_err**2 +         
                        0.4 * (v - v_max_now)**2 +     
                        0.8 * w**2                     
                    )

                if cost < best_cost:
                    best_cost = cost
                    best_v = v
                    best_w = w

        
        cmd = Twist()
        cmd.linear.x = best_v
        cmd.angular.z = best_w
        self.cmd_pub.publish(cmd)

        
        if self.get_clock().now().nanoseconds % 1_000_000_000 < 100_000_000:
            self.get_logger().info(
                f"dist={dist:.2f}  yaw_err={math.degrees(yaw_error):.1f}°  "
                f"best_v={best_v:.2f}  best_w={best_w:.2f}  cost={best_cost:.1f}"
            )

    def stop(self):
        cmd = Twist()
        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = MPCController()
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