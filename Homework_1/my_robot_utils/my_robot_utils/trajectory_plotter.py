#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
from collections import deque
import math

class TrajectoryPlotter(Node):
    def __init__(self):
        super().__init__('trajectory_plotter')

        self.max_points = 2000  # лимит точек для скорости
        self.xs = deque(maxlen=self.max_points)
        self.ys = deque(maxlen=self.max_points)
        self.times = deque(maxlen=self.max_points)
        self.yaws = deque(maxlen=self.max_points)

        self.start_time = None

        self.sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)

        plt.ion()  # интерактивный режим
        self.fig, self.axs = plt.subplots(2, 2, figsize=(12, 10))
        self.fig.suptitle('Live Trajectory & States')

        self.timer = self.create_timer(0.5, self.update_plot)

    def quaternion_to_yaw(self, q):
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny, cosy)

    def odom_callback(self, msg):
        t = self.get_clock().now().nanoseconds / 1e9
        if self.start_time is None:
            self.start_time = t

        rel_t = t - self.start_time

        self.times.append(rel_t)
        self.xs.append(msg.pose.pose.position.x)
        self.ys.append(msg.pose.pose.position.y)
        self.yaws.append(self.quaternion_to_yaw(msg.pose.pose.orientation))

    def update_plot(self):
        if len(self.times) < 2:
            return

        ts = list(self.times)
        xs = list(self.xs)
        ys = list(self.ys)
        yaws_deg = [math.degrees(y) for y in self.yaws]

        # XY
        self.axs[0,0].cla()
        self.axs[0,0].plot(xs, ys, 'b-', lw=2)
        self.axs[0,0].grid(True)
        self.axs[0,0].set_aspect('equal')
        self.axs[0,0].set_title('XY Trajectory')
        self.axs[0,0].set_xlabel('X [m]')
        self.axs[0,0].set_ylabel('Y [m]')

        # X(t)
        self.axs[0,1].cla()
        self.axs[0,1].plot(ts, xs, 'g-')
        self.axs[0,1].grid(True)
        self.axs[0,1].set_title('X vs Time')
        self.axs[0,1].set_xlabel('Time [s]')
        self.axs[0,1].set_ylabel('X [m]')

        # Y(t)
        self.axs[1,0].cla()
        self.axs[1,0].plot(ts, ys, 'm-')
        self.axs[1,0].grid(True)
        self.axs[1,0].set_title('Y vs Time')
        self.axs[1,0].set_xlabel('Time [s]')
        self.axs[1,0].set_ylabel('Y [m]')

        # Yaw(t)
        self.axs[1,1].cla()
        self.axs[1,1].plot(ts, yaws_deg, 'c-')
        self.axs[1,1].grid(True)
        self.axs[1,1].set_title('Yaw vs Time [°]')
        self.axs[1,1].set_xlabel('Time [s]')
        self.axs[1,1].set_ylabel('Yaw [deg]')

        plt.tight_layout()
        plt.draw()
        plt.pause(0.01)

def main():
    rclpy.init()
    node = TrajectoryPlotter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        plt.ioff()
        plt.show()

if __name__ == '__main__':
    main()