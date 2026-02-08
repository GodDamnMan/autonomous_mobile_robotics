#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class SimplePublisher(Node):
    # Define a class that inherits from Node to create a ROS 2 publisher node.

    def __init__(self):

        super().__init__('simple_publisher')
        # Call the parent Node class's constructor with the node name 'simple_publisher'.

        self.publisher_ = self.create_publisher(String, 'topic', 10)
        # Create a publisher that publishes String messages to the 'topic' topic.
        # The '10' refers to the message queue size (depth).

        self.timer = self.create_timer(1.0, self.publish_message)
        # Create a timer that triggers the publish_message method every 1 second.

        self.counter = 0
        # Initialize a counter variable to track the number of messages published.

    def publish_message(self):

        msg = String()
        msg.data = f"Hello, ROS 2! Count: {self.counter}"
        self.publisher_.publish(msg)
        self.get_logger().info(f"Publishing: {msg.data}")
        self.counter += 1


def main(args=None):
    # Entry point for the program.

    rclpy.init(args=args)
    # Initialize the rclpy library

    node = SimplePublisher()
    # Create an instance of the SimplePublisher node.

    try:
        rclpy.spin(node)
        # Keep the node running and listening for callbacks.
    except KeyboardInterrupt:
        pass
    
    finally:
        node.destroy_node()
        # Clean up the node and release resources.

        rclpy.shutdown()
        # Shut down the rclpy library.

if __name__ == '__main__':
    main()