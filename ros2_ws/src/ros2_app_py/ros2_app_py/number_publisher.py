#!/usr/bin/env pytohn3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8


class NumberPublisherNode(Node):
    def __init__(self):
        super().__init__("number_publisher")

        # Declare parameters.
        self.declare_parameter("pub_freq", value=1.0)
        self.declare_parameter("num2pub", value=0)

        # Get value from parameters.
        self.pub_freq_ = self.get_parameter("pub_freq").value
        self.num2pub_ = self.get_parameter("num2pub").value

        self.number_timer_ = self.create_timer(
            1.0 / self.pub_freq_, self.publish_number)
        self.pub_ = self.create_publisher(Int8, 'number', 10)

        # Confirm node initialization.
        self.get_logger().info("number_publisher Python node initialized.")

    def publish_number(self):
        msg = Int8()
        msg.data = self.num2pub_
        self.pub_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = NumberPublisherNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
