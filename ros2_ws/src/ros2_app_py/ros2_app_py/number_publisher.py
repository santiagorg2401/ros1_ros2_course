#!/usr/bin/env pytohn3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8


class NumberPublisherNode(Node):
    def __init__(self):
        super().__init__("number_publisher")
        self.get_logger().info("number_publisher Python node initialized.")

        self.number_ = 4
        self.number_timer_ = self.create_timer(0.5, self.publish_number)
        self.pub_ = self.create_publisher(Int8, 'number', 10)

    def publish_number(self):
        msg = Int8()
        msg.data = self.number_
        self.pub_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = NumberPublisherNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
