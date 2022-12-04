#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8
from my_robot_interfaces.srv import ResetCounter


class NumberCounterNode(Node):
    def __init__(self):
        super().__init__("number_counter")
        self.get_logger().info("number_counter Python node initialized.")

        self.sub_ = self.create_subscription(
            Int8, 'number', self.callback_number, 10)
        self.reset_srv = self.create_service(
            ResetCounter, "reset_counter", self.reset_counter_callback)
        self.counter_ = 0

    def callback_number(self, msg):
        self.counter_ += msg.data
        self.get_logger().info(f"Counter: {self.counter_}.")

    def reset_counter_callback(self, request, response):
        if request.reset_value >= 0:
            self.counter_ = request.reset_value
            self.get_logger().info(
                f"Reset service activated, counter value is {self.counter_}.")
            response.success = True
        else:
            response.success = False

        return response


def main(args=None):
    rclpy.init(args=args)
    node = NumberCounterNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
