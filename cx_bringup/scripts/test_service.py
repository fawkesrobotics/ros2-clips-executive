#!/usr/bin/env python3
# Licensed under GPLv2. See LICENSE file. Copyright Carologistics.
import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool


class SetBoolService(Node):

    def __init__(self):
        super().__init__("set_bool_service")
        self.srv = self.create_service(SetBool, "ros_cx_client", self.set_bool_callback)

    def set_bool_callback(self, request, response):
        if request.data:
            response.success = True
            response.message = "The request was true!"
        else:
            response.success = False
            response.message = "The request was false!"
        self.get_logger().info(f"Received request: {request.data}")
        return response


def main(args=None):
    rclpy.init(args=args)
    node = SetBoolService()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
