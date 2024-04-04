#!/usr/bin/env python3

import rclpy
from rclpy.node import Node


class GetParam(Node):
    def __init__(self):
        super().__init__("get_param")

        self.declare_parameter("peg_a_latitude", rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter("peg_a_longitude", rclpy.Parameter.Type.DOUBLE)

        peg_a_lat = self.get_parameter("peg_a_latitude")
        peg_a_long = self.get_parameter("peg_a_longitude")

        self.get_logger().info(
            "Peg A location: %f %f"
            % (
                peg_a_lat.value,
                peg_a_long.value,
            )
        )


def main(args=None):
    rclpy.init(args=args)
    get_param_test = GetParam()
    rclpy.spin(get_param_test)
    get_param_test.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
