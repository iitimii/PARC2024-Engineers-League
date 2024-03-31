#!/usr/bin/env python3

import rclpy
from rclpy.node import Node


class GetParam(Node):
    def __init__(self):
        super().__init__("get_param")

        self.declare_parameter(
            "/load_task1_params/goal_latitude", rclpy.Parameter.Type.DOUBLE
        )
        self.declare_parameter(
            "/load_task1_params/goal_longitude", rclpy.Parameter.Type.DOUBLE
        )

        lat = self.get_parameter("/load_task1_params/goal_latitude")
        long = self.get_parameter("/load_task1_params/goal_longitude")

        self.get_logger().info(
            "goal location: %f %f"
            % (
                str(lat.value),
                str(long.value),
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
