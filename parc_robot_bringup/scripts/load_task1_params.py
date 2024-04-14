#!/usr/bin/python3

import rclpy
from rclpy.node import Node


class LoadTask1Params(Node):

    def __init__(self):
        super().__init__("load_task1_params")
        self.declare_parameters(
            namespace="",
            parameters=[
                ("x", 0.0),
                ("y", 0.0),
                ("z", 0.0),
                ("yaw", 0.0),
                ("goal_x", 0.0),
                ("goal_y", 0.0),
                ("goal_z", 0.0),
                ("goal_latitude", 0.0),
                ("goal_longitude", 0.0),
                ("origin_latitude", 0.0),
                ("origin_longitude", 0.0),
            ],
        )


def main(args=None):
    rclpy.init(args=args)

    load_task1_params = LoadTask1Params()

    rclpy.spin(load_task1_params)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
