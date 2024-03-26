#!/usr/bin/python3

import rclpy
from rclpy.node import Node


class LoadTask2Params(Node):

    def __init__(self):
        super().__init__("load_task2_params")
        self.declare_parameters(
            namespace="",
            parameters=[
                ("x", 0.0),
                ("y", 0.0),
                ("z", 0.0),
                ("yaw", 0.0),
                ("origin_latitude", 0.0),
                ("origin_longitude", 0.0),
            ],
        )


def main(args=None):
    rclpy.init(args=args)

    load_task2_params = LoadTask2Params()

    rclpy.spin(load_task2_params)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
