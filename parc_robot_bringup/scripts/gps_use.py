#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from parc_robot_bringup.gps2cartesian import gps_to_cartesian


class GPSUse(Node):
    def __init__(self):
        super().__init__("gps_use")

        self.gps_sub = self.create_subscription(
            NavSatFix, "/gps/fix", self.gps_callback, 1
        )

    def gps_callback(self, gps):

        #
        x, y = gps_to_cartesian(gps.latitude, gps.longitude)

        self.get_logger().info(
            "The translation from the origin (0, 0) to the gps location provided: %f %f"
            % (x, y)
        )


def main(args=None):
    rclpy.init(args=args)

    gps_use = GPSUse()

    rclpy.spin(gps_use)

    gps_use.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
