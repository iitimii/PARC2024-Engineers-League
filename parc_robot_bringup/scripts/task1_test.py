#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from parc_robot_bringup.gps2cartesian import gps_to_cartesian


class Task1Test(Node):

    def __init__(self):
        super().__init__("task1_test_node")
        self.declare_parameters(
            namespace='',
            parameters=[
                ('x', 0.0),
                ('y', 0.0),
                ('z', 0.0),
                ('yaw', 0.0),
                ('goal_x', 0.0),
                ('goal_y', 0.0),
                ('goal_z', 0.0),
                ('goal_latitude', 0.0),
                ('goal_longitude', 0.0),
                ('origin_latitude', 0.0),
                ('origin_longitude', 0.0),
            ])

        self.origin_latitude = self.get_parameter('origin_latitude').value
        self.origin_longitude = self.get_parameter('origin_longitude').value

        # self.origin_lat = 0.0
        # self.origin_long = 0.0

        self.subscription = self.create_subscription(
            NavSatFix, "gps/fix", self.listener_callback, 10
        )
        self.subscription

    def listener_callback(self, msg):
        # self.get_logger().info('x: "%.3f"' msg.data)
        # x, y = gps_to_cartesian(self.origin_latitude, self.origin_longitude, msg.latitude, msg.longitude)
        # x, y = gps_to_cartesian(self.origin_lat, self.origin_long, msg.latitude, msg.longitude)
        x, y = gps_to_cartesian(msg.latitude, msg.longitude)
        self.get_logger().info(
            "The gps location provided is {:.3f}, {:.3f} m.".format(x, y)
        )


def main(args=None):
    rclpy.init(args=args)

    task1_test_node = Task1Test()

    rclpy.spin(task1_test_node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
