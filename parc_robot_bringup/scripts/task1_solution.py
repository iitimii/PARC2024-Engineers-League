#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Twist
from parc_robot_bringup.gps2cartesian import gps_to_cartesian

import time


class TaskNav(Node):
    def __init__(self):
        super().__init__("task_nav")

        # Declare latitude and longitude parameters
        self.declare_parameter("goal_latitude", rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter("goal_longitude", rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter("peg_a_latitude", rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter("peg_a_longitude", rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter("peg_b_latitude", rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter("peg_b_longitude", rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter("peg_c_latitude", rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter("peg_c_longitude", rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter("peg_d_latitude", rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter("peg_d_longitude", rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter("peg_e_latitude", rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter("peg_e_longitude", rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter("peg_f_latitude", rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter("peg_f_longitude", rclpy.Parameter.Type.DOUBLE)

        # Get latitude and longitude values from world coordinates yaml file
        self.goal_lat = self.get_parameter("goal_latitude").value
        self.goal_long = self.get_parameter("goal_longitude").value
        self.peg_a_lat = self.get_parameter("peg_a_latitude").value
        self.peg_a_long = self.get_parameter("peg_a_longitude").value
        self.peg_b_lat = self.get_parameter("peg_b_latitude").value
        self.peg_b_long = self.get_parameter("peg_b_longitude").value
        self.peg_c_lat = self.get_parameter("peg_c_latitude").value
        self.peg_c_long = self.get_parameter("peg_c_longitude").value
        self.peg_d_lat = self.get_parameter("peg_d_latitude").value
        self.peg_d_long = self.get_parameter("peg_d_longitude").value
        self.peg_e_lat = self.get_parameter("peg_e_latitude").value
        self.peg_e_long = self.get_parameter("peg_e_longitude").value
        self.peg_f_lat = self.get_parameter("peg_f_latitude").value
        self.peg_f_long = self.get_parameter("peg_f_longitude").value

        # Subscribe to gps topic
        self.gps_sub = self.create_subscription(
            NavSatFix, "/gps/fix", self.gps_callback, 1
        )

        # Create a publisher to /robot_base_controller/cmd_vel_unstamped topic
        self.pub = self.create_publisher(
            Twist, "/robot_base_controller/cmd_vel_unstamped", 10
        )

        self.lane_counter = 0  #
        self.lane_length = 6.76  #
        self.lane_width = 1.65  #    (4.875+0.075)/3

    #
    def gps_callback(self, gps):

        # Get the cartesian coordinates from the GPS coordinates
        robot_x, robot_y = gps_to_cartesian(gps.latitude, gps.longitude)

        # Print cartesian coordinates
        self.get_logger().info(
            "The translation from the origin (0, 0) to the gps location provided: %.3f %.3f"
            % (robot_x, robot_y)
        )

    # Peg proximity check
    def peg_prox_check(self, robot_x, robot_y):
        pass

    # Lane completion check
    def lane_check(self):
        pass

    #
    def run(self):

        move_cmd = Twist()

        move_cmd.linear.x = 0.5
        move_cmd.angular.z = 0.0

        now = time.time()
        while time.time() - now < 20:
            self.pub.publish(move_cmd)


def main(args=None):
    rclpy.init(args=args)

    task_nav = TaskNav()
    task_nav.run()
    rclpy.spin(task_nav)

    task_nav.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
