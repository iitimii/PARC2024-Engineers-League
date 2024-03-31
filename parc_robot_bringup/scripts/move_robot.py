#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time


class MoveRobot(Node):
    def __init__(self):
        super().__init__("move_robot")
        self.pub = self.create_publisher(
            Twist, "/robot_base_controller/cmd_vel_unstamped", 10
        )

    def run(self):
        move_cmd = Twist()

        print("Moving Straight")
        move_cmd.linear.x = 0.3  # move in x axis at 0.3 m/s
        # move_cmd.linear.x = 0.5  # move in x axis at 0.5 m/s
        move_cmd.angular.z = 0.0

        now = time.time()
        # # For the next 4 seconds publish cmd_vel move commands
        # while time.time() - now < 4:
        # For the next 3 seconds publish cmd_vel move commands
        while time.time() - now < 3:
            self.pub.publish(move_cmd)  # publish to robot

        # print("Stopping")
        # move_cmd.linear.x = 0.0
        # move_cmd.angular.z = 0.0  # Assigning zero to both variables stops the robot
        #
        # now = time.time()
        # # For the next 5 seconds publish cmd_vel move commands
        # while time.time() - now < 5:
        #     self.pub.publish(move_cmd)

        print("Rotating")
        move_cmd.linear.x = 0.0
        move_cmd.angular.z = 0.2  # rotate at 0.2 rad/s
        # move_cmd.angular.z = 0.7  # rotate at 0.7 rad/s

        now = time.time()
        # # For the next 15 seconds publish cmd_vel move commands
        # while time.time() - now < 15:
        # For the next 3 seconds publish cmd_vel move commands
        while time.time() - now < 3:
            self.pub.publish(move_cmd)

        print("Stopping")
        move_cmd.linear.x = 0.0
        move_cmd.angular.z = 0.0

        now = time.time()
        # # For the next 3 seconds publish cmd_vel move commands
        # while time.time() - now < 3:
        # For the next 1 second publish cmd_vel move commands
        while time.time() - now < 1:
            self.pub.publish(move_cmd)

        print("Exit")


def main(args=None):
    rclpy.init(args=args)

    move_robot = MoveRobot()
    move_robot.run()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
