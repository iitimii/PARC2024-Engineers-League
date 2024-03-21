#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Image

import cv2
import numpy as np


class ImageSubscriber(Node):

    def __init__(self):
        super().__init__("image_subscriber")

        self.right_cam_sub = self.create_subscription(
            Image, "right_camera/image_raw", self.right_cam_callback, 1
        )
        self.right_cam_sub  # prevent unused variable warning

        # Used to convert between ROS and OpenCV images
        self.br = CvBridge()

        # self.test_image

    def right_cam_callback(self, data):

        self.get_logger().info("Receiving image from right camera")

        # Convert ROS Image message to OpenCV image
        current_frame = self.br.imgmsg_to_cv2(data)

        # Convert color from BGR to RGB
        current_frame = cv2.cvtColor(current_frame, cv2.COLOR_BGR2RGB)

        # Display image
        cv2.imshow("Right Camera", current_frame)

        cv2.waitKey(1)


def main(args=None):

    rclpy.init(args=args)

    image_subscriber = ImageSubscriber()

    # Spin the node so the callback function is called
    rclpy.spin(image_subscriber)

    image_subscriber.destroy_node()

    # rclpy.shutdown()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        rclpy.shutdown()
