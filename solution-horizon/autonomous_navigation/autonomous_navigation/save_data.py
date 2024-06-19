import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import NavSatFix, Imu, Image, LaserScan
import numpy as np
import cv2
from cv_bridge import CvBridge
import os
import csv
from datetime import datetime

now = datetime.now()
dt_string = now.strftime("%d_%m_%Y_%H_%M_%S")

class SaveDataNode(Node):
    def __init__(self):
        super().__init__("save_data_node")

        self.goal = None
        self.left_img = None
        self.right_img = None
        self.centre_img = None
        self.lidar_data = []
        self.gps = None
        self.imu = None
        self.vel = None

        self.cv_bridge = CvBridge()
        self.declare_parameter("goal_latitude", rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter("goal_longitude", rclpy.Parameter.Type.DOUBLE)

        self.goal_lat = self.get_parameter("goal_latitude").value
        self.goal_long = self.get_parameter("goal_longitude").value
        self.goal = np.array([self.goal_lat, self.goal_long])

        self.create_subscription(Image, "/left_camera/image_raw", self.left_camera_callback, 10)
        self.create_subscription(Image, "/right_camera/image_raw", self.right_camera_callback, 10)
        self.create_subscription(Image, "/zed2_center_camera/image_raw", self.centre_camera_callback, 10)
        self.create_subscription(LaserScan, "/scan", self.lidar_callback, 10)
        self.create_subscription(NavSatFix, "/gps/fix", self.gps_callback, 1)
        self.create_subscription(Imu, "/imu_plugin/out", self.imu_callback, 10)
        self.create_subscription(Twist, "/robot_base_controller/cmd_vel_unstamped", self.vel_callback, 10)
        
        self.create_timer(1, self.timer_callback)

        self.data_directory = f"data/run_{dt_string}"
        self.left_img_directory = os.path.join(self.data_directory, "left_img")
        self.right_img_directory = os.path.join(self.data_directory, "right_img")
        self.centre_img_directory = os.path.join(self.data_directory, "centre_img")
        if not os.path.exists(self.data_directory):
            os.makedirs(self.data_directory)
            os.makedirs(self.left_img_directory)
            os.makedirs(self.right_img_directory)
            os.makedirs(self.centre_img_directory)

        self.csv_file_path = os.path.join(self.data_directory, f"sensor_data.csv")
        self.lidar_file_path = os.path.join(self.data_directory, "lidar_data.npy")
        self.init_csv_file()
        self.get_logger().info(f"Starting Save Node")

    def init_csv_file(self):
        with open(self.csv_file_path, mode='w') as file:
            writer = csv.writer(file)
            writer.writerow(["timestamp", "goal_latitude", "goal_longitude", "gps_latitude", "gps_longitude", "imu_orientation_x", "imu_orientation_y", "imu_orientation_z", "imu_orientation_w",
                             "imu_angular_velocity_x", "imu_angular_velocity_y", "imu_angular_velocity_z",
                             "imu_linear_acceleration_x", "imu_linear_acceleration_y", "imu_linear_acceleration_z",
                             "vel_linear_x", "vel_angular_z"])

    def left_camera_callback(self, msg: Image):
        self.left_img = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    
    def right_camera_callback(self, msg: Image):
        self.right_img = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    
    def centre_camera_callback(self, msg: Image):
        self.centre_img = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    
    def lidar_callback(self, msg: LaserScan):
        self.lidar_data_temp = msg.ranges

    def gps_callback(self, msg: NavSatFix):
        self.gps = np.array([msg.latitude, msg.longitude])

    def imu_callback(self, msg: Imu):
        self.imu = np.array([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w,
                             msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z,
                             msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])
        
    def vel_callback(self, msg: Twist):
        self.vel = np.array([msg.linear.x, msg.angular.z])

    def timer_callback(self):
        self.save_data()

    def save_data(self):
        if (self.left_img is not None and self.right_img is not None and self.centre_img is not None and
            self.lidar_data_temp is not None and self.gps is not None and self.imu is not None and self.vel is not None):

            timestamp_sec = self.get_clock().now().seconds_nanoseconds()[0]
            timestamp_nanosec = self.get_clock().now().seconds_nanoseconds()[1]
            timestamp = f"{timestamp_sec}_{timestamp_nanosec}"

            # Save images
            left_img_path = os.path.join(self.left_img_directory, f"left_img_{timestamp}.png")
            right_img_path = os.path.join(self.right_img_directory, f"right_img_{timestamp}.png")
            centre_img_path = os.path.join(self.centre_img_directory, f"centre_img_{timestamp}.png")
            cv2.imwrite(left_img_path, self.left_img)
            cv2.imwrite(right_img_path, self.right_img)
            cv2.imwrite(centre_img_path, self.centre_img)

            # Save all lidar data to a single file
            self.lidar_data.append(self.lidar_data_temp)
            np.save(self.lidar_file_path, np.array(self.lidar_data))

            # Save other sensor data to CSV
            with open(self.csv_file_path, mode='a') as file:
                writer = csv.writer(file)
                writer.writerow([timestamp] + self.goal.tolist() + self.gps.tolist() + self.imu.tolist() + self.vel.tolist())

            self.get_logger().info(f"Data saved at timestamp {timestamp}")


def main(args=None):
    rclpy.init(args=args)
    node = SaveDataNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
