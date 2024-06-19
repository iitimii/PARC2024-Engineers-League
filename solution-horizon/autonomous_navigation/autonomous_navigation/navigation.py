import rclpy
from rclpy.node import Node
import numpy as np

from sensor_msgs.msg import LaserScan, NavSatFix, Imu, Image, PointCloud
from geometry_msgs.msg import Twist, Pose, TransformStamped, Quaternion

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from parc_robot_bringup.gps2cartesian import gps_to_cartesian
from tf_transformations import euler_from_quaternion


class NavigationNodeClass(Node):
    def __init__(self):
        super().__init__("navigation_node")

        self.declare_parameter("goal_latitude", rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter("goal_longitude", rclpy.Parameter.Type.DOUBLE)

        self.goal_lat = self.get_parameter("goal_latitude").value
        self.goal_long = self.get_parameter("goal_longitude").value

        self.gps_sub = self.create_subscription(NavSatFix, "/gps/fix", self.gps_callback, 1)
        self.lidar_sub = self.create_subscription(LaserScan, "/scan", self.lidar_callback, 10)
        self.tf_buffer_ = Buffer()
        self.tf_listener_ = TransformListener(self.tf_buffer_, self)

        self.vel_pub = self.create_publisher(Twist, "/robot_base_controller/cmd_vel_unstamped", 10)

        self.timer_ = self.create_timer(1, self.timer_callback)
        self.tf_timer = self.create_timer(0.1, self.transform_callback)

        self.start_pose = Pose()
        self.start_pose.position.x = 0.0
        self.start_pose.position.y = 0.0
        self.init_start = False

        self.goal_pose = Pose()
        self.goal_pose.position.x, self.goal_pose.position.y = gps_to_cartesian(self.goal_lat, self.goal_long)
        self.get_logger().info(f"Heading to goal location x: {self.goal_pose.position.x}, y: {self.goal_pose.position.y}")

        self.current_pose = Pose()

        self.current_pose_tf = Pose()

    def lidar_callback(self, msg: LaserScan):
        fov = 30
        scan = msg.ranges[(201-fov//2):(201+fov//2)]
        scan = np.array(scan)
        left = np.linspace(0, -1, fov//2)
        right = np.linspace(1, 0, fov//2)
        both = np.concatenate([right, left], axis=0)
        both = both/(scan*scan)
        self.steering_angle = both.sum()
        # print(self.steering_angle)
        # print(f" Len: {len(scan)} \n{scan}\n\n")

    def gps_callback(self, msg: NavSatFix):
        if self.init_start == False:
            self.start_pose.position.x, self.start_pose.position.y = gps_to_cartesian(msg.latitude, msg.longitude)
            self.init_start = True
            self.get_logger().info(f"Starting at location x: {self.start_pose.position.x}, y: {self.start_pose.position.y}")

        self.current_pose.position.x, self.current_pose.position.y = gps_to_cartesian(msg.latitude, msg.longitude)
        self.get_logger().info(f"Currently at x: {self.current_pose.position.x}, y: {self.current_pose.position.y}")


    def transform_callback(self):
        requested_transform = TransformStamped()
        try:
            requested_transform = self.tf_buffer_.lookup_transform("odom", "base_footprint", rclpy.time.Time())
        except TransformException as e:
            # self.get_logger().error(f"Failed to get transform from odom to base_footprint: {e}")
            return

        if requested_transform.header.frame_id == "odom" and requested_transform.child_frame_id == "base_footprint":
            self.current_pose_tf.position.x = requested_transform.transform.translation.x
            self.current_pose_tf.position.y = requested_transform.transform.translation.y
            self.current_pose_tf.position.z = requested_transform.transform.translation.z
            self.current_pose_tf.orientation.x = requested_transform.transform.rotation.x
            self.current_pose_tf.orientation.y = requested_transform.transform.rotation.y
            self.current_pose_tf.orientation.z = requested_transform.transform.rotation.z
            self.current_pose_tf.orientation.w = requested_transform.transform.rotation.w
            quat = [self.current_pose_tf.orientation.x, self.current_pose_tf.orientation.y, self.current_pose_tf.orientation.z, self.current_pose_tf.orientation.w]
            self.robot_orientation = euler_from_quaternion(quat)
            print(f"\nRobot Orientation: {self.robot_orientation}")
            
            # self.get_logger().info(f"Currently at tf_x: {self.current_pose_tf.position.x}, tf_y: {self.current_pose_tf.position.y}")


    def timer_callback(self):
        msg = Twist()
        msg.angular.z = 1.7#max(min(self.steering_angle, 0.5), -0.5)*0.9 + (self.start_pose.position.x - self.current_pose.position.x)*0.1
        msg.linear.x = min(1.0, (self.current_pose.position.y - self.goal_pose.position.y))
        self.vel_pub.publish(msg)
        print(f"Linear: {msg.linear.x}, Angular: {msg.angular.z}")




def main(args=None):
    rclpy.init(args=args)

    node = NavigationNodeClass()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()