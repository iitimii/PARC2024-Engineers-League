import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from parc_robot_interfaces.msg import CropYield
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO, solutions
import os


class CountClass(Node):
    def __init__(self):
        super().__init__("count_tomatoes_node")
        print(os.getcwd())

        self.declare_parameter('model_path', value='install/crop_yield/share/crop_yield/models/best.pt')
        model_path = self.get_parameter('model_path').value
        self.line_points = [(320, 0), (320, 480)]
        self.classes_to_count = [0] 

        self.left_model = YOLO(model_path)
        self.left_counter = solutions.ObjectCounter(
            reg_pts=self.line_points,
            classes_names=self.left_model.names,
        )
        self.right_model = YOLO(model_path)
        self.right_counter = solutions.ObjectCounter(
            reg_pts=self.line_points,
            classes_names=self.right_model.names,
        )

        self.bridge = CvBridge()
        self.left_img = None
        self.right_img = None

        self.create_subscription(Image, "/left_camera/image_raw", self.left_camera_callback, 10)
        self.create_subscription(Image, "/right_camera/image_raw", self.right_camera_callback, 10)


        self.create_subscription(String, "/parc_robot/robot_status", self.end_check_callback, 1)
        self.yield_pub = self.create_publisher(CropYield, "/parc_robot/crop_yield", 1)

        self.timer_ = self.create_timer(0.001, self.timer_callback)

        self.left_count = 0
        self.right_count = 0
        self.total_count = 0

        self.get_logger().info('Started Counting Node')

    
    def left_camera_callback(self, msg):
        self.left_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def right_camera_callback(self, msg):
        self.right_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")



    def end_check_callback(self, msg: String):
        status = msg.data
        if status == 'finished':
            self.total_count = self.left_count + self.right_count
            total_yield = CropYield()
            total_yield.data = self.total_count
            self.yield_pub.publish(total_yield)


    def timer_callback(self):
        if self.left_img is not None:
            tracks = self.left_model.track(self.left_img, persist=True, show=False, classes=self.classes_to_count)
            counted_image = self.left_counter.start_counting(self.left_img, tracks)
            self.left_count = self.left_counter.out_counts + self.left_counter.in_counts 

        if self.right_img is not None:
            tracks = self.right_model.track(self.right_img, persist=True, show=False, classes=self.classes_to_count)
            counted_image = self.right_counter.start_counting(self.right_img, tracks)
            self.right_count = self.right_counter.out_counts + self.right_counter.in_counts 

        self.total_count = self.left_count + self.right_count
        


def main(args=None):
    rclpy.init(args=args)
    node = CountClass()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
