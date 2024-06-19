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
import pickle
import torch
from torch import nn
from torchvision import transforms

class ML_NAV(Node):
    def __init__(self):
        super().__init__("ml_nav_node")

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

        self.declare_parameter('model_path', value='src/PARC2024-Engineers-League/autonomous_navigation/autonomous_navigation/models/best_cat.pth')
        self.declare_parameter('lidar_scaler_path', value='src/PARC2024-Engineers-League/autonomous_navigation/autonomous_navigation/models/lidar_scaler.pkl')
        self.declare_parameter('sensor_scaler_path', value='src/PARC2024-Engineers-League/autonomous_navigation/autonomous_navigation/models/sensor_scaler.pkl')

        self.goal_lat = self.get_parameter("goal_latitude").value
        self.goal_long = self.get_parameter("goal_longitude").value
        self.goal = np.array([self.goal_lat, self.goal_long])

        model_path = self.get_parameter('model_path').value
        lidar_scaler_path = self.get_parameter('lidar_scaler_path').value
        sensor_scaler_path = self.get_parameter('sensor_scaler_path').value

        self.create_subscription(Image, "/left_camera/image_raw", self.left_camera_callback, 10)
        self.create_subscription(Image, "/right_camera/image_raw", self.right_camera_callback, 10)
        self.create_subscription(Image, "/zed2_center_camera/image_raw", self.centre_camera_callback, 10)
        self.create_subscription(LaserScan, "/scan", self.lidar_callback, 10)
        self.create_subscription(NavSatFix, "/gps/fix", self.gps_callback, 1)
        self.create_subscription(Imu, "/imu_plugin/out", self.imu_callback, 10)

        self.vel_pub = self.create_publisher(Twist, "/robot_base_controller/cmd_vel_unstamped", 10)

        self.create_timer(1, self.timer_callback)

        self.model = CategoricalMultimodalNetwork(0.1)
        self.model.load_state_dict(torch.load(model_path))
        self.model.eval()

        with open(lidar_scaler_path,'rb') as f:
            self.lidar_scaler = pickle.load(f)

        with open(sensor_scaler_path,'rb') as f:
            self.sensor_scaler = pickle.load(f)

        if torch.cuda.is_available():
            self.device = torch.device("cuda")
        elif torch.backends.mps.is_available():
            self.device = torch.device("mps")
        else:
            self.device = torch.device("cpu")

        self.model.to(self.device)

        self.transform = transforms.Compose([
            transforms.ToTensor(),
            transforms.Resize((224, 224)),
            transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])
        ])

        self.get_logger().info("ML Navigation Node initialized")

    def predict_vel(self, img, lidar_data, sensor_features):
        img = self.transform(img).to(self.device).unsqueeze(0)
        lidar_data = torch.tensor(lidar_data, dtype=torch.float32).unsqueeze(0).unsqueeze(0).to(self.device)
        sensor_features = torch.tensor(sensor_features, dtype=torch.float32).unsqueeze(0).to(self.device)

        img = self.transform(img).to(self.device)
        lidar_data = self.lidar_scaler.transform(lidar_data)
        sensor_features = self.sensor_scaler.transform(sensor_features)

        with torch.no_grad():
            output = self.model(img, lidar_data, sensor_features)
            output = output.view(-1, 2, 3)

        softmax = nn.Softmax(dim=2)
        probabilities = softmax(output)
        
        predictions = torch.argmax(probabilities, dim=2)
        predictions = predictions - 1
        
        return predictions

    def left_camera_callback(self, msg: Image):
        self.left_img = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    
    def right_camera_callback(self, msg: Image):
        self.right_img = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    
    def centre_camera_callback(self, msg: Image):
        self.centre_img = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    
    def lidar_callback(self, msg: LaserScan):
        self.lidar_data_temp = np.array(msg.ranges)

    def gps_callback(self, msg: NavSatFix):
        self.gps = np.array([msg.latitude, msg.longitude])

    def imu_callback(self, msg: Imu):
        self.imu = np.array([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w,
                             msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z,
                             msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])
        
    def timer_callback(self):
        if self.left_img is not None and self.right_img is not None and self.centre_img is not None and self.lidar_data_temp is not None and self.gps is not None and self.imu is not None:
            sensor_features = np.concatenate((self.gps, self.imu, self.goal))
            predictions = self.predict_vel(self.centre_img, self.lidar_data_temp, sensor_features)

            twist_msg = Twist()
            twist_msg.linear.x = predictions[0][0].item()
            twist_msg.angular.z = predictions[0][1].item()

            self.vel_pub.publish(twist_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ML_NAV()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()





class ImageFeatureExtractor(nn.Module):
    def __init__(self, drop):
        super(ImageFeatureExtractor, self).__init__()
        self.model = nn.Sequential(
            nn.Conv2d(in_channels=9, out_channels=32, kernel_size=5),
            nn.ReLU(),
            nn.MaxPool2d(2, 2),
            nn.Dropout(p=drop),  # Dropout added

            nn.Conv2d(in_channels=32, out_channels=64, kernel_size=3),
            nn.ReLU(),
            nn.MaxPool2d(2, 2),
            nn.Dropout(p=drop),  # Dropout added

            nn.Conv2d(in_channels=64, out_channels=64, kernel_size=3),
            nn.ReLU(),
            nn.MaxPool2d(2, 2),
            nn.Dropout(p=drop),  # Dropout added

            nn.Conv2d(in_channels=64, out_channels=128, kernel_size=3),
            nn.ReLU(),
            nn.MaxPool2d(2, 2),
            nn.Dropout(p=drop),  # Dropout added

            nn.Conv2d(in_channels=128, out_channels=64, kernel_size=3),
            nn.ReLU(),
            nn.MaxPool2d(2, 2),
            nn.Dropout(p=drop),  # Dropout added
            
            nn.Conv2d(in_channels=64, out_channels=32, kernel_size=3),
            nn.ReLU(),
            nn.Dropout(p=drop),  # Dropout added
            
            nn.Conv2d(in_channels=32, out_channels=32, kernel_size=3),
            nn.ReLU(),
            nn.Dropout(p=drop),  # Dropout added
            nn.Flatten()
        )

    def forward(self, x):
        return self.model(x)

class LiDARFeatureExtractor(nn.Module):
    def __init__(self, drop):
        super(LiDARFeatureExtractor, self).__init__()
        self.model = nn.Sequential(
            nn.Conv1d(in_channels=1, out_channels=16, kernel_size=5, stride=1),
            nn.ReLU(),
            nn.MaxPool1d(kernel_size=2, stride=2),
            nn.Dropout(p=drop),  # Dropout added

            nn.Conv1d(in_channels=16, out_channels=32, kernel_size=3, stride=1),
            nn.ReLU(),
            nn.MaxPool1d(kernel_size=2, stride=2),
            nn.Dropout(p=drop),  # Dropout added

            nn.Conv1d(in_channels=32, out_channels=32, kernel_size=3, stride=1),
            nn.ReLU(),
            nn.MaxPool1d(kernel_size=2, stride=2),
            nn.Dropout(p=drop),  # Dropout added
            
            nn.Conv1d(in_channels=32, out_channels=32, kernel_size=3, stride=1),
            nn.ReLU(),
            nn.MaxPool1d(kernel_size=2, stride=2),
            nn.Dropout(p=drop),  # Dropout added
            
            nn.Conv1d(in_channels=32, out_channels=16, kernel_size=3, stride=1),
            nn.ReLU(),
            nn.MaxPool1d(kernel_size=2, stride=2),
            nn.Dropout(p=drop),  # Dropout added
            
            nn.Conv1d(in_channels=16, out_channels=16, kernel_size=3, stride=1),
            nn.ReLU(),
            nn.MaxPool1d(kernel_size=2, stride=2),
            nn.Dropout(p=drop),  # Dropout added
            
            nn.Conv1d(in_channels=16, out_channels=16, kernel_size=3, stride=1),
            nn.ReLU(),
            nn.Dropout(p=drop),  # Dropout added
            nn.Flatten()
        )

    def forward(self, x):
        return self.model(x)
    
class SensorFeatureExtractor(nn.Module):
    def __init__(self, drop):
        super(SensorFeatureExtractor, self).__init__()
        self.fc = nn.Sequential(
            nn.Linear(14, 32),  # GPS (2) + IMU (10) + Goal (2)
            nn.ReLU(),
            nn.Dropout(p=drop),  # Dropout added
            nn.Linear(32, 32),
            nn.ReLU(),
            nn.Dropout(p=drop),  # Dropout added
            nn.Flatten()
        )

    def forward(self, x):
        return self.fc(x)
    
class MultimodalNetwork(nn.Module):
    def __init__(self, drop):
        super(MultimodalNetwork, self).__init__()
        self.img_extractor = ImageFeatureExtractor(drop)
        self.lidar_extractor = LiDARFeatureExtractor(drop)
        self.sensor_extractor = SensorFeatureExtractor(drop)
        self.fc = nn.Sequential(
            nn.Linear(32 + 32 + 32, 64),
            nn.ReLU(),
            nn.Dropout(p=drop),  # Dropout added
            nn.Linear(64, 32),  # Outputs: linear_x, angular_z
            nn.ReLU(),
            nn.Dropout(p=drop),  # Dropout added
            nn.Linear(32, 16),
            nn.ReLU(),
            nn.Dropout(p=drop),  # Dropout added
            nn.Linear(16, 8),
            nn.ReLU(),
            nn.Dropout(p=drop),  # Dropout added
            nn.Linear(8, 4),
            nn.ReLU(),
            nn.Dropout(p=drop),  # Dropout added
            nn.Linear(4, 2),
            nn.Tanh(),
        )

    def forward(self, img, lidar, sensor):
        image_features = self.img_extractor(img)
        lidar_features = self.lidar_extractor(lidar)
        sensor_features = self.sensor_extractor(sensor)
        combined_features = torch.cat((image_features, lidar_features, sensor_features), dim=1)
        return self.fc(combined_features)


class CategoricalMultimodalNetwork(nn.Module):
    def __init__(self, drop):
        super(CategoricalMultimodalNetwork, self).__init__()
        self.img_extractor = ImageFeatureExtractor(drop)
        self.lidar_extractor = LiDARFeatureExtractor(drop)
        self.sensor_extractor = SensorFeatureExtractor(drop)
        self.fc = nn.Sequential(
            nn.Linear(32 + 32 + 32, 64),
            nn.ReLU(),
            nn.Dropout(p=drop),
            nn.Linear(64, 32),
            nn.ReLU(),
            nn.Dropout(p=drop),
            nn.Linear(32, 16),
            nn.ReLU(),
            nn.Dropout(p=drop),
            nn.Linear(16, 8),
            nn.ReLU(),
            nn.Dropout(p=drop),
            nn.Linear(8, 6)  # 2 elements each with 3 classes (-1, 0, 1)
        )

    def forward(self, img, lidar, sensor):
        image_features = self.img_extractor(img)
        lidar_features = self.lidar_extractor(lidar)
        sensor_features = self.sensor_extractor(sensor)
        combined_features = torch.cat((image_features, lidar_features, sensor_features), dim=1)
        output = self.fc(combined_features)
        return output.view(-1, 2, 3)  # Reshape for 2 elements each with 3 class logits