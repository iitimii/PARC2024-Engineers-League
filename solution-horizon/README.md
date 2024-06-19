# README

## Introduction

This project contains solutions to two tasks: autonomous navigation and crop yield counting. The autonomous navigation task involves using machine learning to control a robot's movement based on sensor data and camera images. The crop yield counting task involves using YOLOv8 to detect and count tomatoes in images captured by the robot's cameras.

## Dependencies

To run this project, you need to install the following packages:

- ROS2 Humble
- Python 3.8+
- NumPy
- OpenCV
- CvBridge
- Torch
- torchvision
- YOLOv8 (ultralytics)
- Sensor message types: `geometry_msgs`, `sensor_msgs`, `std_msgs`, `parc_robot_interfaces`

To install the required Python packages, run:

```sh
pip install numpy opencv-python cv_bridge torch torchvision ultralytics 
```

## Task 1 - Autonomous Navigation

The autonomous navigation node, `ML_NAV`, uses a multimodal neural network to predict the robot's velocity based on images from three cameras, lidar data, GPS, and IMU data.

### Running the Node

1. Ensure that the necessary models and scalers are in the specified paths in the `ML_NAV` class.
2. Launch the ROS2 environment and run the following command to start the node:

```sh
ros2 run autonomous_navigation task1_solution

```

### Node Description

The node subscribes to the following topics:

- `/left_camera/image_raw`: Receives images from the left camera.
- `/right_camera/image_raw`: Receives images from the right camera.
- `/zed2_center_camera/image_raw`: Receives images from the center camera.
- `/scan`: Receives lidar data.
- `/gps/fix`: Receives GPS data.
- `/imu_plugin/out`: Receives IMU data.

The node publishes the predicted velocity to `/robot_base_controller/cmd_vel_unstamped`.

## Task 2 - Crop Yield Counting

The crop yield counting node, `CountClass`, uses YOLOv8 to detect and count tomatoes in images from the robot's left and right cameras.

### Running the Node

1. Ensure that the YOLOv8 model is in the specified path in the `CountClass` class.
2. Launch the ROS2 environment and run the following command to start the node:

```sh
ros2 run crop_yield task2_solution
    
    ```

### Node Description

The node subscribes to the following topics:

- `/left_camera/image_raw`: Receives images from the left camera.
- `/right_camera/image_raw`: Receives images from the right camera.
- `/parc_robot/robot_status`: Receives the robot's status.

The node publishes the total count of detected tomatoes to `/parc_robot/crop_yield`.

## Challenges Faced

### Autonomous Navigation

**Sensor Data Synchronization:** Ensuring that data from different sensors are synchronized was challenging. This was crucial for accurate predictions.

**Model Deployment:** Efficiently deploying and running the neural network on different hardware (CPU, GPU) required careful handling of device compatibility.

### Crop Yield Counting

**Model Accuracy:** Ensuring that the YOLOv8 model accurately detected tomatoes in different lighting conditions and backgrounds was a challenge.

**Real-time Processing:** Processing the images in real-time without significant delays required optimization of the detection pipeline.

By addressing these challenges, the project aims to achieve robust autonomous navigation and accurate crop yield estimation using state-of-the-art machine learning techniques.

