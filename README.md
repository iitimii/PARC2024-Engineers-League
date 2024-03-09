# PARC2024-Engineers-League
Pan-African Robotics Competition Engineers League category project development for the year 2024.

## Package Overview
- [`parc_robot_bringup`](./parc_robot_bringup/) : Contains config, world, scripts and launch files to bringup PARC robot for both task 1 and task 2. 
- [`parc_robot_description`](./parc_robot_description/) : Contains the URDF description files for PARC robot, sensors and `ros2 control`.
- [`parc_robot_msgs`](./parc_robot_navigation/) : Contains custom messages

***Work in Progress***

## ROS2 Installation

## Gazebo

### Gazebo model setup


### Geographiclib Installation

```
pip install geographiclib
```

### Task 1

Launch task 1 (choose either `route1`, `route2` or `route3` for PARC robot initial navigation pose):

```
ros2 launch parc_robot_bringup task1_launch.py route:=route2
```

Default route is `route1`.
