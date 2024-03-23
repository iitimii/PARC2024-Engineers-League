# PARC2024-Engineers-League
Pan-African Robotics Competition Engineers League category project development for the year 2024.

## Package Overview
- [`parc_robot_bringup`](./parc_robot_bringup/) : Contains config, world, scripts and launch files to bringup PARC robot for both task 1 and task 2. 
- [`parc_robot_description`](./parc_robot_description/) : Contains the URDF description files for PARC robot, sensors, launch files for robot state publisher and

***Work in Progress***

## Development machine setup

With ROS2 Humble installed, create a workspace on the dev machine and clone this repository:

```
mkdir -p ~/dev_ws/src
cd ~/dev_ws/src
git clone https://github.com/PARC-Robotics/PARC2024-Engineers-League.git .
```

Install ROS dependencies for the PARC robot packages:
```
cd ~/dev_ws
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro humble -r -y
```

## Gazebo Classic

### Gazebo model setup
Navigate to models folder in the `parc_robot_bringup` package, then copy its directory contents to the Gazebo model directory on the PC.

```
cd ~/dev_ws/src/parc_robot_bringup/models
cp -R . ~/.gazebo/models
```

### Geographiclib Installation

```
pip install geographiclib
```


## Task 1

Launch task 1 (choose either `route1`, `route2` or `route3` for PARC robot initial navigation pose):

```
ros2 launch parc_robot_bringup task1_launch.py route:=route2
```

Default route is `route1`.

## Teleop control

To control PARC robot using a keyboard, ensure that `task1_launch.py` is running, open up a new terminal then execute this command:

```
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap \
/cmd_vel:=/robot_base_controller/cmd_vel_unstamped
```
