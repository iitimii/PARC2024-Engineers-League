import os

import yaml

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)

from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # Set the path to different files and folders
    pkg_path = FindPackageShare(package="parc_robot_bringup").find("parc_robot_bringup")
    pkg_description = FindPackageShare(package="parc_robot_description").find(
        "parc_robot_description"
    )
    pkg_gazebo_ros = FindPackageShare(package="gazebo_ros").find("gazebo_ros")

    rviz_config_file = os.path.join(pkg_path, "rviz/parc_robot_bringup.rviz")
    gazebo_params_file = os.path.join(pkg_path, "config/gazebo_params.yaml")
    world_filename = "parc_task2.world"
    world_path = os.path.join(pkg_path, "worlds", world_filename)

    # Launch configuration variables
    use_sim_time = LaunchConfiguration("use_sim_time")
    world = LaunchConfiguration("world")
    route = LaunchConfiguration("route")

    # Declare launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name="use_sim_time",
        default_value="true",
        description="Use simulation (Gazebo) clock if true",
    )

    declare_world_cmd = DeclareLaunchArgument(
        name="world",
        default_value=world_path,
        description="Full path to the world model to load",
    )

    declare_route_cmd = DeclareLaunchArgument(
        name="route",
        default_value="route1",
        description="Route for robot navigation",
        choices=["route1", "route2", "route3"],
    )

    # Start robot state publisher
    start_robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(pkg_description, "launch", "robot_state_publisher_launch.py")]
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    # Launch Gazebo
    start_gazebo_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(pkg_gazebo_ros, "launch", "gazebo.launch.py")]
        ),
        launch_arguments={
            "world": world,
            "extra_gazebo_args": "--ros-args --params-file " + gazebo_params_file,
        }.items(),
    )

    # Function to spawn PARC robot in Gazebo with pose dependent on route chosen
    def spawn_gazebo_entities(context):

        nonlocal route, goal_location_sdf

        actions = []

        # Set path to route parameter yaml file
        params_file = os.path.join(
            pkg_path,
            "config/",
            "task1_" + context.launch_configurations["route"] + "_params.yaml",
        )

        # Open route specific yaml file
        if os.path.exists(params_file):
            with open(params_file, "r") as f:
                params = yaml.safe_load(f)

                spawn_x_val = str(params["/**"]["ros__parameters"]["x"])
                spawn_y_val = str(params["/**"]["ros__parameters"]["y"])
                spawn_z_val = str(params["/**"]["ros__parameters"]["z"])
                spawn_yaw_val = str(params["/**"]["ros__parameters"]["yaw"])

                route_params_file = LaunchConfiguration("route_params_file")

                # Declare route params file launch argument
                actions.append(
                    DeclareLaunchArgument(
                        name="route_params_file",
                        default_value=params_file,
                    )
                )

                # Load route parameters file
                actions.append(
                    Node(
                        package="parc_robot_bringup",
                        executable="load_task1_params.py",
                        parameters=[route_params_file],
                    )
                )

                # Spawn PARC robot in Gazebo
                actions.append(
                    Node(
                        package="gazebo_ros",
                        executable="spawn_entity.py",
                        output="screen",
                        arguments=[
                            "-topic",
                            "robot_description",
                            "-entity",
                            "parc_robot",
                            "-x",
                            spawn_x_val,
                            "-y",
                            spawn_y_val,
                            "-z",
                            spawn_z_val,
                            "-Y",
                            spawn_yaw_val,
                        ],
                    )
                )

        return actions

    # Publish the joint state values for the non-fixed joints in the URDF file.
    start_joint_state_publisher_cmd = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
    )

    # Launch RViz
    start_rviz_cmd = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_route_cmd)

    # Add any actions
    ld.add_action(start_gazebo_cmd)
    ld.add_action(start_rviz_cmd)
    ld.add_action(start_joint_state_publisher_cmd)
    ld.add_action(OpaqueFunction(function=spawn_gazebo_entities))
    ld.add_action(start_robot_state_publisher_cmd)

    return ld
