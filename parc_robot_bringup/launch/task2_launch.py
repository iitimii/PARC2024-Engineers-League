import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
    RegisterEventHandler,
)

from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# PARC robot spawn pose
spawn_x_val = "6.085959"
spawn_y_val = "0.664742"
spawn_z_val = "0.028982"
spawn_yaw_val = "-3.103652"


def generate_launch_description():

    # Set the path to different files and folders
    pkg_path = FindPackageShare(package="parc_robot_bringup").find("parc_robot_bringup")
    pkg_description = FindPackageShare(package="parc_robot_description").find(
        "parc_robot_description"
    )
    pkg_gazebo_ros = FindPackageShare(package="gazebo_ros").find("gazebo_ros")

    gazebo_params_file = os.path.join(pkg_path, "config/gazebo_params.yaml")
    rviz_config_file = os.path.join(pkg_path, "rviz/task2.rviz")

    # Launch configuration variables
    use_sim_time = LaunchConfiguration("use_sim_time")
    world = LaunchConfiguration("world")
    speed = LaunchConfiguration("speed")

    # Declare launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name="use_sim_time",
        default_value="true",
        description="Use simulation (Gazebo) clock if true",
    )

    declare_world_cmd = DeclareLaunchArgument(
        name="world",
        default_value="world1",
        description="World model to load",
        choices=["world1", "world2", "world3"],
    )

    declare_speed_cmd = DeclareLaunchArgument(
        name="speed",
        default_value="0.1",
        description="PARC robot navigation speed in m/s",
    )

    # Start robot state publisher
    start_robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(pkg_description, "launch", "robot_state_publisher_launch.py")]
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    # Function to launch Gazebo, and PARC robot, dependent on the world file chosen
    def launch_gazebo(context):

        nonlocal world

        # List of actions to be added to the launch description later
        actions = []

        world_file = os.path.join(
            pkg_path, "worlds", context.launch_configurations["world"] + ".world"
        )

        # World path launch configuration variable
        world_path = LaunchConfiguration("world_path_file")

        # Declare world path file launch argument
        actions.append(
            DeclareLaunchArgument(
                name="world_path_file",
                default_value=world_file,
                description="Full path to the world model to load",
            )
        )

        # Launch Gazebo
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [os.path.join(pkg_gazebo_ros, "launch", "gazebo.launch.py")]
                ),
                launch_arguments={
                    "world": world_path,
                    "extra_gazebo_args": "--ros-args --params-file "
                    + gazebo_params_file,
                }.items(),
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

    # Spawn robot_base_controller
    start_robot_base_controller_cmd = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "robot_base_controller",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    # Delayed start_robot_base_controller_cmd action
    start_delayed_robot_base_controller_cmd = TimerAction(
        period=4.0, actions=[start_robot_base_controller_cmd]
    )

    # Spawn joint_state_broadcaser
    start_joint_broadcaster_cmd = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broadcaster", "--controller-manager", "/controller_manager"],
    )

    # Delayed joint_broadcaster_cmd action
    start_delayed_joint_broadcaster_cmd = TimerAction(
        period=4.0, actions=[start_joint_broadcaster_cmd]
    )

    # Launch RViz
    start_rviz_cmd = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
    )

    # Launch PARC robot route navigation node
    start_parc_nav_cmd = Node(
        package="parc_robot_bringup",
        executable="parc_robot_nav.py",
        output="screen",
        arguments=[speed],
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_speed_cmd)

    # Add any actions
    ld.add_action(OpaqueFunction(function=launch_gazebo))
    ld.add_action(start_rviz_cmd)
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(start_delayed_robot_base_controller_cmd)
    ld.add_action(start_delayed_joint_broadcaster_cmd)
    ld.add_action(
        RegisterEventHandler(
            OnProcessExit(
                target_action=start_joint_broadcaster_cmd,
                on_exit=[start_parc_nav_cmd],
            )
        )
    )

    return ld
