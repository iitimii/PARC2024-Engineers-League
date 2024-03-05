import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction

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

    gazebo_params_file = os.path.join(pkg_path, "config/gazebo_params.yaml")
    world_filename = "parc_task1.world"
    world_path = os.path.join(pkg_path, "worlds", world_filename)

    # Launch configuration variables
    use_sim_time = LaunchConfiguration("use_sim_time")
    world = LaunchConfiguration("world")

    # Declare launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name="use_sim_time",
        default_value="True",
        description="Use simulation (Gazebo) clock if true",
    )

    declare_world_cmd = DeclareLaunchArgument(
        name="world",
        default_value=world_path,
        description="Full path to the world model to load",
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

    # Spawn PARC robot in Gazebo
    start_spawner_cmd = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        output="screen",
        arguments=["-topic", "robot_description", "-entity", "parc_robot"],
    )

    # Spawn robot_base_controller
    start_robot_base_controller_cmd = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["robot_base_controller"],
    )

    start_delayed_robot_base_controller_cmd = TimerAction(
        period=4.0, actions=[start_robot_base_controller_cmd]
    )

    # Spawn joint_state_broadcaser
    start_joint_broadcaster_cmd = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broadcaster"],
    )

    start_delayed_joint_broadcaster_cmd = TimerAction(
        period=4.0, actions=[start_joint_broadcaster_cmd]
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_world_cmd)

    # TODO: add route launch argument

    # Add any actions
    ld.add_action(start_gazebo_cmd)
    ld.add_action(start_spawner_cmd)
    ld.add_action(start_robot_state_publisher_cmd)
    # ld.add_action(start_robot_base_controller_cmd)
    # ld.add_action(start_joint_broadcaster_cmd)
    ld.add_action(start_delayed_robot_base_controller_cmd)
    ld.add_action(start_delayed_joint_broadcaster_cmd)

    return ld
