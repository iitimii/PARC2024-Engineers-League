# Launch teleop node to drive the robot

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")

    joy_node = Node(
        package="joy",
        executable="joy_node",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    teleop_node = Node(
        package="teleop_twist_keyboard",
        executable="teleop_twist_keyboard",
        parameters=[{"use_sim_time": use_sim_time}],
        remappings=[("/cmd_vel", "/robot_base_controller/cmd_vel_unstamped")],
    )

    return LaunchDescription([joy_node, teleop_node])
