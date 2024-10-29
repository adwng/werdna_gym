from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import DeclareLaunchArgument
from launch_ros.substitutions import FindPackageShare

import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # joy_params = os.path.join(get_package_share_directory('palmvision_control'),'config','joystick.yaml')

    joy_params = PathJoinSubstitution(
        [
            FindPackageShare("palmvision_control"),
            "config",
            "joystick.yaml",
        ]
    )
    joy_node = Node(
            package='joy',
            executable='joy_node',
            parameters=[joy_params]
         )

    teleop_node = Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_node',
            parameters=[joy_params],
            remappings=[('/cmd_vel','/palmvision_base_controller/cmd_vel_unstamped')]
         )


    return LaunchDescription([
        joy_node,
        teleop_node,     
    ])