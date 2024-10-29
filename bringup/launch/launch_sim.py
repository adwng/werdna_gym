import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():    

    bringup_prefix = get_package_share_directory('bringup')
    description_prefix = get_package_share_directory('description')
    
    rsp_launch_file = os.path.join(description_prefix, 'launch', 'rsp.launch.py')
    gazebo_launch_file = os.path.join(description_prefix, 'launch', 'gazebo.launch.py')

    teleop_launch_file =os.path.join(bringup_prefix, 'launch', 'joystick.launch.py')

    twist_mux_config = os.path.join(bringup_prefix, 'config', 'twist_mux.yaml')

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rsp_launch_file),
        launch_arguments = {'use_sim_time': 'false'}.items()
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_file)
    )

    teleop = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(teleop_launch_file)
    )

    diff_drive_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['palmvision_base_controller']
    )

    position_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["servo_base_controller"]
    )

    joint_broad_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster']
    )

    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        parameters=[twist_mux_config, {'use_sim_time': True }],
        remappings=[('/cmd_vel_out', '/palmvision_base_controller/cmd_vel_unstamped')]
    )

    
    return LaunchDescription(
        [
            teleop,
            twist_mux, 
            rsp, 
            gazebo, 
            diff_drive_spawner, 
            joint_broad_spawner,
            position_controller
        ]
    )