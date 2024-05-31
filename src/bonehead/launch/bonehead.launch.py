import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

import xacro


def generate_launch_description():

    # Check if we're told to use sim time
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Process the URDF file
    pkg_path = os.path.join(get_package_share_directory('bonehead'))
    xacro_file = os.path.join(pkg_path,'description','bonehead.urdf.xacro')
    robot_description_config = Command(['xacro ', xacro_file])

    # Create a robot_state_publisher node
    params = {'robot_description': robot_description_config}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    # Launch the rplidar_ros node
    node_laser = Node(
        package='rplidar_ros',
        executable='rplidar_composition',
        output='screen',
        parameters=[{
            'serial_port': '/dev/serial/by-path/platform-fd5000',
            'frame_id': 'laser_frame',
            'angle_compensate': True,
            'scan_mode': 'Standard'
        }]
    )

    # Launch inverse kinematics node
    node_ik = Node(
        package='bonehead_control',
        executable='ik_node',
        output='screen'
    )

    # Launch the gait planner node
    node_gait_scheduler = Node(
        package='bonehead_control',
        executable='gait_scheduler',
        output='screen'
    )

    # Launch the teleoperation node
    node_teleoperation = Node(
        package='bonehead_teleop',
        executable='joystick_node',
        output='screen'
    )

    # Launch the serial communication node
    node_serial_comms = Node(
        package='serial_comms',
        executable='serial',
        output='screen'
    )

    # Source the servo angle topics
    source_list = ['/servo_angles']

    # Create a joint_state_publisher node
    params = {'source_list': source_list}
    node_joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        output='screen',
        parameters=[params]
    )

    # Launch!
    return LaunchDescription([
        node_robot_state_publisher,
        node_joint_state_publisher,
        node_laser,
        node_ik,
        node_gait_scheduler,
        node_teleoperation,
        node_serial_comms
    ])