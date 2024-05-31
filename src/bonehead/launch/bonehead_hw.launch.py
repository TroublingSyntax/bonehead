import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


def generate_launch_description():

    # Launch the rplidar_ros node
    node_laser = Node(
        package='rplidar_ros',
        executable='rplidar_composition',
        output='screen',
        parameters=[{
            'serial_port': '/dev/ttyUSB0',
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

    # Launch the gait planner node
    node_motor_controller = Node(
        package='bonehead_control',
        executable='motor_controller',
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

    # Launch!
    return LaunchDescription([
        node_laser,
        node_ik,
        node_gait_scheduler,
        node_motor_controller,
        node_teleoperation,
        node_serial_comms
    ])