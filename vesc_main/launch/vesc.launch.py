import os
import launch
import launch_ros
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    racecar_version = LaunchConfiguration('racecar_version')
    vesc_config = LaunchConfiguration('vesc_config')
    car_name = LaunchConfiguration('car_name')

    return LaunchDescription([
        DeclareLaunchArgument(
            'racecar_version',
            description='Version of the racecar'
        ),
        DeclareLaunchArgument(
            'vesc_config',
            default_value=PathJoinSubstitution([
                FindPackageShare('vesc_main'),
                'config',
                racecar_version,
                'vesc.yaml'
            ]),
            description='Path to VESC config file'
        ),
        DeclareLaunchArgument(
            'car_name',
            default_value='car',
            description='Name of the car'
        ),

        # Note: ROS2 doesn't have rosparam equivalent in launch, parameters are loaded in nodes

        Node(
            package='vesc_ackermann',
            executable='ackermann_to_vesc_node',
            name='ackermann_to_vesc',
            parameters=[vesc_config],
            remappings=[
                ('ackermann_cmd', ['/', car_name, '/mux/output']),
                ('commands/motor/speed', 'commands/motor/unsmoothed_speed'),
                ('commands/servo/position', 'commands/servo/unsmoothed_position')
            ]
        ),

        Node(
            package='vesc_driver',
            executable='throttle_interpolator.py',
            name='throttle_interpolator',
            parameters=[vesc_config]
        ),

        Node(
            package='vesc_driver',
            executable='vesc_driver_node',
            name='vesc_driver',
            parameters=[vesc_config]
        ),

        Node(
            package='vesc_ackermann',
            executable='vesc_to_odom_node',
            name='vesc_to_odom',
            parameters=[vesc_config]
        )
    ])
