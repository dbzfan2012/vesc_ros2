import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):
    racecar_version = LaunchConfiguration('racecar_version').perform(context)
    car_name = LaunchConfiguration('car_name').perform(context)

    vesc_config_default = os.path.join(
        get_package_share_directory('vesc_main'),
        'config', racecar_version, 'vesc.yaml'
    )
    vesc_config = LaunchConfiguration('vesc_config').perform(context)
    if not vesc_config:
        vesc_config = vesc_config_default

    nodes = []

    # ackermann_to_vesc node
    nodes.append(Node(
        package='vesc_ackermann',
        executable='ackermann_to_vesc_node',
        name='ackermann_to_vesc',
        parameters=[vesc_config],
        remappings=[
            ('ackermann_cmd', '/' + car_name + '/mux/output'),
            ('commands/motor/speed', 'commands/motor/unsmoothed_speed'),
            ('commands/servo/position', 'commands/servo/unsmoothed_position'),
        ],
    ))

    # throttle_interpolator node
    # car_name must be empty because the node is already in the correct namespace
    # (pushed by teleop.launch.py). Passing car_name would double the prefix.
    nodes.append(Node(
        package='vesc_driver',
        executable='throttle_interpolator',
        name='throttle_interpolator',
        parameters=[
            vesc_config,
            {'car_name': ''},
        ],
    ))

    # vesc_driver_node
    nodes.append(Node(
        package='vesc_driver',
        executable='vesc_driver_node',
        name='vesc_driver',
        parameters=[vesc_config],
        # respawn=True,  # uncomment if you want auto-restart
    ))

    # vesc_to_odom node
    nodes.append(Node(
        package='vesc_ackermann',
        executable='vesc_to_odom_node',
        name='vesc_to_odom',
        parameters=[vesc_config],
    ))

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'racecar_version',
            description='Racecar hardware version (e.g., racecar-uw-nano, racecar-mit, racecar-uw-tx2)',
        ),
        DeclareLaunchArgument(
            'vesc_config',
            default_value='',
            description='Path to VESC config YAML file. Defaults to vesc_main/config/<racecar_version>/vesc.yaml',
        ),
        DeclareLaunchArgument(
            'car_name',
            default_value='car',
            description='Car name for topic namespacing',
        ),
        OpaqueFunction(function=launch_setup),
    ])
