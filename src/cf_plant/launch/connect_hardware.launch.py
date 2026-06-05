import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context):
    package_dir = get_package_share_directory('cf_plant')
    config_file = os.path.join(package_dir, 'config', 'crazyflies.yaml')
    robot_name = LaunchConfiguration('robot_name').perform(context)
    robot_uri = LaunchConfiguration('robot_uri').perform(context)

    with open(config_file, 'r', encoding='utf-8') as yaml_file:
        full_yaml = yaml.safe_load(yaml_file)

    if robot_name not in full_yaml['robots']:
        raise RuntimeError(
            f'Robot "{robot_name}" is not defined in {config_file}'
        )

    robot_config = full_yaml['robots'][robot_name]
    robot_config['enabled'] = True
    robot_config['uri'] = robot_uri
    full_yaml['robots'] = {robot_name: robot_config}
    full_yaml['backend'] = 'cpp'

    return [
        Node(
            package='crazyflie',
            executable='crazyflie_server',
            name='crazyflie_server',
            output='screen',
            parameters=[full_yaml],
        ),
        Node(
            package='cf_plant',
            executable='hardware_bridge_node',
            name='cf_hardware_bridge',
            output='screen',
            parameters=[{
                'robot_name': robot_name,
                'use_sim_time': False,
                'auto_arm': LaunchConfiguration('auto_arm'),
                'auto_takeoff': LaunchConfiguration('auto_takeoff'),
                'takeoff_duration': LaunchConfiguration('takeoff_duration'),
                'command_rate': LaunchConfiguration('command_rate'),
            }],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_uri',
            description=(
                'Crazyflie radio URI, for example '
                'radio://0/80/2M/E7E7E7E731'
            ),
        ),
        DeclareLaunchArgument('robot_name', default_value='cf0'),
        DeclareLaunchArgument('auto_arm', default_value='true'),
        DeclareLaunchArgument('auto_takeoff', default_value='true'),
        DeclareLaunchArgument('takeoff_duration', default_value='3.0'),
        DeclareLaunchArgument('command_rate', default_value='50.0'),
        OpaqueFunction(function=launch_setup),
    ])
