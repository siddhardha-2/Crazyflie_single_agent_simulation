import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    package_dir = get_package_share_directory('cf_plant')
    config_file = os.path.join(package_dir, 'config', 'crazyflies.yaml')

    with open(config_file, 'r') as ymlfile:
        full_yaml = yaml.safe_load(ymlfile)

    # Filter out disabled drones so the radio doesn't waste bandwidth
    active_robots = {}
    for robot_name, robot_params in full_yaml['robots'].items():
        if robot_params.get('enabled', False):
            active_robots[robot_name] = robot_params
            
    full_yaml['robots'] = active_robots

    # Inject the native C++ backend
    full_yaml['backend'] = 'cpp'

    crazyflie_backend = Node(
        package='crazyflie',
        executable='crazyflie_server',
        name='crazyflie_server',
        output='screen',
        parameters=[full_yaml]
    )

    hardware_bridge = Node(
        package='cf_plant',
        executable='hardware_bridge_node',
        name='cf_hardware_bridge',
        output='screen',
        parameters=[{
            'robot_name': 'cf0',
            'use_sim_time': False,
            'auto_arm': True,
            'auto_takeoff': True,
            'takeoff_duration': 3.0,
            'command_rate': 50.0,
        }]
    )

    return LaunchDescription([
        crazyflie_backend,
        hardware_bridge,
    ])
