import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    # PID config file
    pid_config = os.path.join(
        get_package_share_directory('control'),
        'config',
        'pid.yaml'
    )

    controller_node = Node(
        package='control',
        executable='controller_node',
        name='controller_node',
        output='screen',
        parameters=[
            pid_config,
            {'use_sim_time': use_sim_time}
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        controller_node,
    ])
