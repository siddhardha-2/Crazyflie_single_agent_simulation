import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

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
            {'use_sim_time': True}
        ]
    )

    return LaunchDescription([
        controller_node,
    ])