from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    state_estimator_node = Node(
        package='navigation',
        executable='state_estimator',
        name='state_estimator',
        output='screen',
        parameters=[{
            'use_sim_time': True,
        }]
    )

    return LaunchDescription([
        state_estimator_node,
    ])