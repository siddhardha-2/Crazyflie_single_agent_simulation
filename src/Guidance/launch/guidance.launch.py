import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # Guidance config file
    guidance_config = os.path.join(
        get_package_share_directory('Guidance'),
        'config',
        'guidance.yaml'
    )

    pattern_node = Node(
        package='Guidance',
        executable='pattern_node',
        name='pattern_node',
        output='screen',
        parameters=[
            guidance_config,
            {'use_sim_time': True}
        ]
    )

    return LaunchDescription([
        pattern_node,
    ])