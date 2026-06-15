import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    guidance_config = os.path.join(
        get_package_share_directory('Guidance'),
        'config',
        'guidance.yaml'
    )

    # ---- existing pattern node ------------------------------------------ #
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

    # ---- TV extremum seeker (hardware) ------------------------------------ #
    tv_esc_hw_node = Node(
        package='Guidance',
        executable='extremum_seeker_tv_hardware',
        name='extremum_seeker_tv_hardware',
        output='screen',
        parameters=[guidance_config]
    )

    return LaunchDescription([
        pattern_node,
        tv_esc_hw_node,
    ])
