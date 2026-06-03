import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    cf_plant_dir = get_package_share_directory('cf_plant')
    navigation_dir = get_package_share_directory('navigation')
    control_dir = get_package_share_directory('control')

    connect_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(cf_plant_dir, 'launch', 'connect_731.launch.py')
        )
    )

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(navigation_dir, 'launch', 'navigation.launch.py')
        ),
        launch_arguments={'use_sim_time': 'false'}.items()
    )

    control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(control_dir, 'launch', 'control.launch.py')
        ),
        launch_arguments={'use_sim_time': 'false'}.items()
    )

    extremum_seeker = Node(
        package='Guidance',
        executable='extremum_seeker_hardware',
        name='extremum_seeker_hardware',
        output='screen',
        parameters=[{
            'use_sim_time': False,
        }]
    )

    return LaunchDescription([
        connect_launch,
        navigation_launch,
        control_launch,
        extremum_seeker,
    ])
