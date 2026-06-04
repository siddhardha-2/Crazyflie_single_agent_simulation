import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    cf_plant_dir = get_package_share_directory('cf_plant')
    navigation_dir = get_package_share_directory('navigation')
    control_dir = get_package_share_directory('control')

    connect_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(cf_plant_dir, 'launch', 'connect_hardware.launch.py')
        ),
        launch_arguments={
            'robot_uri': LaunchConfiguration('robot_uri'),
            'robot_name': LaunchConfiguration('robot_name'),
        }.items(),
    )

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(navigation_dir, 'launch', 'navigation.launch.py')
        ),
        launch_arguments={'use_sim_time': 'false'}.items(),
    )

    control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(control_dir, 'launch', 'control.launch.py')
        ),
        launch_arguments={'use_sim_time': 'false'}.items(),
    )

    guidance_node = Node(
        package='Guidance',
        executable=LaunchConfiguration('guidance_executable'),
        output='screen',
        parameters=[{'use_sim_time': False}],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_uri',
            description=(
                'Crazyflie radio URI, for example '
                'radio://0/80/2M/E7E7E7E731'
            ),
        ),
        DeclareLaunchArgument('robot_name', default_value='cf0'),
        DeclareLaunchArgument(
            'guidance_executable',
            default_value='extremum_seeker_hardware',
        ),
        connect_launch,
        navigation_launch,
        control_launch,
        guidance_node,
    ])
