import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    package_dir = get_package_share_directory('cf_plant')
    generic_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_dir, 'launch', 'hardware.launch.py')
        ),
        launch_arguments={
            'robot_uri': LaunchConfiguration('robot_uri'),
            'robot_name': LaunchConfiguration('robot_name'),
            'backend': LaunchConfiguration('backend'),
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_uri',
            description=(
                'Crazyflie radio URI (required; no URI is stored in Git)'
            ),
        ),
        DeclareLaunchArgument('robot_name', default_value='cf0'),
        DeclareLaunchArgument('backend', default_value='cpp'),
        generic_launch,
    ])
