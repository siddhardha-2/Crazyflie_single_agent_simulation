import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import SetEnvironmentVariable

from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():

    # Your package
    pkg_crazy_sim = get_package_share_directory('crazy_sim')

    # World file
    world_file = os.path.join(
        pkg_crazy_sim,
        'worlds',
        'single_crazy_world.sdf'
    )

    # Models folder
    models_path = os.path.join(
        pkg_crazy_sim,
        'models'
    )

    # Gazebo launch
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                FindPackageShare('ros_gz_sim').find('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            )
        ),
        launch_arguments={
            'gz_args': world_file + ' -r'
        }.items(),
    )
        # ROS <-> Gazebo bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': os.path.join(
                pkg_crazy_sim,
                'config',
                'ros_gz_bridge.yaml'
            ),
        }],
        output='screen',
    )

    return LaunchDescription([

        # Tell Gazebo where models are
        SetEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            models_path
        ),

        gz_sim,
        # ROS-GZ bridge
        bridge,
    ])