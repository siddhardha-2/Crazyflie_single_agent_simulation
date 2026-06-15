import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    cf_plant_dir   = get_package_share_directory('cf_plant')
    navigation_dir = get_package_share_directory('navigation')
    control_dir    = get_package_share_directory('control')
    guidance_dir   = get_package_share_directory('Guidance')

    # Path to the guidance config file — passed to every guidance node so
    # ROS parameters declared in guidance.yaml are actually loaded.
    guidance_config = os.path.join(guidance_dir, 'config', 'guidance.yaml')

    connect_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(cf_plant_dir, 'launch', 'connect_hardware.launch.py')
        ),
        launch_arguments={
            'robot_uri':   LaunchConfiguration('robot_uri'),
            'robot_name':  LaunchConfiguration('robot_name'),
            'backend':     LaunchConfiguration('backend'),
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
        parameters=[
            guidance_config,            # loads guidance.yaml for this node
            {'use_sim_time': False},    # override sim time last
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_uri',
            description=(
                'Crazyflie radio URI, e.g. radio://0/80/2M/E7E7E7E735'
            ),
        ),
        DeclareLaunchArgument('robot_name', default_value='cf0'),
        DeclareLaunchArgument('backend',    default_value='cpp'),
        DeclareLaunchArgument(
            'guidance_executable',
            default_value='extremum_seeker_tv_classical',
            description=(
                'Guidance node to launch. Choices: '
                'pattern_node | extremum_seeker | extremum_seeker_hardware | '
                'extremum_seeker_tv_hardware | extremum_seeker_tv_classical'
            ),
        ),
        connect_launch,
        navigation_launch,
        control_launch,
        guidance_node,
    ])
