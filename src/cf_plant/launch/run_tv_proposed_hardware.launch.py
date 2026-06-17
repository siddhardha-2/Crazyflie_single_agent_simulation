import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Declare that this master launch file accepts a 'robot_uri' argument
    robot_uri_arg = DeclareLaunchArgument(
        'robot_uri',
        description='Crazyflie radio URI'
    )
    
    robot_uri = LaunchConfiguration('robot_uri')

    cf_plant_dir = get_package_share_directory('cf_plant')
    navigation_dir = get_package_share_directory('navigation')
    control_dir = get_package_share_directory('control')
    
    # 2. Explicitly pass the robot_uri down to the connect script
    connect_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(cf_plant_dir, 'launch', 'connect_731.launch.py')
        ),
        launch_arguments={'robot_uri': robot_uri}.items()
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
        executable='extremum_seeker_tv_proposed',
        name='extremum_seeker_tv_proposed',
        output='screen',
        parameters=[{
            'use_sim_time': False,
        }]
    )
    
    return LaunchDescription([
        robot_uri_arg,
        connect_launch,
        navigation_launch,
        control_launch,
        extremum_seeker,
    ])
