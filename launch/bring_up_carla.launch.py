import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg = "autoware_carla_bridge"


    carla_bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory(pkg),
                'launch','bring_up_carla_ros.launch.py'
            )
        ),
        launch_arguments={
            'use_sim_time': 'True',
            'host': 'localhost',
            'port': '2000',
            'timeout': '10.0',
            'synchronous_mode': 'True',
            'town': 'Town01',
        }.items()           
    )
    
    spawn_entity = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('autoware_carla_bridge'),
                'launch', 'carla_spawn_vehicle.launch.py'
            )
        )
    )

    manual_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(
                    'carla_manual_control'), 'carla_manual_control.launch.py')
            ),
            launch_arguments={
                'role_name': 'ego_vehicle',
            }.items()
    )



    ld = LaunchDescription()
    ld.add_action(carla_bridge)
    ld.add_action(spawn_entity)
    ld.add_action(manual_control)
    
    return ld