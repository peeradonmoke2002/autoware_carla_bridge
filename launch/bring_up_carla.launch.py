import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg = "autoware_carla_bridge"

    host = DeclareLaunchArgument(
        'host',
        default_value='localhost'
    )

    port = DeclareLaunchArgument(
        'port',
        default_value='2000',
        description='CARLA port'
    )
    town = DeclareLaunchArgument(
        'town',
        default_value='Town01'
    )
    timeout = DeclareLaunchArgument(
        'timeout',
        default_value='10.0'
    )
    passive = DeclareLaunchArgument(
        'passive',
        default_value='False'
    )
    synchronous_mode = DeclareLaunchArgument(
        'synchronous_mode',
        default_value='True'
    )
    synchronous_mode_wait_for_vehicle_control_command = DeclareLaunchArgument(
        'synchronous_mode_wait_for_vehicle_control_command',
        default_value='False'
    )
    fixed_delta_seconds = DeclareLaunchArgument(
        'fixed_delta_seconds',
        default_value='0.05' # 20 FPS
    )
    register_all_sensors = DeclareLaunchArgument(
        'register_all_sensors',
        default_value='True'
    )
    ego_vehicle_role_name = DeclareLaunchArgument(
        'ego_vehicle_role_name',
        default_value=["hero", "ego_vehicle", "hero0", "hero1", "hero2",
                       "hero3", "hero4", "hero5", "hero6", "hero7", "hero8", "hero9"]
    )
    
    carla_bridge = Node(
        package='carla_ros_bridge',
        executable='bridge',
        name='carla_ros_bridge',
        output='screen',
        emulate_tty='True',
        on_exit= Shutdown(),
        parameters=[
            {
                'use_sim_time': True
            },
            {
                'host': LaunchConfiguration('host')
            },
            {
                'port': LaunchConfiguration('port')
            },
            {
                'timeout': LaunchConfiguration('timeout')
            },
            {
                'passive': LaunchConfiguration('passive')
            },
            {
                'synchronous_mode': LaunchConfiguration('synchronous_mode')
            },
            {
                'synchronous_mode_wait_for_vehicle_control_command': LaunchConfiguration('synchronous_mode_wait_for_vehicle_control_command')
            },
            {
                'fixed_delta_seconds': LaunchConfiguration('fixed_delta_seconds')
            },
            {
                'town': LaunchConfiguration('town')
            },
            {
                'register_all_sensors': LaunchConfiguration('register_all_sensors')
            },
            {
                'ego_vehicle_role_name': LaunchConfiguration('ego_vehicle_role_name')
            }
        ],
            remappings=[
                ('/carla/ego_vehicle/imu', '/sensing/imu/tamagawa/imu_raw'),
                ('/carla/ego_vehicle/gnss', '/sensing/gnss/ublox/nav_sat_fix'),
                ('/carla/ego_vehicle/lidar', '/sensing/lidar/top/pointcloud_before_sync')
                # ('/carla/ego_vehicle/lidar', '/sensing/lidar/top/pointcloud_raw')
            ],
    )
    
    lidar_transform = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='velodyne_top',
            output='screen',
            arguments=['0', '0', '1', '-1.5386', '-0.015', '0.001', 'velodyne_top', 'velodyne_top_changed']
    )

    imu_transform = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='imu',
            output='screen',
            arguments=['0', '0', '1', '-3.10519265', '-0.015', '-3.14059265359', 'tamagawa/imu_link', 'tamagawa/imu_link_changed']
    )
    

    spawn_entity = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory(pkg),
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
    ld.add_action(host)
    ld.add_action(port)
    ld.add_action(timeout)
    ld.add_action(passive)
    ld.add_action(synchronous_mode)
    ld.add_action(synchronous_mode_wait_for_vehicle_control_command)
    ld.add_action(fixed_delta_seconds)
    ld.add_action(town)
    ld.add_action(register_all_sensors)
    ld.add_action(ego_vehicle_role_name)
    ld.add_action(lidar_transform)
    ld.add_action(imu_transform)
    ld.add_action(carla_bridge)
    ld.add_action(spawn_entity)
    ld.add_action(manual_control)
    
    return ld