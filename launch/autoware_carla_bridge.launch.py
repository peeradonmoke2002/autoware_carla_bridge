import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, Shutdown
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg = "autoware_carla_bridge"

    autoware_carlar_bridge = Node(
        package= pkg,
        executable='bridge.py',
        name='bridge',
        output='screen',
        on_exit=Shutdown(),
        parameters=[{'use_sim_time': True}],
        remappings=[
                ('~/input/gear_status', '/carla/ego_vehicle/vehicle_status'),
                ('~/input/odometry', '/carla/ego_vehicle/odometry'),
                ('~/input/status', '/carla/ego_vehicle/vehicle_status'),
                ('~/input/steering', '/carla/ego_vehicle/vehicle_steering'),
                ('~/input/actuation', '/control/command/actuation_cmd'),
                ('~/input/image', '/carla/ego_vehicle/rgb_front/image'),
                ('~/input/camera_info', '/carla/ego_vehicle/rgb_front/camera_info'),
                ('~/input/image_view', '/carla/ego_vehicle/rgb_view/image'),
                ('~/input/camera_info_view', '/carla/ego_vehicle/rgb_view/camera_info'),
                ('~/input/gnss_cov', '/carla/ego_vehicle/odometry'),
                ('~/output/gear_status', '/vehicle/status/gear_status'),
                ('~/output/velocity_status', '/vehicle/status/velocity_status'),
                ('~/output/steering_status', '/vehicle/status/steering_status'),
                ('~/output/actuation_status', '/vehicle/status/actuation_status'),
                ('~/output/control', '/carla/ego_vehicle/vehicle_control_cmd'),
                ('~/output/gnss_cov', '/sensing/gnss/pose_with_covariance'),
                ('~/output/image', '/sensing/camera/traffic_light/image_raw'),
                ('~/output/camera_info', '/sensing/camera/traffic_light/camera_info'),
                ('~/output/image_view', '/sensing/camera/view/rgb_view'),
                ('~/output/camera_info_view', '/sensing/camera/view/camera_info_view'),
        ],
    )

    autoware_vehicle = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory(pkg),
                'launch', 'autoware_vehicle.launch.py'
            )
        )
    )

    ld = LaunchDescription()
    ld.add_action(autoware_carlar_bridge)
    ld.add_action(autoware_vehicle)
    return ld













