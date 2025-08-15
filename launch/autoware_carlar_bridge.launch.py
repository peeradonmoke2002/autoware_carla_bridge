from ament_index_python.packages import get_package_share_directory
import os
import launch
import launch_ros.actions


def generate_launch_description():
    ld = launch.LaunchDescription([

        launch_ros.actions.Node(
            package='autoware_carla_bridge',
            executable='bridge.py',
            name='bridge',
            on_exit=launch.actions.Shutdown(),
            parameters=[
                {
                    'use_sim_time': True
                }
            ],
            remappings=[
                ('~/input/odometry', '/carla/ego_vehicle/odometry'),
                ('~/input/status', '/carla/ego_vehicle/vehicle_status'),
                ('~/input/steering', '/carla/ego_vehicle/vehicle_steering'),
                ('~/input/actuation', '/control/command/actuation_cmd'),
                ('~/input/lidar', '/sensing/lidar/top/pointcloud_raw'),
                ('~/input/image', '/sensing/camera/traffic_light/image_raw'),
                ('~/input/camera_info', '/sensing/camera/traffic_light/camera_info'),
                ('~/output/velocity_status', '/vehicle/status/velocity_status'),
                ('~/output/steering_status', '/vehicle/status/steering_status'),
                ('~/output/actuation_status', '/vehicle/status/actuation_status'),
                ('~/output/control', '/carla/ego_vehicle/vehicle_control_cmd'),
                ('~/output/lidar_ex', '/sensing/lidar/top/pointcloud_raw'),
                ('~/input/gnss', '/sensing/gnss/ublox/nav_sat_fix'),
                ('~/output/gnss_cov', '/sensing/gnss/pose_with_covariance'),
                ('~/output/odometry', '/carla/ego_vehicle/odometry'),
                ('~/output/image', '/sensing/camera/traffic_light/image_raw'),
                ('~/output/camera_info', '/sensing/camera/traffic_light/camera_info'),

            ],
        ),
        
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()