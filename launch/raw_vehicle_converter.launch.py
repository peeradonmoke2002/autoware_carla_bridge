from ament_index_python.packages import get_package_share_directory
import launch
import launch_ros.actions


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='converter_param_path',
            default_value=get_package_share_directory(
                'autoware_carla_bridge') + '/config/raw_vehicle_cmd_converter.param.yaml'
        ),
        launch.actions.DeclareLaunchArgument(
            name='csv_path_accel_map',
            default_value=get_package_share_directory(
                'autoware_carla_bridge') + '/data/carla_tesla_model3/accel_map.csv'
        ),
        launch.actions.DeclareLaunchArgument(
            name='csv_path_brake_map',
            default_value=get_package_share_directory(
                'autoware_carla_bridge') + '/data/carla_tesla_model3/brake_map.csv'
        ),
        launch.actions.DeclareLaunchArgument(
            name='csv_path_steer_map',
            default_value=get_package_share_directory(
                'autoware_carla_bridge') + '/data/carla_tesla_model3/steer_map.csv'
        ),
        launch_ros.actions.Node(
                package='autoware_raw_vehicle_cmd_converter',
                executable='autoware_raw_vehicle_cmd_converter_node',
                name='autoware_raw_vehicle_cmd_converter',
                on_exit=launch.actions.Shutdown(),
                parameters=[
                    launch.substitutions.LaunchConfiguration('converter_param_path'),
                    {
                        'use_sim_time': True,
                        'csv_path_accel_map': launch.substitutions.LaunchConfiguration('csv_path_accel_map'),
                        'csv_path_brake_map': launch.substitutions.LaunchConfiguration('csv_path_brake_map'),
                        # 'csv_path_steer_map': launch.substitutions.LaunchConfiguration('csv_path_steer_map')
                    }
                ],
            remappings=[
                ('~/input/control_cmd', '/control/command/control_cmd'),
                ('~/input/odometry', '/localization/kinematic_state'),
                ('~/input/steering', '/vehicle/status/steering_status'),
                ('~/output/actuation_cmd', '/control/command/actuation_cmd'),
            ],
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()