import launch
import launch_ros.actions


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch_ros.actions.Node(
            package='autoware_carla_bridge',
            executable='carla_service.py',
            on_exit=launch.actions.Shutdown(),
            parameters=[
                {
                    'use_sim_time': True
                },
            ],
            remappings=[
                ('~/input/vehicle_status', '/carla/ego_vehicle/vehicle_status'),
                ('~/output/control', '/carla/ego_vehicle/vehicle_control_cmd'),
            ],
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()