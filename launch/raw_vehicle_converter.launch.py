from ament_index_python.packages import get_package_share_directory
import launch
import launch_ros.actions
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterFile

def generate_launch_description():
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='config_file',
            default_value=get_package_share_directory(
                'autoware_carla_bridge') + '/config/raw_vehicle_cmd_converter.param.yaml'
        ),
        launch_ros.actions.Node(
            package='autoware_raw_vehicle_cmd_converter',
            executable='autoware_raw_vehicle_cmd_converter_node',
            name='autoware_raw_vehicle_cmd_converter',
            on_exit=launch.actions.Shutdown(),
            parameters=[
                {'use_sim_time': True},
                ParameterFile(LaunchConfiguration('config_file'), allow_substs=True),
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