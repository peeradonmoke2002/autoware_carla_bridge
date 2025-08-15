import launch
import launch_ros.actions


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='host',
            default_value='10.61.2.24',
            description='IP of the CARLA server'
        ),
        launch.actions.DeclareLaunchArgument(
            name='port',
            default_value='2000',
            description='TCP port of the CARLA server'
        ),
        launch.actions.DeclareLaunchArgument(
            name='timeout',
            default_value='20.0',
            description='Time to wait for a successful connection to the CARLA server'
        ),
        launch.actions.DeclareLaunchArgument(
            name='passive',
            default_value='False',
            description='When enabled, the ROS bridge will take a backseat and another client must tick the world (only in synchronous mode)'
        ),
        launch.actions.DeclareLaunchArgument(
            name='synchronous_mode',
            default_value='True',
            description='Enable/disable synchronous mode. If enabled, the ROS bridge waits until the expected data is received for all sensors'
        ),
        launch.actions.DeclareLaunchArgument(
            name='synchronous_mode_wait_for_vehicle_control_command',
            default_value='False',
            description='When enabled, pauses the tick until a vehicle control is completed (only in synchronous mode)'
        ),
        launch.actions.DeclareLaunchArgument(
            name='fixed_delta_seconds',
            default_value='0.05',
            description='Simulation time (delta seconds) between simulation steps'
        ),
        launch.actions.DeclareLaunchArgument(
            name='town',
            default_value='Town01',
            description='Either use an available CARLA town (eg. "Town01") or an OpenDRIVE file (ending in .xodr)'
        ),
        launch.actions.DeclareLaunchArgument(
            name='register_all_sensors',
            default_value='True',
            description='Enable/disable the registration of all sensors. If disabled, only sensors spawned by the bridge are registered'
        ),
        launch.actions.DeclareLaunchArgument(
            name='ego_vehicle_role_name',
            default_value=["hero", "ego_vehicle", "hero0", "hero1", "hero2",
                           "hero3", "hero4", "hero5", "hero6", "hero7", "hero8", "hero9"],
            description='Role names to identify ego vehicles. '
        ),
        launch_ros.actions.Node(
            package='carla_ros_bridge',
            executable='bridge',
            name='carla_ros_bridge',
            output='screen',
            emulate_tty='True',
            on_exit=launch.actions.Shutdown(),
            parameters=[
                {
                    'use_sim_time': True
                },
                {
                    'host': launch.substitutions.LaunchConfiguration('host')
                },
                {
                    'port': launch.substitutions.LaunchConfiguration('port')
                },
                {
                    'timeout': launch.substitutions.LaunchConfiguration('timeout')
                },
                {
                    'passive': launch.substitutions.LaunchConfiguration('passive')
                },
                {
                    'synchronous_mode': launch.substitutions.LaunchConfiguration('synchronous_mode')
                },
                {
                    'synchronous_mode_wait_for_vehicle_control_command': launch.substitutions.LaunchConfiguration('synchronous_mode_wait_for_vehicle_control_command')
                },
                {
                    'fixed_delta_seconds': launch.substitutions.LaunchConfiguration('fixed_delta_seconds')
                },
                {
                    'town': launch.substitutions.LaunchConfiguration('town')
                },
                {
                    'register_all_sensors': launch.substitutions.LaunchConfiguration('register_all_sensors')
                },
                {
                    'ego_vehicle_role_name': launch.substitutions.LaunchConfiguration('ego_vehicle_role_name')
                }
            ],
            remappings=[
                ('/carla/ego_vehicle/rgb_front/camera_info',
                 '/sensing/camera/traffic_light/camera_info'),
                ('/carla/ego_vehicle/rgb_front/image', '/sensing/camera/traffic_light/image_raw'),
                ('/carla/ego_vehicle/gnss', '/sensing/gnss/ublox/nav_sat_fix'),
                ('/carla/ego_vehicle/imu', '/sensing/imu/tamagawa/imu_raw'),
                ('/carla/ego_vehicle/lidar', '/sensing/lidar/top/pointcloud_raw'),
                ('/carla/ego_vehicle/gnss','/sensing/gnss/pose_with_covariance')
            ],
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
