import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource


def generate_launch_description():

    carla_spawn_service = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(
                'autoware_carla_bridge'), 'launch/carla_service.launch.py')
        )
    )

    vehicle = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            os.path.join(get_package_share_directory(
                'autoware_carla_bridge'), 'launch/e2e_simulator.launch.xml')
        )
    )

    raw_vehicle = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(
                'autoware_carla_bridge'), 'launch/raw_vehicle_converter.launch.py')
        )
    )

    ld = LaunchDescription()
    ld.add_action(carla_spawn_service)
    ld.add_action(vehicle)
    ld.add_action(raw_vehicle)
    return ld