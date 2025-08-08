#!/usr/bin/python3

import rclpy
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from autoware_carla_bridge.actuation_status import ActuationStatus
from autoware_carla_bridge.control_command import ControlCommand
from autoware_carla_bridge.control_mode import ControlMode
from autoware_carla_bridge.gear_mode import GearMode
from autoware_carla_bridge.lidar import LidarExtended
from autoware_carla_bridge.steering_status import SteeringStatus
from autoware_carla_bridge.velocity_state_report import VelocityStateReport

class AutowareCarlaBridge(Node):
    def __init__(self):
        super().__init__('autoware_carla_bridge')
        self.get_logger().info("Autoware Carla Bridge Node has been started")
        package_path = get_package_share_directory('autoware_carla_bridge')
        self.declare_parameter('csv_path_steer_map',
                               package_path + '/data/carla_tesla_model3/steer_map.csv')
        csv_path_steer_map = self.get_parameter(
            'csv_path_steer_map').get_parameter_value().string_value
        self.lidar_extended = LidarExtended(self)
        self.actuation_status = ActuationStatus(self)
        self.control_command = ControlCommand(self, csv_path_steer_map)
        self.control_mode = ControlMode(self)
        self.gear_mode = GearMode(self)
        self.steering_status = SteeringStatus(self)
        self.velocity_state_report = VelocityStateReport(self)
        hz = 1/500 # 500 hz
        self.create_timer(hz, self.timer_callback) 
        
    def timer_callback(self):
        self.lidar_extended.update()
        self.actuation_status.update()
        self.control_command.update()
        self.control_mode.update()
        self.gear_mode.update()
        self.steering_status.update()
        self.velocity_state_report.update()

    def destroy_node(self):
        self.get_logger().info("Destroying AutowareCarlaBridge node")
        super().destroy_node()
        
def main(args=None):
    rclpy.init(args=args)
    node = AutowareCarlaBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()