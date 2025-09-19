#!/usr/bin/python3

import rclpy
from launch_ros.actions import Node
from rclpy.node import Node
from autoware_carla_bridge.actuation_status import ActuationStatus
from autoware_carla_bridge.control_command import ControlCommand
from autoware_carla_bridge.control_mode import ControlMode
from autoware_carla_bridge.gear_mode import GearMode
from autoware_carla_bridge.steering_status import SteeringStatus
from autoware_carla_bridge.velocity_state_report import VelocityStateReport
from autoware_carla_bridge.gnss_pose_cov import GnssCov
from autoware_carla_bridge.cam_front import CamFront
from autoware_carla_bridge.cam_view import CamView
from autoware_carla_bridge.lidar import Lidar


class AutowareCarlaBridge(Node):
    def __init__(self):
        super().__init__('autoware_carla_bridge')
        self.get_logger().info("Autoware Carla Bridge Node has been started")

        self.actuation_status = ActuationStatus(self)
        self.control_command = ControlCommand(self)
        self.control_mode = ControlMode(self)
        self.gear_mode = GearMode(self)
        self.steering_status = SteeringStatus(self)
        self.velocity_state_report = VelocityStateReport(self)
        self.gnss_cov = GnssCov(self)
        self.cam = CamFront(self)
        self.cam_view = CamView(self)
        # self.lidar = Lidar(self)
        hz = 0.05  # 20 Hz
        self.create_timer(hz, self.timer_callback)

    def timer_callback(self):
        self.actuation_status.update()
        self.control_command.update()
        self.control_mode.update()
        self.gear_mode.update()
        self.steering_status.update()
        self.velocity_state_report.update()
        self.gnss_cov.update()
        self.cam.update()
        self.cam_view.update()
        # self.lidar.update()
        
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