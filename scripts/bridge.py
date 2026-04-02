#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from autoware_vehicle_msgs.msg import Engage
from autoware_carla_bridge.actuation_status import ActuationStatus
from autoware_carla_bridge.control_command import ControlCommand
from autoware_carla_bridge.control_mode import ControlMode
from autoware_carla_bridge.gear_mode import GearMode
from autoware_carla_bridge.steering_status import SteeringStatus
from autoware_carla_bridge.velocity_state_report import VelocityStateReport
from autoware_carla_bridge.gnss_pose_cov import GnssCov
from autoware_carla_bridge.cam_front import CamFront
from autoware_carla_bridge.cam_view import CamView
from autoware_carla_bridge.lidar_ex import LidarExtended
# from autoware_carla_bridge.lidar import Lidar
from autoware_carla_bridge.imu import Imu
from autoware_carla_bridge.gt_detection import GroundTruthDetection


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
        # self.cam_camera0 = CamCamera0(self)
        self.cam_view = CamView(self)
        self.lidar_ex = LidarExtended(self)
        # self.lidar = Lidar(self)
        self.imu = Imu(self)
        self.gt_detection = GroundTruthDetection(self)
        # self._engage_pub = self.create_publisher(Engage, '/autoware/engage', 1)
        hz = 0.0333  # 30 Hz
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
        # self.cam_camera0.update()
        self.cam_view.update()
        self.lidar_ex.update()
        self.imu.update()
        self.gt_detection.update()
        # Only engage once CARLA is actually sending data (GNSS stamp non-zero)
        # if self.gnss_cov.input_gnss.header.stamp.sec > 0:
        #     engage_msg = Engage()
        #     engage_msg.engage = True
        #     self._engage_pub.publish(engage_msg)
        
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