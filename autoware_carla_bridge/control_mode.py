#!/usr/bin/python3
# control_mode.py
from autoware_vehicle_msgs.msg import ControlModeReport
from rclpy.node import Node

class ControlMode(object):

    def __init__(self, node: Node):
        self.node = node
        self.pub_ctrl_mode = self.node.create_publisher(
            ControlModeReport, "/vehicle/status/control_mode", 1
        )

    def update(self):
        out_ctrl_mode = ControlModeReport()
        out_ctrl_mode.stamp = self.node.get_clock().now().to_msg()
        out_ctrl_mode.mode =  ControlModeReport.AUTONOMOUS
        self.pub_ctrl_mode.publish(out_ctrl_mode)

    def destroy(self):
        self.node.get_logger().info("Destroying ControlMode")
        self.pub_ctrl_mode.destroy()