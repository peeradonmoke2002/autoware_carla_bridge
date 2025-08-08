#!/usr/bin/python3
# gear_mode.py
from autoware_vehicle_msgs.msg import GearReport
from rclpy.node import Node

class GearMode(object):

    def __init__(self, node: Node):
        self.node = node
        self.pub_gear_state = self.node.create_publisher(
            GearReport, "/vehicle/status/gear_status", 1
        )
        
    def update(self):
        out_gear_state = GearReport()
        out_gear_state.stamp = self.node.get_clock().now().to_msg()
        out_gear_state.report = GearReport.DRIVE
        self.pub_gear_state.publish(out_gear_state)

    def destroy(self):
        self.node.get_logger().info("Destroying GearMode")
        self.pub_gear_state.destroy()