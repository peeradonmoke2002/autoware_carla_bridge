#!/usr/bin/python3
from rclpy.node import Node
from carla_msgs.msg import CarlaEgoVehicleStatus
from autoware_vehicle_msgs.msg import GearReport 

class GearMode(object):
    def __init__(self, node: Node):
        self.node = node
        self.vehicle_status = CarlaEgoVehicleStatus()
        self.vehicle_status_sub = self.node.create_subscription(
            CarlaEgoVehicleStatus, '~/input/gear_status',
            self.vehicle_status_cb, 1)
        self.gear_pub = self.node.create_publisher(
            GearReport, '~/output/gear_status', 1)
    
    def vehicle_status_cb(self, msg: CarlaEgoVehicleStatus):
        self.vehicle_status = msg

    def update(self):
        aw = GearReport()
        aw.stamp = self.node.get_clock().now().to_msg()
        reverse = bool(self.vehicle_status.control.reverse)
        gear = int(self.vehicle_status.control.gear)

        if reverse or gear < 0:
            aw.report = GearReport.REVERSE
        elif gear == 0:
            aw.report = GearReport.NEUTRAL
        else:
            aw.report = GearReport.DRIVE

        self.gear_pub.publish(aw)

    def destroy_node(self):
            self.node.get_logger().info("Destroying GearMode node")
            self.vehicle_status_sub.destroy()
            self.gear_pub.destroy()
