#!/usr/bin/python3
# actuation_status.py
from carla_msgs.msg._carla_ego_vehicle_status import CarlaEgoVehicleStatus
from tier4_vehicle_msgs.msg import ActuationStatusStamped
from rclpy.node import Node

class ActuationStatus(object):

    def __init__(self, node: Node):
        self.node = node
        self.control = CarlaEgoVehicleStatus()

        self._actuation_status_publisher = self.node.create_publisher(
            ActuationStatusStamped, '~/output/actuation_status', 1)

        self._vehicle_status_subscriber = self.node.create_subscription(
            CarlaEgoVehicleStatus, '~/input/status',
            self.vehicle_status_callback, 1)

    def vehicle_status_callback(self, msg: CarlaEgoVehicleStatus):
        self.control = msg

    def update(self):
        msg = ActuationStatusStamped()
        msg.header.frame_id = "base_link"
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.status.accel_status = self.control.control.throttle
        msg.status.brake_status = self.control.control.brake
        msg.status.steer_status = self.control.control.steer

        self._actuation_status_publisher.publish(msg)

    def destroy(self):
        self.node.get_logger().info("Destroying ActuationStatus publisher")
        self._actuation_status_publisher.destroy()
        self._vehicle_status_subscriber.destroy()