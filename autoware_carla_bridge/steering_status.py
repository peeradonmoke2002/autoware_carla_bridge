#!/usr/bin/python3
# steering_status.py
from autoware_vehicle_msgs.msg import SteeringReport
from rclpy.node import Node
from carla_msgs.msg import CarlaEgoVehicleSteering

class SteeringStatus(object):

    def __init__(self, node: Node):
        self.node = node
        self.vehicle_steering_msg = CarlaEgoVehicleSteering()
        self.vehicle_steering_subscriber = self.node.create_subscription(
            CarlaEgoVehicleSteering, '~/input/steering',
            self.vehicle_steering_callback, 1)
        self.steering_status_publisher = self.node.create_publisher(
            SteeringReport, '~/output/steering_status', 1)

    def vehicle_steering_callback(self, msg: CarlaEgoVehicleSteering):
        """Callback for vehicle steering messages."""
        self.vehicle_steering_msg = msg


    def update(self):
        out_steering_state = SteeringReport()
        out_steering_state.stamp = self.node.get_clock().now().to_msg()
        out_steering_state.steering_tire_angle = -self.vehicle_steering_msg.steering_tire_angle
        self.steering_status_publisher.publish(out_steering_state)


    def destroy(self):
        self.node.get_logger().info("Destroying SteeringStatus")
        self.vehicle_steering_subscriber.destroy()
        self.steering_status_publisher.destroy()