#!/usr/bin/python3
# control_command.py
from tier4_vehicle_msgs.msg import ActuationCommandStamped
from rclpy.node import Node
from carla_msgs.msg import CarlaEgoVehicleControl
from carla_msgs.msg import CarlaEgoVehicleInfo
import numpy as np
from nav_msgs.msg import Odometry
import csv

class ControlCommand(object):

    def __init__(self, node: Node):
        self.node = node
        self.timestamp = None
        self.tau = 0.2
        self.prev_timestamp = None
        self.prev_steer_output = 0.0
        self.in_cmd = None  # Initialize to None
        self.steer_curve = None
        self.current_vel = None
        
        # Load steering map from CSV
        # try:
        #     with open(steer_map_path, newline='') as csvfile:
        #         csv_reader = csv.reader(csvfile)
        #         self._tire_angle = np.float32(next(csv_reader))
        #         self._steer_cmd = np.float32(next(csv_reader))
        #         self.node.get_logger().info(f"Loaded steering map from {steer_map_path}")
        # except Exception as e:
        #     self.node.get_logger().error(f"Failed to load steering map: {e}")

            
        self._control_command_subscriber = self.node.create_subscription(
            ActuationCommandStamped, '~/input/actuation',
            self.control_callback, 1)
        
        self.odom = self.node.create_subscription(
            Odometry, '~/input/odometry',
            self.vel_callback, 1)

        self._vehicle_control_command_publisher = self.node.create_publisher(
            CarlaEgoVehicleControl, '~/output/control', 1)
        
    def vel_callback(self, msg: Odometry):
        """Callback for Odometry messages."""
        self.current_vel = msg.twist.twist.linear
        
    def steering_curve_callback(self, msg: CarlaEgoVehicleInfo):
        """Callback for CarlaEgoVehicleInfo messages."""
        self.steer_curve = msg.steering_curve

        
    def first_order_steering(self, steer_input):
        """First order steering model."""
        steer_output = 0.0
        if self.prev_timestamp is None:
            self.prev_timestamp = self.timestamp

        dt = self.timestamp - self.prev_timestamp
        if dt > 0.0:
            steer_output = self.prev_steer_output + (steer_input - self.prev_steer_output) * (
                dt / (self.tau + dt)
            )
        self.prev_steer_output = steer_output
        self.prev_timestamp = self.timestamp
        return steer_output

    def control_callback(self, msg: ActuationCommandStamped):
        """Callback for ActuationCommand messages."""
        self.in_cmd = msg
        
    def update(self):
        """Convert and publish CARLA Ego Vehicle Control to AUTOWARE."""
        # Check if we've received a command yet
        if not hasattr(self, 'in_cmd') or self.in_cmd is None:
            self.node.get_logger().debug("No control command received yet, skipping update")
            return
            
        now = self.node.get_clock().now().to_msg()
        self.timestamp = now.sec + now.nanosec * 1e-9
        throttle = self.in_cmd.actuation.accel_cmd
        
        # Check if we have steering curve and velocity data
        if not hasattr(self, 'steer_curve') or not self.steer_curve or not hasattr(self, 'current_vel'):
            self.node.get_logger().debug("Missing steering curve or velocity data, using default steering")
            steer = -self.in_cmd.actuation.steer_cmd  # Default conversion
        else:
            try:
                max_steer_ratio = np.interp(
                    abs(self.current_vel.x), [v.x for v in self.steer_curve], [v.y for v in self.steer_curve]
                )
                steer = self.first_order_steering(-self.in_cmd.actuation.steer_cmd) * max_steer_ratio
            except Exception as e:
                self.node.get_logger().error(f"Error calculating steering: {e}")
                steer = -self.in_cmd.actuation.steer_cmd  # Fallback
        
        brake = self.in_cmd.actuation.brake_cmd
        manual_gear_shift = False
        
        msg = CarlaEgoVehicleControl()
        msg.header.stamp = now
        msg.throttle = throttle
        msg.brake = brake
        msg.manual_gear_shift = manual_gear_shift
        msg.steer = steer
        self._vehicle_control_command_publisher.publish(msg)
    
    # def _convert_from_tire_to_steer(self, tire_angle) -> float:
    #     nearest_idx = (np.abs(self._tire_angle - tire_angle)).argmin()
    #     return float(self._steer_cmd[nearest_idx])

    # def update(self):
    #     # Skip if no command has been received yet
    #     if self.in_cmd is None:
    #         return
            
    #     steer = self._convert_from_tire_to_steer(self.in_cmd.actuation.steer_cmd)
    #     output_control_command = CarlaEgoVehicleControl()
    #     output_control_command.header.stamp = self.node.get_clock().now().to_msg()
    #     output_control_command.throttle = self.in_cmd.actuation.accel_cmd
    #     output_control_command.brake = self.in_cmd.actuation.brake_cmd
    #     output_control_command.manual_gear_shift = False
    #     output_control_command.steer = steer
    #     self._vehicle_control_command_publisher.publish(output_control_command)

        

    def destroy(self):
        self.node.get_logger().info("Destroying ControlCommand subscriber")
        self._vehicle_control_command_publisher.destroy()
        self._control_command_subscriber.destroy()