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
        self.in_cmd = None
        self.steer_curve = None        # expected: list of pairs (speed_mps, steer_ratio)
        self.current_vel = None        # geometry_msgs/Vector3 or None

        # Subscriptions
        self._control_command_subscriber = self.node.create_subscription(
            ActuationCommandStamped, '~/input/actuation',
            self.control_callback, 1
        )
        self._odom_sub = self.node.create_subscription(
            Odometry, '~/input/odometry',
            self.vel_callback, 1
        )
        self._veh_info_sub = self.node.create_subscription(
            CarlaEgoVehicleInfo, '~/input/vehicle_info',
            self.steering_curve_callback, 1
        )

        # Publisher
        self._vehicle_control_command_publisher = self.node.create_publisher(
            CarlaEgoVehicleControl, '~/output/control', 1
        )

    def vel_callback(self, msg: Odometry):
        self.current_vel = msg.twist.twist.linear
        
    def steering_curve_callback(self, msg: CarlaEgoVehicleInfo):
        """Callback for CarlaEgoVehicleInfo messages."""
        self.steer_curve = msg.steering_curve

        
    def first_order_steering(self, steer_input):
        """First order steering model with deadband."""
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
        if not hasattr(self, 'in_cmd') or self.in_cmd is None:
            return
            
        now = self.node.get_clock().now().to_msg()
        self.timestamp = now.sec + now.nanosec * 1e-9
        throttle = self.in_cmd.actuation.accel_cmd
        
        if hasattr(self, 'current_vel') and abs(self.current_vel.x) < 0.1:  # < 0.1 m/s
            steer = 0.0
            self.prev_steer_output = 0.0  # Reset filter memory
        else:
            # Your existing steering calculation
            if not hasattr(self, 'steer_curve') or not self.steer_curve or not hasattr(self, 'current_vel'):
                steer = -self.in_cmd.actuation.steer_cmd
            else:
                try:
                    max_steer_ratio = np.interp(
                        abs(self.current_vel.x), [v.x for v in self.steer_curve], [v.y for v in self.steer_curve]
                    )
                    steer = self.first_order_steering(-self.in_cmd.actuation.steer_cmd) * max_steer_ratio
                except Exception as e:
                    self.node.get_logger().error(f"Error calculating steering: {e}")
                    steer = -self.in_cmd.actuation.steer_cmd
    
        brake = self.in_cmd.actuation.brake_cmd
        manual_gear_shift = False
        
        msg = CarlaEgoVehicleControl()
        msg.header.stamp = now
        msg.throttle = throttle
        msg.brake = brake
        msg.manual_gear_shift = manual_gear_shift
        msg.steer = steer
        self._vehicle_control_command_publisher.publish(msg)
    
    def destroy(self):
        self.node.get_logger().info("Destroying ControlCommand subscriber")
        self._vehicle_control_command_publisher.destroy()
        self._control_command_subscriber.destroy()