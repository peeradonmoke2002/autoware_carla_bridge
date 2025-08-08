#!/usr/bin/python3
# control_command.py
from tier4_vehicle_msgs.msg import ActuationCommandStamped
from rclpy.node import Node
from carla_msgs.msg import CarlaEgoVehicleControl
import numpy as np
import csv

class ControlCommand(object):

    def __init__(self, node: Node, steer_map_path=''):
        self.node = node
        # self.ego_actor = ego_actor
        # self.physics_control = None
        # self.timestamp = None
        # self.tau = 0.2
        # self.prev_timestamp = None
        # self.prev_steer_output = 0.0
        self.in_cmd = None  # Initialize to None
        # self.physics_control = ego_actor.get_physics_control()
        
        # Load steering map from CSV
        try:
            with open(steer_map_path, newline='') as csvfile:
                csv_reader = csv.reader(csvfile)
                self._tire_angle = np.float32(next(csv_reader))
                self._steer_cmd = np.float32(next(csv_reader))
                self.node.get_logger().info(f"Loaded steering map from {steer_map_path}")
        except Exception as e:
            self.node.get_logger().error(f"Failed to load steering map: {e}")
            # Initialize with default values if file loading fails
            self._tire_angle = np.array([-1.0, 0.0, 1.0], dtype=np.float32)
            self._steer_cmd = np.array([-1.0, 0.0, 1.0], dtype=np.float32)
            
        self._control_command_subscriber = self.node.create_subscription(
            ActuationCommandStamped, '~/input/actuation',
            self.control_callback, 1)
        
        self._vehicle_control_command_publisher = self.node.create_publisher(
            CarlaEgoVehicleControl, '~/output/control', 1)
        
    # def first_order_steering(self, steer_input):
    #     """First order steering model."""
    #     steer_output = 0.0
    #     if self.prev_timestamp is None:
    #         self.prev_timestamp = self.timestamp

    #     dt = self.timestamp - self.prev_timestamp
    #     if dt > 0.0:
    #         steer_output = self.prev_steer_output + (steer_input - self.prev_steer_output) * (
    #             dt / (self.tau + dt)
    #         )
    #     self.prev_steer_output = steer_output
    #     self.prev_timestamp = self.timestamp
    #     return steer_output



    def _convert_from_tire_to_steer(self, tire_angle) -> float:
        nearest_idx = (np.abs(self._tire_angle - tire_angle)).argmin()
        return float(self._steer_cmd[nearest_idx])

    def control_callback(self, msg: ActuationCommandStamped):
        """Callback for ActuationCommand messages."""
        self.in_cmd = msg
        
    # def update(self):
    #     """Convert and publish CARLA Ego Vehicle Control to AUTOWARE."""
    #     now = self.node.get_clock().now().to_msg()
    #     self.timestamp = now.sec + now.nanosec * 1e-9
    #     throttle = self.in_cmd.actuation.accel_cmd
    #     steer_curve = self.physics_control.steering_curve
    #     current_vel = self.ego_actor.get_velocity()
    #     max_steer_ratio = numpy.interp(
    #         abs(current_vel.x), [v.x for v in steer_curve], [v.y for v in steer_curve]
    #     )
    #     steer = self.first_order_steering(-self.in_cmd.actuation.steer_cmd) * max_steer_ratio
    #     brake = self.in_cmd.actuation.brake_cmd
    #     manual_gear_shift = False
        
    #     msg = CarlaEgoVehicleControl()
    #     msg.header.frame_id = "base_link"
    #     msg.header.stamp = now
    #     msg.throttle = throttle
    #     msg.brake = brake
    #     msg.manual_gear_shift = manual_gear_shift
    #     msg.steer = steer
    #     self._vehicle_control_command_publisher.publish(msg)
    
    
    def update(self):
        # Skip if no command has been received yet
        if self.in_cmd is None:
            return
            
        steer = self._convert_from_tire_to_steer(self.in_cmd.actuation.steer_cmd)
        output_control_command = CarlaEgoVehicleControl()
        output_control_command.header.stamp = self.node.get_clock().now().to_msg()
        output_control_command.throttle = self.in_cmd.actuation.accel_cmd
        output_control_command.brake = self.in_cmd.actuation.brake_cmd
        output_control_command.manual_gear_shift = False
        output_control_command.steer = steer
        self._vehicle_control_command_publisher.publish(output_control_command)

        

    def destroy(self):
        self.node.get_logger().info("Destroying ControlCommand subscriber")
        self._vehicle_control_command_publisher.destroy()
        self._control_command_subscriber.destroy()