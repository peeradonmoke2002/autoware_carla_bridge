#!/usr/bin/python3
# control_command.py
from tier4_vehicle_msgs.msg import ActuationCommandStamped
from rclpy.node import Node
from carla_msgs.msg import CarlaEgoVehicleControl
from carla_msgs.msg import CarlaEgoVehicleInfo
import numpy as np
from nav_msgs.msg import Odometry


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

        # P1: Control timeout tracking
        self._last_cmd_time = None
        self._cmd_timeout = 0.5  # seconds

        # P2: Timestamp validation and deduplication
        self._last_cmd_timestamp = None

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
        """First order steering model.

        Gracefully handles:
        - Early control commands before first simulation tick (returns raw input)
        - Multiple commands in same CARLA tick (preserves filter state, no zero spike)
        """
        # Guard against control commands arriving before first sensor callback
        if self.timestamp is None:
            return steer_input  # No filtering yet, return raw command

        # Initialize on first call
        if self.prev_timestamp is None:
            self.prev_timestamp = self.timestamp
            self.prev_steer_output = steer_input
            return steer_input

        dt = self.timestamp - self.prev_timestamp

        # Multiple commands in same simulation tick (dt = 0)
        # Preserve filter state to avoid zero spike - return previous output
        if dt <= 0.0:
            return self.prev_steer_output

        # Normal case: time has advanced, apply low-pass filter
        steer_output = self.prev_steer_output + (steer_input - self.prev_steer_output) * (
            dt / (self.tau + dt)
        )
        self.prev_steer_output = steer_output
        self.prev_timestamp = self.timestamp
        return steer_output


    def control_callback(self, msg: ActuationCommandStamped):
        """Callback for ActuationCommand messages.

        P2 Implementation: Timestamp validation and deduplication.
        - Rejects commands that don't match current sim time (stale)
        - Deduplicates repeated timestamps (multiple deliveries)
        - Warns on timestamp mismatches
        """
        # Extract command timestamp from message header
        cmd_timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        # Get current simulation time
        now = self.node.get_clock().now().to_msg()
        sim_time = now.sec + now.nanosec * 1e-9

        # P2a: Check for timestamp mismatch (command is stale or too far in future)
        time_delta = abs(cmd_timestamp - sim_time)
        if time_delta > 0.1:  # 100ms threshold
            self.node.get_logger().warning(
                f"\033[93mWARNING: Command timestamp mismatch. Expected {sim_time:.3f}s, "
                f"got {cmd_timestamp:.3f}s (delta: {time_delta:.3f}s). Command ignored.\033[0m"
            )
            return

        # P2b: Deduplication check - reject if same timestamp as last command
        if self._last_cmd_timestamp is not None and abs(self._last_cmd_timestamp - cmd_timestamp) < 1e-6:
            self.node.get_logger().warning(
                f"\033[93mWARNING: Received duplicate command with timestamp {cmd_timestamp:.3f}s. Ignoring.\033[0m"
            )
            return

        # Update tracking fields
        self._last_cmd_timestamp = cmd_timestamp
        self._last_cmd_time = sim_time
        self.in_cmd = msg

        
    def update(self):
        now = self.node.get_clock().now().to_msg()
        sim_time = now.sec + now.nanosec * 1e-9

        # P3: Scenario reset detection - detect backward clock jump
        if self.prev_timestamp is not None and sim_time < self.prev_timestamp - 0.1:
            self.node.get_logger().info(
                f"Scenario reset detected (clock jumped from {self.prev_timestamp:.3f}s to {sim_time:.3f}s). "
                "Clearing filter state."
            )
            # Reset filter state on scenario restart
            self.prev_steer_output = 0.0
            self.prev_timestamp = None
            self.in_cmd = None
            self._last_cmd_time = None
            self._last_cmd_timestamp = None

        self.timestamp = sim_time

        # P1: Control timeout handling
        # If no command received within timeout window, publish safe brake command
        if self._last_cmd_time is None or (sim_time - self._last_cmd_time) > self._cmd_timeout:
            if self._last_cmd_time is not None:
                self.node.get_logger().warning(
                    f"\033[93mWARNING: Control command timeout. Last command received "
                    f"{sim_time - self._last_cmd_time:.3f}s ago. Publishing safe brake.\033[0m"
                )
                self._last_cmd_time = None
                self.in_cmd = None

            # Publish safe command (zero throttle, full brake)
            msg = CarlaEgoVehicleControl()
            msg.header.stamp = now
            msg.throttle = 0.0
            msg.brake = 1.0
            msg.steer = 0.0
            msg.manual_gear_shift = False
            self._vehicle_control_command_publisher.publish(msg)
            return

        if not hasattr(self, 'in_cmd') or self.in_cmd is None:
            return

        throttle = self.in_cmd.actuation.accel_cmd

        if self.current_vel is not None and abs(self.current_vel.x) < 0.1:  # < 0.1 m/s
            steer = 0.0
            self.prev_steer_output = 0.0  # Reset filter memory
        else:
            # steer_cmd is already normalized [-1, 1] (convert_steer_cmd: false, max_steer: 1.0)
            # CARLA handles speed-dependent steering internally — no need to multiply by max_steer_ratio
            if not self.steer_curve or self.current_vel is None:
                steer = -self.in_cmd.actuation.steer_cmd
            else:
                try:
                    steer = self.first_order_steering(-self.in_cmd.actuation.steer_cmd)
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