from tier4_vehicle_msgs.msg import ActuationCommandStamped
from rclpy.node import Node
from carla_msgs.msg import CarlaEgoVehicleControl
import numpy

class ControlCommand(object):

    def __init__(self, ego_actor, node: Node):
        self.node = node
        self.ego_actor = ego_actor
        self.physics_control = None
        self.timestamp = None
        self.tau = 0.2
        self.prev_timestamp = None
        self.prev_steer_output = 0.0
        self.physics_control = ego_actor.get_physics_control()
        self.sub_control = self.node.create_subscription(
            ActuationCommandStamped, "~/input/actuation", self.control_callback, 1
        )
        self._vehicle_control_command_publisher = self.node.create_publisher(
            CarlaEgoVehicleControl, '~/output/control', 1)
        
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

    def control_callback(self, in_cmd):
        """Convert and publish CARLA Ego Vehicle Control to AUTOWARE."""
        now = self.node.get_clock().now().to_msg()
        self.timestamp = now.sec + now.nanosec * 1e-9
        throttle = in_cmd.actuation.accel_cmd
        steer_curve = self.physics_control.steering_curve
        current_vel = self.ego_actor.get_velocity()
        max_steer_ratio = numpy.interp(
            abs(current_vel.x), [v.x for v in steer_curve], [v.y for v in steer_curve]
        )
        steer = self.first_order_steering(-in_cmd.actuation.steer_cmd) * max_steer_ratio
        brake = in_cmd.actuation.brake_cmd
        manual_gear_shift = False
        
        msg = CarlaEgoVehicleControl()
        msg.header.frame_id = "base_link"
        msg.header.stamp = now
        msg.throttle = throttle
        msg.brake = brake
        msg.manual_gear_shift = manual_gear_shift
        msg.steer = steer
        self._vehicle_control_command_publisher.publish(msg)

    def destroy(self):
        self.node.get_logger().info("Destroying ControlCommand subscriber")
        self.sub_control.destroy()