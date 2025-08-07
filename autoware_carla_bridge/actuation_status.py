from tier4_vehicle_msgs.msg import ActuationStatusStamped
from rclpy.node import Node

class ActuationStatus(object):

    def __init__(self, carla_control, node: Node):
        self.control = carla_control
        self.node = node
        self._actuation_status_publisher = self.node.create_publisher(
            ActuationStatusStamped, '~/output/actuation_status', 1)

    def update(self):
        msg = ActuationStatusStamped()
        msg.header.frame_id = "base_link"
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.status.accel_status = self.control.throttle
        msg.status.brake_status = self.control.brake
        msg.status.steer_status = -self.control.steer

        self._actuation_status_publisher.publish(msg)

    def destroy(self):
        self.node.get_logger().info("Destroying ActuationStatus publisher")
        self._actuation_status_publisher.destroy()