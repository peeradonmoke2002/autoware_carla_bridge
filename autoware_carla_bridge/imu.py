#!/usr/bin/python3
# imu.py
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile
from sensor_msgs.msg import Imu as ImuMsg


class Imu(object):

    def __init__(self, node: Node):
        self.node = node
        self._input_imu = None
        self._subscriber = self.node.create_subscription(
            ImuMsg, '~/input/imu', self._callback, self._create_sensor_qos())
        self._publisher = self.node.create_publisher(
            ImuMsg, '~/output/imu', self._create_sensor_qos())

    def _create_sensor_qos(self):
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.RELIABLE
        return qos
    

    def _callback(self, msg: ImuMsg):
        self._input_imu = msg

    def update(self):
        if self._input_imu is None:
            return
        out = ImuMsg()
        out.header.stamp = self._input_imu.header.stamp
        out.header.frame_id = 'tamagawa/imu_link'
        out.orientation = self._input_imu.orientation
        out.orientation_covariance = self._input_imu.orientation_covariance
        out.angular_velocity = self._input_imu.angular_velocity
        out.angular_velocity_covariance = self._input_imu.angular_velocity_covariance
        out.linear_acceleration = self._input_imu.linear_acceleration
        out.linear_acceleration_covariance = self._input_imu.linear_acceleration_covariance
        self._publisher.publish(out)

    def destroy(self):
        self.node.get_logger().info("Destroying Imu")
        self._subscriber.destroy()
        self._publisher.destroy()
