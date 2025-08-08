#!/usr/bin/python3
# velocity_state_report.py
from autoware_vehicle_msgs.msg import VelocityReport
from rclpy.node import Node
from nav_msgs.msg import Odometry



class VelocityStateReport(object):

    def __init__(self, node: Node):
        self.node = node
        self.yaw_rate = 0.0
        self.longitudinal_velocity = 0.0
        self.lateral_velocity = 0.0
        self.odom = Odometry()
        self._odometry_subscriber = self.node.create_subscription(
            Odometry, '~/input/odometry',
            self.odometry_callback, 1)
        self.pub_vel_state = self.node.create_publisher(
            VelocityReport, '~/output/velocity_status', 1)

    def odometry_callback(self, msg: Odometry):
        self.odom = msg
        self.yaw_rate = msg.twist.twist.angular.z
        self.longitudinal_velocity = msg.twist.twist.linear.x
        self.lateral_velocity = msg.twist.twist.linear.y
        
        
    def update(self):
        out_vel_state = VelocityReport()
        out_vel_state.header.frame_id = "base_link"
        out_vel_state.header.stamp = self.node.get_clock().now().to_msg()
        out_vel_state.heading_rate = -self.yaw_rate
        out_vel_state.longitudinal_velocity = self.longitudinal_velocity
        out_vel_state.lateral_velocity = -self.lateral_velocity
        self.pub_vel_state.publish(out_vel_state)

    def destroy(self):
        self.node.get_logger().info("Destroying VelocityStateReport")
        self._odometry_subscriber.destroy()
        self.pub_vel_state.destroy()