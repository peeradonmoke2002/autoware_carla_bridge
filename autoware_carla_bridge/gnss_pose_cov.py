#!/usr/bin/python3
# gnss.py
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry


class GnssCov(object):

    def __init__(self, node: Node):
        self.node = node
        self.input_gnss = Odometry()
        self._gnss_subscriber = self.node.create_subscription(
            Odometry, '~/input/gnss_cov',
            self.gnss_callback, 1)
        self._gnss_cov_publisher = self.node.create_publisher(
            PoseWithCovarianceStamped, '~/output/gnss_cov', 1)

    def gnss_callback(self, msg: Odometry):
        self.input_gnss = msg
        
    def update(self):
        # Check if we have received a valid message
        if not hasattr(self.input_gnss.header, 'stamp') or self.input_gnss.header.stamp.sec == 0:
            return
            
        pose_cov_msg = PoseWithCovarianceStamped()
        pose_cov_msg.header.stamp = self.input_gnss.header.stamp
        pose_cov_msg.header.frame_id = 'map'
        pose_cov_msg.pose.pose.position.x = self.input_gnss.pose.pose.position.x
        pose_cov_msg.pose.pose.position.y = self.input_gnss.pose.pose.position.y
        pose_cov_msg.pose.pose.position.z = self.input_gnss.pose.pose.position.z
        pose_cov_msg.pose.covariance = [
            0.1,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.1,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.1,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
        ]
        self._gnss_cov_publisher.publish(pose_cov_msg)

    def destroy(self):
        self.node.get_logger().info("Destroying GnssCov")
        self._gnss_subscriber.destroy()
        self._gnss_cov_publisher.destroy()