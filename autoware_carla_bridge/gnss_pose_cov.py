#!/usr/bin/python3
# gnss.py
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix


class GnssCov(object):

    def __init__(self, node: Node):
        self.node = node
        self.input_gnss = NavSatFix()
        self._gnss_subscriber = self.node.create_subscription(
            NavSatFix, '~/input/gnss',
            self.gnss_callback, 1)
        self._gnss_cov_publisher = self.node.create_publisher(
            NavSatFix, '~/output/gnss_cov', 1)

    def gnss_callback(self, msg: NavSatFix):
        self.input_gnss = msg
        
    def update(self):        
        gnss_cov_msg = NavSatFix()
        gnss_cov_msg.header.stamp = self.input_gnss.header.stamp
        gnss_cov_msg.header.frame_id = 'map'
        gnss_cov_msg.status = self.input_gnss.status
        gnss_cov_msg.latitude = self.input_gnss.latitude
        gnss_cov_msg.longitude = self.input_gnss.longitude
        gnss_cov_msg.altitude = self.input_gnss.altitude
        gnss_cov_msg.position_covariance = [
            0.1, 0.0, 0.0,
            0.0, 0.1, 0.0,
            0.0, 0.0, 0.1
        ]
        gnss_cov_msg.position_covariance_type = 2  

        self._gnss_cov_publisher.publish(gnss_cov_msg)
        

    def destroy(self):
        self.node.get_logger().info("Destroying GnssCov")
        self._gnss_subscriber.destroy()
        self._gnss_cov_publisher.destroy()