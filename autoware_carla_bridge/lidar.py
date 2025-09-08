#!/usr/bin/python3
# lidar.py
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2


class Lidar(object):

    def __init__(self, node: Node):
        self.node = node
        self.input_pointcloud = PointCloud2()
        self._lidar_subscriber = self.node.create_subscription(
            PointCloud2, '~/input/lidar', self.lidar_callback, 1)
        self._lidar_ex_publisher = self.node.create_publisher(
            PointCloud2, '~/output/lidar', 1)

    def lidar_callback(self, msg: PointCloud2):
        self.input_pointcloud = msg

    def update(self):
        pc_in = self.input_pointcloud
        pc_out = PointCloud2()
        pc_out.header.frame_id = pc_in.header.frame_id
        pc_out.header.stamp = pc_in.header.stamp
        pc_out.height = pc_in.height
        pc_out.width = pc_in.width
        pc_out.is_bigendian = pc_in.is_bigendian
        pc_out.is_dense = pc_in.is_dense
        pc_out.fields = pc_in.fields
        pc_out.point_step = pc_in.point_step
        pc_out.row_step = pc_in.row_step
        pc_out.data = pc_in.data
        self._lidar_ex_publisher.publish(pc_out)

    def destroy(self):
        self.node.get_logger().info("Destroying Lidar")
        self._lidar_subscriber.destroy()
        self._lidar_ex_publisher.destroy()
