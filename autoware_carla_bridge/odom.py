#!/usr/bin/python3
# odom.py
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped


class Odom(object):

    def __init__(self, node: Node):
        self.node = node
        self.input_odom = Odometry()
        self._odom_subscriber = self.node.create_subscription(
            Odometry, '~/input/odometry',
            self.odom_callback, 1)
        self._odom_publisher = self.node.create_publisher(
            Odometry, '~/output/odometry', 1)
        
        # Initialize TF broadcaster for map->odom transform
        self.tf_broadcaster = TransformBroadcaster(self.node)

    def odom_callback(self, msg: Odometry):
        self.input_odom = msg

    def update(self):
        odom_msg = Odometry()
        odom_msg.header.stamp = self.input_odom.header.stamp
        odom_msg.header.frame_id = 'odom' 
        odom_msg.child_frame_id = 'base_link'
        odom_msg.pose = self.input_odom.pose
        odom_msg.twist = self.input_odom.twist     
        self._odom_publisher.publish(odom_msg)
  
        # Create transforms for the TF tree
        transforms = []
        
        # odom → base_link transform
        t_odom_base = TransformStamped()
        t_odom_base.header.stamp = odom_msg.header.stamp
        t_odom_base.header.frame_id = "odom"
        t_odom_base.child_frame_id = "base_link"
        t_odom_base.transform.translation.x = odom_msg.pose.pose.position.x
        t_odom_base.transform.translation.y = odom_msg.pose.pose.position.y
        t_odom_base.transform.translation.z = odom_msg.pose.pose.position.z
        t_odom_base.transform.rotation = odom_msg.pose.pose.orientation
        transforms.append(t_odom_base)
        
        # map → odom transform (identity transform since we're assuming odom is at origin of map)
        # In a real system, this would typically be published by a localization system
        t_map_odom = TransformStamped()
        t_map_odom.header.stamp = odom_msg.header.stamp
        t_map_odom.header.frame_id = "map"
        t_map_odom.child_frame_id = "odom"
        t_map_odom.transform.translation.x = 0.0
        t_map_odom.transform.translation.y = 0.0
        t_map_odom.transform.translation.z = 0.0
        t_map_odom.transform.rotation.w = 1.0
        transforms.append(t_map_odom)

        self.tf_broadcaster.sendTransform(transforms)

    def destroy(self):
        self.node.get_logger().info("Destroying Odom")
        self._odom_subscriber.destroy()
        self._odom_publisher.destroy()
        # TF broadcaster doesn't need explicit destruction