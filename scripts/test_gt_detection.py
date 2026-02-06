#!/usr/bin/env python3
"""
Quick test script to verify ground truth detection bridge works.
Run this alongside CARLA simulator to see if detections are published.

Usage:
  ros2 run autoware_carla_bridge test_gt_detection.py
"""

import rclpy
from rclpy.node import Node
from autoware_perception_msgs.msg import DetectedObjects


class GTDetectionTester(Node):
    def __init__(self):
        super().__init__('gt_detection_tester')

        self.sub = self.create_subscription(
            DetectedObjects,
            '/perception/object_recognition/detection/objects',
            self.detection_callback,
            10
        )

        self.get_logger().info("Listening for DetectedObjects on /perception/object_recognition/detection/objects")
        self.get_logger().info("Make sure CARLA and the bridge are running!")

    def detection_callback(self, msg):
        self.get_logger().info(
            f"Received {len(msg.objects)} detections in frame '{msg.header.frame_id}'"
        )

        for i, obj in enumerate(msg.objects):
            cls = obj.classification[0] if obj.classification else None
            cls_label = cls.label if cls else 0
            cls_names = {1: "CAR", 2: "TRUCK", 3: "BUS", 5: "MOTORCYCLE",
                         6: "BICYCLE", 7: "PEDESTRIAN"}
            cls_str = cls_names.get(cls_label, f"UNKNOWN({cls_label})")

            pos = obj.kinematics.pose_with_covariance.pose.position
            dims = obj.shape.dimensions

            self.get_logger().info(
                f"  [{i}] {cls_str}: pos=({pos.x:.1f}, {pos.y:.1f}, {pos.z:.1f}), "
                f"dims=({dims.x:.1f}×{dims.y:.1f}×{dims.z:.1f})"
            )


def main(args=None):
    rclpy.init(args=args)
    node = GTDetectionTester()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
