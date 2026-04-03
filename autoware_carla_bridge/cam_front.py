#!/usr/bin/python3
# cam_front.py
import math
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, CameraInfo

_CAM_FRAME = "CAM_FRONT/camera_optical_link"
_CAM_FOV_DEG = 70.0  # matches sensor_kit xacro fov="1.22" rad


def _compute_camera_info(ci_in, frame_id, fov_deg=_CAM_FOV_DEG):
    """Compute camera intrinsics from FOV and image dimensions."""
    w = ci_in.width
    h = ci_in.height
    cx = w / 2.0
    cy = h / 2.0
    fx = w / (2.0 * math.tan(math.radians(fov_deg / 2.0)))
    fy = fx
    out = CameraInfo()
    out.header.stamp = ci_in.header.stamp
    out.header.frame_id = frame_id
    out.width = w
    out.height = h
    out.distortion_model = "plumb_bob"
    out.d = [0.0, 0.0, 0.0, 0.0, 0.0]
    out.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
    out.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    out.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
    return out


class CamFront(object):
    def __init__(self, node: Node):
        self.node = node
        self.input_image = None
        self.input_camera_info = None

        self._image_subscriber = self.node.create_subscription(
            Image, '~/input/image', self.image_callback, self._create_sensor_qos())
        self._image_publisher = self.node.create_publisher(
            Image, '~/output/image', self._create_sensor_qos())

        self._image_info_subscriber = self.node.create_subscription(
            CameraInfo, '~/input/camera_info', self.image_info_callback, self._create_sensor_qos())
        self._image_info_publisher = self.node.create_publisher(
            CameraInfo, '~/output/camera_info', self._create_sensor_qos())


    def _create_sensor_qos(self):
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.RELIABLE
        return qos
    
    def image_callback(self, msg: Image):
        self.input_image = msg

    def image_info_callback(self, msg: CameraInfo):
        self.input_camera_info = msg

    def update(self):
        if self.input_image is not None:
            out = Image()
            out.header.stamp = self.input_image.header.stamp
            out.header.frame_id = _CAM_FRAME
            out.height = self.input_image.height
            out.width = self.input_image.width
            out.encoding = self.input_image.encoding
            out.is_bigendian = self.input_image.is_bigendian
            out.step = self.input_image.step
            out.data = self.input_image.data
            self._image_publisher.publish(out)

        if self.input_camera_info is not None:
            self._image_info_publisher.publish(
                _compute_camera_info(self.input_camera_info, _CAM_FRAME))

    def destroy(self):
        self.node.get_logger().info("Destroying CamFront")
        self._image_subscriber.destroy()
        self._image_publisher.destroy()
        self._image_info_subscriber.destroy()
        self._image_info_publisher.destroy()
