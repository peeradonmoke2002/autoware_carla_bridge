#!/usr/bin/python3
# cam_camera0.py - Camera publisher for YOLOX object detection (camera0)
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import QoSProfile, ReliabilityPolicy
from tf2_ros import Buffer, TransformListener, TransformException, TransformBroadcaster, ConnectivityException, ExtrapolationException
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Image, CameraInfo

class CamCamera0(object):
    def __init__(self, node: Node):
        self.node = node

        # Input holders (None until first msg arrives)
        self.input_image = None
        self.input_camera_info = None

        # IO - Subscribe to CARLA camera input
        self._image_subscriber = self.node.create_subscription(
            Image, '~/input/image', self.image_callback, self._create_sensor_qos())
        # Publish directly to camera0 topic for YOLOX with sensor QoS
        self._image_publisher = self.node.create_publisher(
            Image, '~/output/camera0/image_rect_color', self._create_sensor_qos())

        self._image_info_subscriber = self.node.create_subscription(
            CameraInfo, '~/input/camera_info', self.image_info_callback, self._create_sensor_qos())
        self._image_info_publisher = self.node.create_publisher(
            CameraInfo, '~/output/camera0/camera_info', self._create_sensor_qos())

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node)
        self.tf_broadcaster = TransformBroadcaster(self.node)

    def _create_sensor_qos(self):
        """Create QoS profile for sensor data with BEST_EFFORT reliability."""
        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        return qos

    # --- Callbacks ---
    def image_callback(self, msg: Image):
        self.input_image = msg

    def image_info_callback(self, msg: CameraInfo):
        self.input_camera_info = msg

    # --- Publishers for republished image/info with camera0 frame_id ---
    def image_update(self):
        if self.input_image is None:
            return
        out = Image()
        # Use synchronized timestamp for camera-lidar fusion
        out.header.stamp = self.input_camera_info.header.stamp
        out.header.frame_id = "camera0/camera_optical_link"
        out.height = self.input_image.height
        out.width = self.input_image.width
        out.encoding = self.input_image.encoding
        out.is_bigendian = self.input_image.is_bigendian
        out.step = self.input_image.step
        out.data = self.input_image.data
        self._image_publisher.publish(out)

    def image_info_update(self):
        if self.input_camera_info is None:
            return
        out = CameraInfo()
        # Use synchronized timestamp for camera-lidar fusion
        out.header.stamp = self.input_camera_info.header.stamp
        out.header.frame_id = "camera0/camera_optical_link"
        out.width = self.input_camera_info.width
        out.height = self.input_camera_info.height
        out.distortion_model = "plumb_bob"
        out.d = list(self.input_camera_info.d)
        out.k = list(self.input_camera_info.k)
        out.r = list(self.input_camera_info.r)
        out.p = list(self.input_camera_info.p)
        out.binning_x = self.input_camera_info.binning_x
        out.binning_y = self.input_camera_info.binning_y
        self._image_info_publisher.publish(out)

    def update(self):
        # Republish data (if any)
        self.image_update()
        self.image_info_update()


        if not self.tf_buffer.can_transform("base_link", "ego_vehicle/rgb_front", rclpy.time.Time()):
            return

        try:
            cam_tf_msg = self.tf_buffer.lookup_transform(
                "ego_vehicle", "ego_vehicle/rgb_front", rclpy.time.Time()
            )
        except (TransformException, ConnectivityException, ExtrapolationException):
            self.node.get_logger().warn("Transform not available")
            return


        cam_tf_msg.header.stamp = self.node.get_clock().now().to_msg()
        cam_tf_msg.header.frame_id = "camera0/camera_optical_link" 
        cam_tf_msg.transform.translation = cam_tf_msg.transform.translation
        cam_tf_msg.transform.rotation = cam_tf_msg.transform.rotation
        self.tf_broadcaster.sendTransform(cam_tf_msg)

    def destroy(self):
        self.node.get_logger().info("Destroying Cam")
        self._image_subscriber.destroy()
        self._image_publisher.destroy()
        self._image_info_subscriber.destroy()
        self._image_info_publisher.destroy()

