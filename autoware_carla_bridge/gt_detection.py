#!/usr/bin/python3
# gt_detection.py
#
# Ground truth detection bridge: converts CARLA ground truth objects to
# Autoware DetectedObjects, filtered by minimum LiDAR point count per class.
# This provides stable detections for the planning module by only publishing
# objects that are actually visible (have enough LiDAR returns).

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from tf2_ros import Buffer, TransformListener, TransformException, LookupException

from sensor_msgs.msg import PointCloud2
from derived_object_msgs.msg import ObjectArray

from autoware_perception_msgs.msg import (
    DetectedObjects,
    DetectedObject,
    DetectedObjectKinematics,
    ObjectClassification,
    Shape,
)
from geometry_msgs.msg import (
    PoseWithCovariance,
    TwistWithCovariance,
    Vector3,
)


class GroundTruthDetection(object):
    """Bridge from CARLA ground truth objects to Autoware DetectedObjects.

    Subscribes to:
      - ~/input/gt_objects   : derived_object_msgs/ObjectArray (CARLA ground truth)
      - ~/input/pointcloud   : sensor_msgs/PointCloud2 (LiDAR for point counting)

    Uses TF to get transforms:
      - map -> lidar (for transforming objects to lidar frame)
      - map -> base_link (for publishing detections in base_link frame)

    Publishes to:
      - ~/output/detected_objects : autoware_perception_msgs/DetectedObjects
    """

    # derived_object_msgs classification -> autoware ObjectClassification label
    CLASSIFICATION_MAP = {
        6: ObjectClassification.CAR,          # CLASSIFICATION_CAR
        7: ObjectClassification.TRUCK,        # CLASSIFICATION_TRUCK
        9: ObjectClassification.BUS,          # CLASSIFICATION_OTHER_VEHICLE -> BUS
        8: ObjectClassification.MOTORCYCLE,   # CLASSIFICATION_MOTORCYCLE
        5: ObjectClassification.BICYCLE,      # CLASSIFICATION_BIKE
        4: ObjectClassification.PEDESTRIAN,   # CLASSIFICATION_PEDESTRIAN
    }

    # Minimum LiDAR point thresholds per Autoware classification
    MIN_POINTS_DEFAULT = {
        ObjectClassification.CAR: 20,
        ObjectClassification.TRUCK: 20,
        ObjectClassification.BUS: 20,
        ObjectClassification.MOTORCYCLE: 10,
        ObjectClassification.BICYCLE: 10,
        ObjectClassification.PEDESTRIAN: 5,
    }

    def __init__(self, node: Node):
        self.node = node
        self.objects_msg = None
        self.pointcloud_msg = None

        sensor_qos = self._create_sensor_qos()

        # Subscribers
        self._objects_sub = self.node.create_subscription(
            ObjectArray, '~/input/gt_objects',
            self._objects_cb, sensor_qos)

        self._pc_sub = self.node.create_subscription(
            PointCloud2, '~/input/pointcloud',
            self._pointcloud_cb, sensor_qos)

        # Publisher
        self._det_pub = self.node.create_publisher(
            DetectedObjects, '~/output/detected_objects', 1)

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node)

    def _create_sensor_qos(self):
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        return qos

    def _objects_cb(self, msg):
        self.objects_msg = msg

    def _pointcloud_cb(self, msg):
        self.pointcloud_msg = msg

    # ------------------------------------------------------------------
    # TF transform helpers
    # ------------------------------------------------------------------

    def _transform_point(self, point, transform):
        """Transform a 3D point using a TransformStamped message."""
        t = transform.transform.translation
        r = transform.transform.rotation

        # Convert quaternion to rotation matrix
        R = self._quat_to_rot(r.x, r.y, r.z, r.w)

        # Apply transform: p_out = R @ p_in + t
        p_in = np.array(point)
        p_out = R @ p_in + np.array([t.x, t.y, t.z])
        return p_out

    def _transform_orientation(self, quat_in, transform):
        """Transform a quaternion using a TransformStamped message."""
        r = transform.transform.rotation

        # Quaternion multiplication: q_out = q_transform * q_in
        q_out = self._quat_multiply(
            (r.x, r.y, r.z, r.w),
            quat_in
        )
        return q_out

    @staticmethod
    def _quat_to_rot(q_x, q_y, q_z, q_w):
        """Quaternion (x,y,z,w) -> 3x3 rotation matrix."""
        return np.array([
            [1 - 2*(q_y*q_y + q_z*q_z),
             2*(q_x*q_y - q_w*q_z),
             2*(q_x*q_z + q_w*q_y)],
            [2*(q_x*q_y + q_w*q_z),
             1 - 2*(q_x*q_x + q_z*q_z),
             2*(q_y*q_z - q_w*q_x)],
            [2*(q_x*q_z - q_w*q_y),
             2*(q_y*q_z + q_w*q_x),
             1 - 2*(q_x*q_x + q_y*q_y)],
        ])

    @staticmethod
    def _quat_multiply(a, b):
        """Hamilton product of two quaternions (x,y,z,w)."""
        ax, ay, az, aw = a
        bx, by, bz, bw = b
        return (
            aw*bx + ax*bw + ay*bz - az*by,
            aw*by - ax*bz + ay*bw + az*bx,
            aw*bz + ax*by - ay*bx + az*bw,
            aw*bw - ax*bx - ay*by - az*bz,
        )

    # ------------------------------------------------------------------
    # Point cloud helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _extract_xyz(pc_msg):
        """Extract (N, 3) float32 xyz array from PointCloud2."""
        n = pc_msg.width * pc_msg.height
        if n == 0:
            return None
        ps = pc_msg.point_step
        # Build a structured dtype that reads x,y,z at their known offsets
        dt = np.dtype({
            'names': ['x', 'y', 'z'],
            'formats': [np.float32, np.float32, np.float32],
            'offsets': [0, 4, 8],
            'itemsize': ps,
        })
        s = np.frombuffer(pc_msg.data, dtype=dt)
        pts = np.column_stack([s['x'], s['y'], s['z']])
        # Filter NaN / inf
        valid = np.isfinite(pts).all(axis=1)
        return pts[valid]

    # ------------------------------------------------------------------
    # Core: count LiDAR points inside an oriented bounding box
    # ------------------------------------------------------------------

    @staticmethod
    def _count_points_in_obb(points, center, R_box_inv, half_ext):
        """Count points inside an oriented bounding box.

        All inputs must be in the same frame (lidar frame).

        Args:
            points:      (N, 3) points in lidar frame.
            center:      (3,) box centre in lidar frame.
            R_box_inv:   (3, 3) inverse rotation of the box in lidar frame.
            half_ext:    (3,) half-extents (dx, dy, dz).

        Returns:
            int: number of points inside.
        """
        # Pre-filter by bounding sphere for speed
        max_r = np.linalg.norm(half_ext) + 1.0  # 1 m margin
        diff = points - center
        dist_sq = np.einsum('ij,ij->i', diff, diff)
        mask = dist_sq <= max_r * max_r
        if not np.any(mask):
            return 0

        # Transform filtered points into box-local frame
        local = (R_box_inv @ diff[mask].T).T  # (M, 3)
        inside = (
            (np.abs(local[:, 0]) <= half_ext[0]) &
            (np.abs(local[:, 1]) <= half_ext[1]) &
            (np.abs(local[:, 2]) <= half_ext[2])
        )
        return int(np.sum(inside))

    # ------------------------------------------------------------------
    # Conversion: derived_object_msgs/Object -> DetectedObject
    # ------------------------------------------------------------------

    def _to_detected_object(self, obj, aw_label, tf_map_to_base):
        """Convert a CARLA ground-truth object to an Autoware DetectedObject
        in the base_link frame using TF transform."""

        det = DetectedObject()
        det.existence_probability = 1.0

        # --- classification ---
        cls = ObjectClassification()
        cls.label = aw_label
        cls.probability = 1.0
        det.classification = [cls]

        # --- kinematics (pose in base_link using TF) ---
        # CARLA pose convention: vehicles at ground, pedestrians at center
        height = obj.shape.dimensions[2]
        is_pedestrian = (aw_label == ObjectClassification.PEDESTRIAN)
        z_offset = 0.0 if is_pedestrian else height / 2.0

        obj_pos_map = [
            obj.pose.position.x,
            obj.pose.position.y,
            obj.pose.position.z + z_offset,
        ]
        pos_base = self._transform_point(obj_pos_map, tf_map_to_base)

        obj_q_map = (obj.pose.orientation.x, obj.pose.orientation.y,
                     obj.pose.orientation.z, obj.pose.orientation.w)
        q_base = self._transform_orientation(obj_q_map, tf_map_to_base)

        kin = DetectedObjectKinematics()
        pose_cov = PoseWithCovariance()
        pose_cov.pose.position.x = float(pos_base[0])
        pose_cov.pose.position.y = float(pos_base[1])
        pose_cov.pose.position.z = float(pos_base[2])
        pose_cov.pose.orientation.x = q_base[0]
        pose_cov.pose.orientation.y = q_base[1]
        pose_cov.pose.orientation.z = q_base[2]
        pose_cov.pose.orientation.w = q_base[3]
        kin.pose_with_covariance = pose_cov
        kin.has_position_covariance = False
        kin.orientation_availability = DetectedObjectKinematics.AVAILABLE

        # Twist (transform linear velocity to base_link frame)
        twist_cov = TwistWithCovariance()
        vel_map = [obj.twist.linear.x, obj.twist.linear.y, obj.twist.linear.z]
        # For velocity, only rotation part of transform applies (no translation)
        t = tf_map_to_base.transform.rotation
        R = self._quat_to_rot(t.x, t.y, t.z, t.w)
        vel_base = R @ np.array(vel_map)
        twist_cov.twist.linear.x = float(vel_base[0])
        twist_cov.twist.linear.y = float(vel_base[1])
        twist_cov.twist.linear.z = float(vel_base[2])
        kin.twist_with_covariance = twist_cov
        kin.has_twist = True
        kin.has_twist_covariance = False
        det.kinematics = kin

        # --- shape (bounding box) ---
        shape = Shape()
        shape.type = Shape.BOUNDING_BOX
        dims = Vector3()
        dims.x = float(obj.shape.dimensions[0])  # length
        dims.y = float(obj.shape.dimensions[1])  # width
        dims.z = float(obj.shape.dimensions[2])  # height
        shape.dimensions = dims
        det.shape = shape

        return det

    # ------------------------------------------------------------------
    # Main update (called at 30 Hz from bridge timer)
    # ------------------------------------------------------------------

    def update(self):
        if self.objects_msg is None or self.pointcloud_msg is None:
            return

        # Get the lidar frame from point cloud header
        lidar_frame = self.pointcloud_msg.header.frame_id
        if not lidar_frame:
            lidar_frame = "velodyne_top_changed"  # fallback

        # Extract lidar points (in lidar frame)
        pts_lidar = self._extract_xyz(self.pointcloud_msg)
        if pts_lidar is None or len(pts_lidar) == 0:
            return

        # Get TF: map -> lidar (for transforming objects to lidar frame for point counting)
        try:
            tf_map_to_lidar = self.tf_buffer.lookup_transform(
                lidar_frame,  # target frame
                "map",        # source frame
                rclpy.time.Time()  # get latest
            )
        except (TransformException, LookupException) as e:
            self.node.get_logger().warn(
                f"TF lookup failed (map -> {lidar_frame}): {e}",
                throttle_duration_sec=2.0
            )
            return

        # Get TF: map -> base_link (for publishing detections in base_link frame)
        try:
            tf_map_to_base = self.tf_buffer.lookup_transform(
                "base_link",  # target frame
                "map",        # source frame
                rclpy.time.Time()  # get latest
            )
        except (TransformException, LookupException) as e:
            self.node.get_logger().warn(
                f"TF lookup failed (map -> base_link): {e}",
                throttle_duration_sec=2.0
            )
            return

        # Debug: log point cloud stats (only occasionally)
        self.node.get_logger().info(
            f"LiDAR frame={lidar_frame}, {len(pts_lidar)} points, "
            f"GT objects: {len(self.objects_msg.objects)}",
            throttle_duration_sec=5.0
        )

        # --- process each ground-truth object ---
        detected = []
        for obj in self.objects_msg.objects:
            aw_label = self.CLASSIFICATION_MAP.get(obj.classification)
            if aw_label is None:
                continue

            min_pts = self.MIN_POINTS_DEFAULT.get(aw_label, 20)

            # CARLA pose convention differs by object type:
            # - Vehicles: pose is at ground level, shift up by half height
            # - Pedestrians: pose is already at center (torso), no shift needed
            height = obj.shape.dimensions[2]
            is_pedestrian = (aw_label == ObjectClassification.PEDESTRIAN)
            z_offset = 0.0 if is_pedestrian else height / 2.0

            obj_pos_map = [
                obj.pose.position.x,
                obj.pose.position.y,
                obj.pose.position.z + z_offset,
            ]

            # Transform corrected center: map -> lidar
            center_lidar = self._transform_point(obj_pos_map, tf_map_to_lidar)

            # Transform object orientation: map -> lidar
            obj_q_map = (obj.pose.orientation.x, obj.pose.orientation.y,
                         obj.pose.orientation.z, obj.pose.orientation.w)
            q_lidar = self._transform_orientation(obj_q_map, tf_map_to_lidar)
            R_obj_lidar = self._quat_to_rot(*q_lidar)
            R_obj_lidar_inv = R_obj_lidar.T

            half_ext = np.array([
                obj.shape.dimensions[0] / 2.0,
                obj.shape.dimensions[1] / 2.0,
                obj.shape.dimensions[2] / 2.0,
            ])

            # Count raw lidar points inside the OBB (all in lidar frame)
            n_pts = self._count_points_in_obb(pts_lidar, center_lidar,
                                              R_obj_lidar_inv, half_ext)

            # Debug: log objects with points
            if n_pts > 0:
                self.node.get_logger().info(
                    f"Object {obj.id}: class={obj.classification}, "
                    f"center_lidar=[{center_lidar[0]:.1f},{center_lidar[1]:.1f},{center_lidar[2]:.1f}], "
                    f"dims=[{obj.shape.dimensions[0]:.1f},{obj.shape.dimensions[1]:.1f},{obj.shape.dimensions[2]:.1f}], "
                    f"pts={n_pts}/{min_pts}",
                    throttle_duration_sec=2.0
                )

            if n_pts >= min_pts:
                det = self._to_detected_object(obj, aw_label, tf_map_to_base)
                detected.append(det)

        # --- publish ---
        msg = DetectedObjects()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.objects = detected

        # Debug: log summary
        self.node.get_logger().info(
            f"Published {len(detected)}/{len(self.objects_msg.objects)} objects",
            throttle_duration_sec=2.0
        )

        self._det_pub.publish(msg)

    def destroy(self):
        self.node.get_logger().info("Destroying GroundTruthDetection")
        self._objects_sub.destroy()
        self._pc_sub.destroy()
        self._det_pub.destroy()
