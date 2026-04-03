#!/usr/bin/env python3

from __future__ import annotations

import os
import time
from typing import Dict, Optional

import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseArray, Pose, Quaternion
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, ReliabilityPolicy, QoSProfile
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray

from .color_detector import ColorDetector
from .utils import (
    estimate_depth_from_diameter_px,
    pixel_to_camera_xyz,
    rotation_matrix_from_rpy,
    rotate_vector_by_quaternion,
)


class ColorDetectorNode(Node):
    """ROS2 node: detect red/blue pads from HSV masks and publish poses + RViz markers."""

    def __init__(self) -> None:
        super().__init__("color_detector_node")

        def as_bool(v) -> bool:
            if isinstance(v, bool):
                return v
            if isinstance(v, str):
                return v.strip().lower() in ("true", "1", "yes", "y", "on")
            return bool(v)

        def as_list3(v) -> list[float]:
            if isinstance(v, (list, tuple)) and len(v) == 3:
                return [float(v[0]), float(v[1]), float(v[2])]
            if isinstance(v, str):
                parsed = yaml.safe_load(v)
                if isinstance(parsed, (list, tuple)) and len(parsed) == 3:
                    return [float(parsed[0]), float(parsed[1]), float(parsed[2])]
            raise ValueError(f"Expected a 3-item list for parameter, got: {v!r}")

        # --- Parameters (tunable from launch) ---
        self.declare_parameter("camera_topic", "/camera/image_raw")
        self.declare_parameter("pose_topic", "/drone1/pose")
        self.declare_parameter("use_drone_pose_transform", True)
        self.declare_parameter("require_pose_for_map_transform", False)

        # HSV threshold YAML file with structure:
        #   red: {hsv_lower, hsv_upper, hsv_lower2, hsv_upper2}
        #   blue: {hsv_lower, hsv_upper}
        #   general: {min_contour_area, max_contour_area, pad_real_size_m, debug}
        self.declare_parameter("thresholds_file", "config/color_thresholds.yaml")

        # Output topics
        self.declare_parameter("pads_pose_topic", "/skyw_detection/pads")
        self.declare_parameter("markers_topic", "/skyw_detection/pad_markers")
        self.declare_parameter("debug_image_topic_prefix", "/skyw_detection/debug")

        # Marker placement / frames
        self.declare_parameter("camera_frame_id", "camera_link")
        self.declare_parameter("marker_frame_id", "map")
        self.declare_parameter("marker_scale_m", 0.15)

        # Camera intrinsics for depth estimation + pixel->3D
        self.declare_parameter("camera_fx", 320.0)
        self.declare_parameter("camera_fy", 320.0)
        self.declare_parameter("camera_cx", -1.0)
        self.declare_parameter("camera_cy", -1.0)

        # Camera extrinsics relative to the drone/body frame.
        # RPY is roll-pitch-yaw in radians, applied in ZYX order.
        self.declare_parameter("camera_to_drone_translation", [0.0, 0.0, 0.0])
        self.declare_parameter("camera_to_drone_rpy", [0.0, 0.0, 0.0])

        # Throttling (process every frame by default)
        self.declare_parameter("min_process_period_sec", 0.0)

        # --- Load thresholds YAML + build detector ---
        thresholds_file_param = self.get_parameter("thresholds_file").value
        thresholds_file = self._resolve_thresholds_path(thresholds_file_param)

        try:
            with open(thresholds_file, "r", encoding="utf-8") as f:
                thresholds: Dict = yaml.safe_load(f)
        except FileNotFoundError as exc:
            raise RuntimeError(
                f"thresholds_file not found: {thresholds_file}. "
                f"Make sure it exists under skyw_detection share dir."
            ) from exc

        self.detector = ColorDetector(thresholds)
        self.pad_real_size_m = float(thresholds.get("general", {}).get("pad_real_size_m", self.detector.pad_real_size_m))
        self.publish_debug_images = bool(thresholds.get("general", {}).get("debug", self.detector.debug))

        # --- Camera intrinsics ---
        self.fx = float(self.get_parameter("camera_fx").value)
        self.fy = float(self.get_parameter("camera_fy").value)
        self.cx = float(self.get_parameter("camera_cx").value)
        self.cy = float(self.get_parameter("camera_cy").value)

        # --- Camera extrinsics ---
        t = as_list3(self.get_parameter("camera_to_drone_translation").value)
        rpy = as_list3(self.get_parameter("camera_to_drone_rpy").value)
        self.t_camera_to_drone = np.array(t, dtype=float).reshape(3)
        self.R_camera_to_drone = rotation_matrix_from_rpy(rpy[0], rpy[1], rpy[2])

        # --- Pose / transformation state ---
        self.use_drone_pose_transform: bool = as_bool(self.get_parameter("use_drone_pose_transform").value)
        self.require_pose_for_map_transform: bool = as_bool(
            self.get_parameter("require_pose_for_map_transform").value
        )
        self.last_drone_pose = None  # type: Optional[object]
        self.pose_topic = str(self.get_parameter("pose_topic").value)

        # --- Image processing throttling ---
        self.min_process_period_sec = float(self.get_parameter("min_process_period_sec").value)
        self.last_process_time = 0.0

        qos_profile = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        # --- Subscribers ---
        camera_topic = str(self.get_parameter("camera_topic").value)
        self.image_sub = self.create_subscription(Image, camera_topic, self.on_image, qos_profile)
        self.get_logger().info(f"Subscribed to camera topic: {camera_topic}")

        if self.use_drone_pose_transform and self.pose_topic:
            from geometry_msgs.msg import PoseStamped

            self.pose_sub = self.create_subscription(
                PoseStamped, self.pose_topic, self.on_pose, qos_profile
            )
            self.get_logger().info(f"Subscribed to drone pose topic: {self.pose_topic}")

        # --- Publishers ---
        self.bridge = CvBridge()

        self.pads_pose_pub = self.create_publisher(PoseArray, str(self.get_parameter("pads_pose_topic").value), 10)
        self.markers_pub = self.create_publisher(
            MarkerArray, str(self.get_parameter("markers_topic").value), 10
        )

        self.camera_frame_id = str(self.get_parameter("camera_frame_id").value)
        self.marker_frame_id = str(self.get_parameter("marker_frame_id").value)
        self.marker_scale_m = float(self.get_parameter("marker_scale_m").value)

        # Optional debug mask image publishers (useful to tune HSV quickly).
        debug_prefix = str(self.get_parameter("debug_image_topic_prefix").value)
        if self.publish_debug_images:
            self.red_mask_pub = self.create_publisher(
                Image, f"{debug_prefix}/red_mask", qos_profile
            )
            self.blue_mask_pub = self.create_publisher(
                Image, f"{debug_prefix}/blue_mask", qos_profile
            )

    def _resolve_thresholds_path(self, thresholds_file_param: str) -> str:
        # Allow either absolute path or package-relative path.
        if os.path.isabs(thresholds_file_param):
            return thresholds_file_param
        share_dir = get_package_share_directory("skyw_detection")
        return os.path.join(share_dir, thresholds_file_param)

    def on_pose(self, msg) -> None:
        self.last_drone_pose = msg

    def _color_to_marker_color(self, color_name: str) -> tuple[float, float, float]:
        if color_name == "red":
            return (1.0, 0.1, 0.1)
        if color_name == "blue":
            return (0.1, 0.1, 1.0)
        return (1.0, 1.0, 1.0)

    def on_image(self, msg: Image) -> None:
        now_s = time.time()
        if self.min_process_period_sec > 0.0:
            if (now_s - self.last_process_time) < self.min_process_period_sec:
                return
            self.last_process_time = now_s

        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().warn(f"Failed to convert image to BGR8: {e}")
            return

        height, width = bgr.shape[:2]
        if self.cx < 0.0:
            self.cx = float(width) / 2.0
        if self.cy < 0.0:
            self.cy = float(height) / 2.0

        detections, debug_masks = self.detector.detect_pads(bgr, return_masks=self.publish_debug_images)

        # Prepare output containers.
        pads_pose_msg = PoseArray()
        pads_pose_msg.header = Header(stamp=self.get_clock().now().to_msg(), frame_id=self.marker_frame_id)

        marker_array_msg = MarkerArray()

        if not detections:
            self.pads_pose_pub.publish(pads_pose_msg)
            self.markers_pub.publish(marker_array_msg)
            return

        # Transform parameters.
        if self.use_drone_pose_transform:
            if self.last_drone_pose is None:
                if self.require_pose_for_map_transform:
                    return
                # Fallback: publish in camera frame.
                current_frame_id = self.camera_frame_id
                pads_pose_msg.header.frame_id = current_frame_id
            else:
                current_frame_id = self.marker_frame_id
                pads_pose_msg.header.frame_id = current_frame_id

        else:
            current_frame_id = self.camera_frame_id
            pads_pose_msg.header.frame_id = current_frame_id

        # For unique marker IDs.
        base_ns = "skyw_detection_pads"
        stamp = self.get_clock().now().to_msg()

        for i, det in enumerate(detections):
            u, v = det.centroid_px
            _, _, w, h = det.bbox_xywh
            diameter_px = max(float(w), float(h))

            z_m = estimate_depth_from_diameter_px(diameter_px, fx_px=self.fx, pad_real_size_m=self.pad_real_size_m)
            p_cam = pixel_to_camera_xyz(
                u_px=float(u),
                v_px=float(v),
                z_m=z_m,
                fx_px=self.fx,
                fy_px=self.fy,
                cx_px=self.cx,
                cy_px=self.cy,
            )

            if self.use_drone_pose_transform and self.last_drone_pose is not None:
                # 1) camera optical -> drone/body frame.
                p_drone = self.R_camera_to_drone @ p_cam + self.t_camera_to_drone

                # 2) drone/body -> map/world frame using drone pose orientation + translation.
                pose = self.last_drone_pose.pose
                q_map_drone = pose.orientation
                p_map_drone = np.array([pose.position.x, pose.position.y, pose.position.z], dtype=float)
                p_map = rotate_vector_by_quaternion(q_map_drone, p_drone) + p_map_drone
                p_out = p_map
            else:
                p_out = p_cam

            pose_item = Pose()
            pose_item.position.x = float(p_out[0])
            pose_item.position.y = float(p_out[1])
            pose_item.position.z = float(p_out[2])
            pose_item.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
            pads_pose_msg.poses.append(pose_item)

            r, g, b = self._color_to_marker_color(det.color)
            marker = Marker()
            marker.header.stamp = stamp
            marker.header.frame_id = current_frame_id
            marker.ns = base_ns
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD

            marker.pose = pose_item
            marker.scale.x = self.marker_scale_m
            marker.scale.y = self.marker_scale_m
            marker.scale.z = self.marker_scale_m

            marker.color.r = r
            marker.color.g = g
            marker.color.b = b
            marker.color.a = 0.9
            marker.lifetime.sec = 0
            marker.lifetime.nanosec = int(0.2 * 1e9)

            marker_array_msg.markers.append(marker)

        self.pads_pose_pub.publish(pads_pose_msg)
        self.markers_pub.publish(marker_array_msg)

        if self.publish_debug_images:
            try:
                red_mask = debug_masks.get("red")
                blue_mask = debug_masks.get("blue")
                if red_mask is not None:
                    red_msg = self.bridge.cv2_to_imgmsg(red_mask, encoding="mono8")
                    red_msg.header = msg.header
                    self.red_mask_pub.publish(red_msg)
                if blue_mask is not None:
                    blue_msg = self.bridge.cv2_to_imgmsg(blue_mask, encoding="mono8")
                    blue_msg.header = msg.header
                    self.blue_mask_pub.publish(blue_msg)
            except Exception as e:
                self.get_logger().warn(f"Failed to publish debug masks: {e}")


def main() -> None:
    rclpy.init()
    node = ColorDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

