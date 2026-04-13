#!/usr/bin/env python3

from __future__ import annotations

import os
import time
from typing import Dict

import cv2
import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from px4_msgs.msg import VehicleAttitude, VehicleLocalPosition
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, ReliabilityPolicy, QoSProfile
from sensor_msgs.msg import Image
from skyw_interfaces.msg import LandingPad, LandingPadArray

from .color_detector import ColorDetector
from .utils import (
    intersect_ray_with_horizontal_plane,
    ned_to_world_xyz,
    pixel_to_camera_ray,
    rotation_matrix_from_quaternion_wxyz,
)


class SwarmPadDetectionNode(Node):
    """Detect red/blue landing pads from a downward-facing camera and publish world observations."""

    def __init__(self) -> None:
        super().__init__("swarm_pad_detection_node")

        def as_bool(value) -> bool:
            if isinstance(value, bool):
                return value
            if isinstance(value, str):
                return value.strip().lower() in ("true", "1", "yes", "y", "on")
            return bool(value)

        def as_list(value, expected_len: int) -> list[float]:
            if isinstance(value, (list, tuple)) and len(value) == expected_len:
                return [float(item) for item in value]
            if isinstance(value, str):
                parsed = yaml.safe_load(value)
                if isinstance(parsed, (list, tuple)) and len(parsed) == expected_len:
                    return [float(item) for item in parsed]
            raise ValueError(f"Expected a {expected_len}-item list, got: {value!r}")

        self.declare_parameter("drone_id", 2)
        self.declare_parameter("camera_topic", "/drone2/camera_down/image_raw")
        self.declare_parameter("position_topic", "/px4_2/fmu/out/vehicle_local_position_v1")
        self.declare_parameter("attitude_topic", "/px4_2/fmu/out/vehicle_attitude")
        self.declare_parameter("thresholds_file", "config/color_thresholds.yaml")
        self.declare_parameter("observations_topic", "/skyw_detection/pad_observations")
        self.declare_parameter("camera_fx", 541.7)
        self.declare_parameter("camera_fy", 541.7)
        self.declare_parameter("camera_cx", -1.0)
        self.declare_parameter("camera_cy", -1.0)
        self.declare_parameter("spawn_xy", "[-7.0, 4.0]")
        self.declare_parameter("camera_translation_body_frd", "[0.0, 0.0, -0.10]")
        self.declare_parameter("ground_plane_ned_z", 0.0)
        self.declare_parameter("ground_plane_world_z", 0.01)
        self.declare_parameter("publish_debug_images", False)
        self.declare_parameter("debug_image_topic_prefix", "/skyw_detection/debug")
        self.declare_parameter("min_process_period_sec", 0.0)
        self.declare_parameter("enable_visualization", False)
        self.declare_parameter("visualization_window_name", "Pad Detection")
        self.declare_parameter("visualization_scale", 0.45)

        self.drone_id = int(self.get_parameter("drone_id").value)
        self.fx = float(self.get_parameter("camera_fx").value)
        self.fy = float(self.get_parameter("camera_fy").value)
        self.cx = float(self.get_parameter("camera_cx").value)
        self.cy = float(self.get_parameter("camera_cy").value)
        self.spawn_xy = as_list(self.get_parameter("spawn_xy").value, 2)
        self.camera_translation_body_frd = np.array(
            as_list(self.get_parameter("camera_translation_body_frd").value, 3),
            dtype=float,
        )
        self.ground_plane_ned_z = float(self.get_parameter("ground_plane_ned_z").value)
        self.ground_plane_world_z = float(self.get_parameter("ground_plane_world_z").value)
        self.publish_debug_images = as_bool(self.get_parameter("publish_debug_images").value)
        self.min_process_period_sec = float(self.get_parameter("min_process_period_sec").value)
        self.enable_visualization = as_bool(self.get_parameter("enable_visualization").value)
        self.visualization_window_name = str(
            self.get_parameter("visualization_window_name").value
        )
        self.visualization_scale = max(
            0.1, float(self.get_parameter("visualization_scale").value)
        )

        thresholds_file = self._resolve_thresholds_path(
            str(self.get_parameter("thresholds_file").value)
        )
        with open(thresholds_file, "r", encoding="utf-8") as handle:
            thresholds: Dict = yaml.safe_load(handle) or {}
        self.detector = ColorDetector(thresholds)
        self.publish_debug_images = bool(
            thresholds.get("general", {}).get("debug", self.publish_debug_images)
        )

        self.last_position: VehicleLocalPosition | None = None
        self.last_attitude: VehicleAttitude | None = None
        self.last_process_time = 0.0
        self.bridge = CvBridge()

        qos_profile = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.create_subscription(
            Image,
            str(self.get_parameter("camera_topic").value),
            self.on_image,
            qos_profile,
        )
        self.create_subscription(
            VehicleLocalPosition,
            str(self.get_parameter("position_topic").value),
            self.on_position,
            qos_profile,
        )
        self.create_subscription(
            VehicleAttitude,
            str(self.get_parameter("attitude_topic").value),
            self.on_attitude,
            qos_profile,
        )

        self.observations_pub = self.create_publisher(
            LandingPadArray,
            str(self.get_parameter("observations_topic").value),
            10,
        )

        if self.publish_debug_images:
            debug_prefix = str(self.get_parameter("debug_image_topic_prefix").value).rstrip("/")
            self.red_mask_pub = self.create_publisher(
                Image, f"{debug_prefix}/drone{self.drone_id}/red_mask", qos_profile
            )
            self.blue_mask_pub = self.create_publisher(
                Image, f"{debug_prefix}/drone{self.drone_id}/blue_mask", qos_profile
            )

        self.get_logger().info(
            "Swarm pad detector ready for drone %d on %s"
            % (self.drone_id, str(self.get_parameter("camera_topic").value))
        )

    @staticmethod
    def _camera_optical_to_body_frd(point_optical: np.ndarray) -> np.ndarray:
        # x_opt -> body right, y_opt -> body backward, z_opt -> body down.
        return np.array(
            [-point_optical[1], point_optical[0], point_optical[2]],
            dtype=float,
        )

    def _resolve_thresholds_path(self, thresholds_file_param: str) -> str:
        if os.path.isabs(thresholds_file_param):
            return thresholds_file_param
        share_dir = get_package_share_directory("skyw_detection")
        return os.path.join(share_dir, thresholds_file_param)

    def on_position(self, msg: VehicleLocalPosition) -> None:
        self.last_position = msg

    def on_attitude(self, msg: VehicleAttitude) -> None:
        self.last_attitude = msg

    def on_image(self, msg: Image) -> None:
        if self.last_position is None or self.last_attitude is None:
            return

        now_s = time.time()
        if self.min_process_period_sec > 0.0:
            if (now_s - self.last_process_time) < self.min_process_period_sec:
                return
            self.last_process_time = now_s

        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().warn(f"Drone {self.drone_id}: failed to convert image: {exc}")
            return

        height, width = bgr.shape[:2]
        if self.cx < 0.0:
            self.cx = (float(width) - 1.0) / 2.0
        if self.cy < 0.0:
            self.cy = (float(height) - 1.0) / 2.0

        detections, debug_masks = self.detector.detect_pads(
            bgr, return_masks=self.publish_debug_images
        )
        annotated = bgr.copy() if self.enable_visualization else None

        observations = LandingPadArray()
        observations.header.stamp = self.get_clock().now().to_msg()
        observations.header.frame_id = "map"

        if detections:
            position_ned = np.array(
                [
                    float(self.last_position.x),
                    float(self.last_position.y),
                    float(self.last_position.z),
                ],
                dtype=float,
            )
            q = self.last_attitude.q
            body_to_ned = rotation_matrix_from_quaternion_wxyz(
                float(q[0]), float(q[1]), float(q[2]), float(q[3])
            )
            camera_origin_ned = position_ned + body_to_ned @ self.camera_translation_body_frd

            for detection in detections:
                if annotated is not None:
                    self._draw_detection_overlay(annotated, detection)

                ray_optical = pixel_to_camera_ray(
                    float(detection.centroid_px[0]),
                    float(detection.centroid_px[1]),
                    self.fx,
                    self.fy,
                    self.cx,
                    self.cy,
                )
                ray_body = self._camera_optical_to_body_frd(ray_optical)
                ray_body /= max(np.linalg.norm(ray_body), 1e-6)
                ray_ned = body_to_ned @ ray_body
                hit_ned = intersect_ray_with_horizontal_plane(
                    camera_origin_ned,
                    ray_ned,
                    plane_z=self.ground_plane_ned_z,
                )
                if hit_ned is None:
                    continue

                hit_world = ned_to_world_xyz(hit_ned, self.spawn_xy)
                observation = LandingPad()
                observation.color = detection.color
                observation.pose.position.x = float(hit_world[0])
                observation.pose.position.y = float(hit_world[1])
                observation.pose.position.z = float(self.ground_plane_world_z)
                observation.pose.orientation.w = 1.0
                observation.confidence = float(detection.confidence)
                observation.detection_count = 1
                observation.source_drone_id = self.drone_id
                observations.pads.append(observation)

        self.observations_pub.publish(observations)

        if annotated is not None:
            display_image = self._resize_for_visualization(annotated)
            cv2.imshow(self.visualization_window_name, display_image)
            cv2.waitKey(1)

        if self.publish_debug_images:
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

    def _draw_detection_overlay(self, image: np.ndarray, detection) -> None:
        color_bgr = (0, 0, 255) if detection.color == "red" else (255, 0, 0)
        x, y, w, h = detection.bbox_xywh
        cx = int(round(detection.centroid_px[0]))
        cy = int(round(detection.centroid_px[1]))
        radius = max(int(round(detection.radius_px)), 4)

        cv2.rectangle(image, (x, y), (x + w, y + h), color_bgr, 2)
        cv2.circle(image, (cx, cy), radius, color_bgr, 2)
        cv2.circle(image, (cx, cy), 3, (0, 255, 255), -1)
        label = f"{detection.color} conf={detection.confidence:.2f}"
        cv2.putText(
            image,
            label,
            (x, max(20, y - 8)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            color_bgr,
            2,
        )

    def _resize_for_visualization(self, image: np.ndarray) -> np.ndarray:
        if abs(self.visualization_scale - 1.0) < 1e-6:
            return image
        width = max(1, int(round(image.shape[1] * self.visualization_scale)))
        height = max(1, int(round(image.shape[0] * self.visualization_scale)))
        return cv2.resize(image, (width, height), interpolation=cv2.INTER_AREA)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = SwarmPadDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if node.enable_visualization:
                cv2.destroyWindow(node.visualization_window_name)
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
