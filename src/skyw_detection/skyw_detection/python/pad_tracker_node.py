#!/usr/bin/env python3

from __future__ import annotations

import json
import os
import time
from dataclasses import dataclass

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, ReliabilityPolicy, QoSProfile
from skyw_interfaces.msg import LandingPad, LandingPadArray
from visualization_msgs.msg import Marker, MarkerArray


@dataclass
class PadTrack:
    color: str
    position_xyz: np.ndarray
    confidence: float
    detection_count: int
    last_seen_sec: float
    source_drone_id: int


class LandingPadTrackerNode(Node):
    """Fuse repeated red/blue pad observations into a shared registry."""

    def __init__(self) -> None:
        super().__init__("landing_pad_tracker_node")

        self.declare_parameter("observations_topic", "/skyw_detection/pad_observations")
        self.declare_parameter("registry_topic", "/skyw_detection/pad_registry")
        self.declare_parameter("markers_topic", "/skyw_detection/pad_markers")
        self.declare_parameter("registry_file_path", "/tmp/skyw_landing_pads.json")
        self.declare_parameter("merge_distance_m", 4.0)
        self.declare_parameter("stale_after_sec", 120.0)
        self.declare_parameter("min_observation_confidence", 0.2)
        self.declare_parameter("publish_rate_hz", 5.0)
        self.declare_parameter("marker_scale_m", 0.45)

        self.merge_distance_m = float(self.get_parameter("merge_distance_m").value)
        self.stale_after_sec = float(self.get_parameter("stale_after_sec").value)
        self.min_observation_confidence = float(
            self.get_parameter("min_observation_confidence").value
        )
        self.marker_scale_m = float(self.get_parameter("marker_scale_m").value)
        self.registry_file_path = str(self.get_parameter("registry_file_path").value).strip()

        self.tracks: dict[str, PadTrack] = {}

        qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.create_subscription(
            LandingPadArray,
            str(self.get_parameter("observations_topic").value),
            self.on_observations,
            qos_profile,
        )
        self.registry_pub = self.create_publisher(
            LandingPadArray,
            str(self.get_parameter("registry_topic").value),
            10,
        )
        self.markers_pub = self.create_publisher(
            MarkerArray,
            str(self.get_parameter("markers_topic").value),
            10,
        )

        publish_rate_hz = max(float(self.get_parameter("publish_rate_hz").value), 0.1)
        self.create_timer(1.0 / publish_rate_hz, self.publish_registry)

        self.get_logger().info("Landing pad tracker ready.")

    def on_observations(self, msg: LandingPadArray) -> None:
        now_s = time.time()
        for pad in msg.pads:
            color = pad.color.strip().lower()
            if color not in {"red", "blue"}:
                continue
            if float(pad.confidence) < self.min_observation_confidence:
                continue

            position_xyz = np.array(
                [
                    float(pad.pose.position.x),
                    float(pad.pose.position.y),
                    float(pad.pose.position.z),
                ],
                dtype=float,
            )

            track = self.tracks.get(color)
            if track is None:
                self.tracks[color] = PadTrack(
                    color=color,
                    position_xyz=position_xyz,
                    confidence=float(pad.confidence),
                    detection_count=max(int(pad.detection_count), 1),
                    last_seen_sec=now_s,
                    source_drone_id=int(pad.source_drone_id),
                )
                continue

            distance_m = float(np.linalg.norm(position_xyz[:2] - track.position_xyz[:2]))
            if distance_m > self.merge_distance_m and track.detection_count >= 3:
                continue

            observation_weight = max(float(pad.confidence) * 2.0, 0.25)
            history_weight = min(max(float(track.detection_count), 1.0), 5.0)
            total_weight = history_weight + observation_weight
            track.position_xyz = (
                track.position_xyz * history_weight + position_xyz * observation_weight
            ) / total_weight
            track.confidence = min(
                1.0,
                max(track.confidence * 0.6 + float(pad.confidence) * 0.4, 0.1),
            )
            track.detection_count += max(int(pad.detection_count), 1)
            track.last_seen_sec = now_s
            track.source_drone_id = int(pad.source_drone_id)

    def publish_registry(self) -> None:
        now_s = time.time()
        msg = LandingPadArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"

        markers = MarkerArray()
        marker_id = 0
        serializable: dict[str, dict[str, float | int]] = {}

        for color in sorted(self.tracks.keys()):
            track = self.tracks[color]
            if (now_s - track.last_seen_sec) > self.stale_after_sec:
                continue

            pad = LandingPad()
            pad.color = track.color
            pad.pose.position.x = float(track.position_xyz[0])
            pad.pose.position.y = float(track.position_xyz[1])
            pad.pose.position.z = float(track.position_xyz[2])
            pad.pose.orientation.w = 1.0
            pad.confidence = float(
                min(1.0, max(track.confidence, min(track.detection_count / 6.0, 1.0)))
            )
            pad.detection_count = int(track.detection_count)
            pad.source_drone_id = int(track.source_drone_id)
            msg.pads.append(pad)

            serializable[color] = {
                "x": float(track.position_xyz[0]),
                "y": float(track.position_xyz[1]),
                "z": float(track.position_xyz[2]),
                "confidence": float(pad.confidence),
                "detection_count": int(track.detection_count),
                "source_drone_id": int(track.source_drone_id),
                "last_seen_sec": float(track.last_seen_sec),
            }

            marker = Marker()
            marker.header = msg.header
            marker.ns = "skyw_landing_pads"
            marker.id = marker_id
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.pose = pad.pose
            marker.scale.x = self.marker_scale_m
            marker.scale.y = self.marker_scale_m
            marker.scale.z = 0.05
            marker.color.a = 0.9
            if color == "red":
                marker.color.r = 1.0
                marker.color.g = 0.15
                marker.color.b = 0.15
            else:
                marker.color.r = 0.15
                marker.color.g = 0.25
                marker.color.b = 1.0
            markers.markers.append(marker)
            marker_id += 1

            label = Marker()
            label.header = msg.header
            label.ns = "skyw_landing_pad_labels"
            label.id = marker_id
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose = pad.pose
            label.pose.position.z += 0.45
            label.scale.z = 0.22
            label.color.a = 1.0
            label.color.r = 1.0
            label.color.g = 1.0
            label.color.b = 1.0
            label.text = f"{color} ({pad.confidence:.2f})"
            markers.markers.append(label)
            marker_id += 1

        self.registry_pub.publish(msg)
        self.markers_pub.publish(markers)
        self._write_registry_file(serializable)

    def _write_registry_file(self, serializable: dict[str, dict[str, float | int]]) -> None:
        if not self.registry_file_path:
            return
        try:
            directory = os.path.dirname(self.registry_file_path)
            if directory:
                os.makedirs(directory, exist_ok=True)
            payload = {
                "updated_at_sec": time.time(),
                "pads": serializable,
            }
            with open(self.registry_file_path, "w", encoding="utf-8") as handle:
                json.dump(payload, handle, indent=2, sort_keys=True)
        except Exception as exc:
            self.get_logger().warn(f"Failed to write pad registry file: {exc}")


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = LandingPadTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
