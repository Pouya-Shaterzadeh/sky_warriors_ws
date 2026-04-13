#!/usr/bin/env python3

import itertools
import json
import math
from collections import deque

import rclpy
from geometry_msgs.msg import PoseStamped, Quaternion
from px4_msgs.msg import VehicleCommand
from rclpy.node import Node
from skyw_interfaces.msg import LandingPadArray
from std_msgs.msg import String

try:
    from skyw_swarm import formation_math
except ImportError:
    import formation_math


def yaw_to_quaternion(yaw):
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


class MissionSequencer(Node):
    def __init__(self):
        super().__init__("mission_sequencer")

        self.declare_parameter("drone_count", 3)
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("transit_z", -6.0)
        self.declare_parameter("wall_z", -2.0)
        self.declare_parameter("transit_tolerance_m", 0.5)
        self.declare_parameter("safety_offset_m", 2.0)
        self.declare_parameter("landing_pad_topic", "/skyw_detection/pad_registry")
        self.declare_parameter("pad_search_spacing_m", 3.0)
        self.declare_parameter("landing_zone_radius_m", 1.5)
        self.declare_parameter("landing_slot_radius_m", 0.75)
        self.declare_parameter("landing_stage_radius_m", 2.25)
        self.declare_parameter("landing_min_separation_m", 1.0)
        self.declare_parameter("landing_slot_tolerance_m", 0.50)
        self.declare_parameter("landing_pad_lock_confidence_min", 0.60)
        self.declare_parameter("landing_pad_lock_jitter_max_m", 0.40)
        self.declare_parameter("landing_pad_lock_dwell_s", 2.0)
        self.declare_parameter("landing_retry_backoff_s", 2.0)
        self.declare_parameter("landing_pad_search_z", -5.0)
        self.declare_parameter("landing_pad_search_timeout_s", 30.0)
        self.declare_parameter("landing_pad_fallback_max_age_s", 60.0)
        self.declare_parameter("landing_pad_fallback_confidence_min", 0.40)
        self.declare_parameter("landing_pad_fallback_min_detections", 2)
        self.declare_parameter("landing_pad_search_log_period_s", 1.0)
        self.declare_parameter("landing_abort_on_missing_lock", True)
        self.declare_parameter("landing_pose_timeout_s", 0.75)
        self.declare_parameter("landing_slot_hover_z", -2.0)
        self.declare_parameter("landing_sync_release_z", -0.75)
        self.declare_parameter("landing_sync_duration_s", 3.0)
        self.declare_parameter("landing_release_tolerance_m", 0.60)
        self.declare_parameter("landing_complete_z_m", -0.15)

        self.drone_count = int(self.get_parameter("drone_count").value)
        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.transit_z = float(self.get_parameter("transit_z").value)
        self.wall_z = float(self.get_parameter("wall_z").value)
        self.transit_tolerance_m = float(self.get_parameter("transit_tolerance_m").value)
        self.safety_offset_m = float(self.get_parameter("safety_offset_m").value)
        self.pad_search_spacing_m = float(self.get_parameter("pad_search_spacing_m").value)
        self.landing_zone_radius_m = float(self.get_parameter("landing_zone_radius_m").value)
        self.landing_slot_radius_m = float(self.get_parameter("landing_slot_radius_m").value)
        self.landing_stage_radius_m = float(self.get_parameter("landing_stage_radius_m").value)
        self.landing_min_separation_m = float(
            self.get_parameter("landing_min_separation_m").value
        )
        self.landing_slot_tolerance_m = float(
            self.get_parameter("landing_slot_tolerance_m").value
        )
        self.landing_pad_lock_confidence_min = float(
            self.get_parameter("landing_pad_lock_confidence_min").value
        )
        self.landing_pad_lock_jitter_max_m = float(
            self.get_parameter("landing_pad_lock_jitter_max_m").value
        )
        self.landing_pad_lock_dwell_s = float(
            self.get_parameter("landing_pad_lock_dwell_s").value
        )
        self.landing_retry_backoff_s = float(
            self.get_parameter("landing_retry_backoff_s").value
        )
        self.landing_pad_search_z = float(
            self.get_parameter("landing_pad_search_z").value
        )
        self.landing_pad_search_timeout_s = float(
            self.get_parameter("landing_pad_search_timeout_s").value
        )
        self.landing_pad_fallback_max_age_s = float(
            self.get_parameter("landing_pad_fallback_max_age_s").value
        )
        self.landing_pad_fallback_confidence_min = float(
            self.get_parameter("landing_pad_fallback_confidence_min").value
        )
        self.landing_pad_fallback_min_detections = int(
            self.get_parameter("landing_pad_fallback_min_detections").value
        )
        self.landing_pad_search_log_period_s = float(
            self.get_parameter("landing_pad_search_log_period_s").value
        )
        self.landing_abort_on_missing_lock = bool(
            self.get_parameter("landing_abort_on_missing_lock").value
        )
        self.landing_pose_timeout_s = float(
            self.get_parameter("landing_pose_timeout_s").value
        )
        self.landing_slot_hover_z = float(self.get_parameter("landing_slot_hover_z").value)
        self.landing_sync_release_z = float(
            self.get_parameter("landing_sync_release_z").value
        )
        self.landing_sync_duration_s = float(
            self.get_parameter("landing_sync_duration_s").value
        )
        self.landing_release_tolerance_m = float(
            self.get_parameter("landing_release_tolerance_m").value
        )
        self.landing_complete_z_m = float(
            self.get_parameter("landing_complete_z_m").value
        )

        self.takeoff_timer_start = None

        self.gz_spawns = {
            1: {"x": -7.0, "y": 5.0},
            2: {"x": -7.0, "y": 4.0},
            3: {"x": -7.0, "y": 6.0},
        }

        self.wall_waypoints = [
            {"x": 6.50, "y": 0.00, "gz_yaw": 0.000},
            {"x": 3.25, "y": 5.629, "gz_yaw": 1.047},
            {"x": -3.25, "y": 5.629, "gz_yaw": 2.094},
            {"x": -6.50, "y": 0.00, "gz_yaw": 3.142},
            {"x": -3.25, "y": -5.629, "gz_yaw": -2.094},
            {"x": 3.25, "y": -5.629, "gz_yaw": -1.047},
        ]
        self.current_wp_idx = 0

        self.d1_target_x = 0.0
        self.d1_target_y = 0.0
        self.d1_target_z = self.transit_z
        self.d1_target_yaw = 0.0
        self.overwatch_gz_x = 0.0
        self.overwatch_gz_y = 0.0

        self.current_spacing = 2.0
        self.target_formation = "v"
        self.active_formation = "column"
        self.payload_h = 0.0
        self.next_wall_ovr = None

        self.pending_payload = None
        self.landing_pad_color = None
        self.detected_pad_world = {}
        self.pad_track_history = {
            "red": deque(maxlen=120),
            "blue": deque(maxlen=120),
        }
        self.final_pad_locked_xy = None
        self.final_pad_lock_confidence = 0.0
        self.final_stage_targets_world = {}
        self.final_slot_targets_world = {}
        self.land_commands_sent = False
        self.last_pad_search_log_s = 0.0

        self.setpoint_pubs = {}
        self.land_command_pubs = {}
        self.last_pose = {}
        self.last_pose_time = {}
        for i in range(1, self.drone_count + 1):
            self.setpoint_pubs[i] = self.create_publisher(
                PoseStamped, f"/drone{i}/setpoint_position", 10
            )
            self.land_command_pubs[i] = self.create_publisher(
                VehicleCommand, f"/px4_{i}/fmu/in/vehicle_command", 10
            )
            self.create_subscription(
                PoseStamped, f"/drone{i}/pose", lambda msg, idx=i: self._pose_cb(msg, idx), 10
            )

        self.create_subscription(String, "/qr_decoded", self._qr_cb, 10)
        self.create_subscription(
            LandingPadArray,
            str(self.get_parameter("landing_pad_topic").value),
            self._pad_registry_cb,
            10,
        )

        self.state = "TAKEOFF"
        self.state_started = self.get_clock().now()
        self.qr_seen = False

        period = 1.0 / self.publish_rate_hz if self.publish_rate_hz > 0.0 else 0.05
        self.timer = self.create_timer(period, self._tick)
        self.get_logger().info("Mission sequencer started! Final-Station Hover mode active.")

    def _now_seconds(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _local_to_global(self, idx: int, px4_x: float, px4_y: float):
        gz_x = px4_y + self.gz_spawns[idx]["x"]
        gz_y = px4_x + self.gz_spawns[idx]["y"]
        return gz_x, gz_y

    def _global_to_local(self, idx: int, gz_x: float, gz_y: float):
        px4_x = gz_y - self.gz_spawns[idx]["y"]
        px4_y = gz_x - self.gz_spawns[idx]["x"]
        return px4_x, px4_y

    def _qr_cb(self, msg: String):
        if self.state == "MISSION_DONE":
            return
        try:
            self.pending_payload = json.loads(msg.data.strip())
            self.qr_seen = True
        except Exception:
            self.qr_seen = True

    def _apply_buffered_payload(self):
        if not self.pending_payload:
            return

        config = self.pending_payload
        self.current_spacing = float(config.get("s", self.current_spacing))
        self.target_formation = str(config.get("f", self.target_formation)).lower()
        self.payload_h = float(config.get("h", 0.0))

        next_n = config.get("n")
        if next_n is not None:
            try:
                self.next_wall_ovr = int(next_n)
            except (ValueError, TypeError):
                self.next_wall_ovr = None

        new_a = config.get("a")
        if new_a is not None:
            self.d1_target_z = -abs(float(new_a))

        gorev = config.get("gorev", {})
        form = gorev.get("formasyon", {})
        if form.get("aktif"):
            self.current_spacing = float(form.get("spacing", self.current_spacing))
            self.target_formation = str(form.get("tip", self.target_formation)).lower()

        irtifa = gorev.get("irtifa_degisim", {})
        if irtifa.get("aktif"):
            self.d1_target_z = -abs(float(irtifa.get("deger", abs(self.d1_target_z))))

        landing_color = config.get("l", config.get("landing_color"))
        if landing_color is None:
            landing_color = gorev.get("landing_color")
        if landing_color is None:
            landing_color = gorev.get("inis_pisti", {}).get("renk")
        if landing_color is not None:
            landing_color = str(landing_color).strip().lower()
            if landing_color in {"red", "blue"}:
                self.landing_pad_color = landing_color
                self.get_logger().info(
                    f"Final landing pad color set to: {self.landing_pad_color}"
                )

        self.active_formation = self.target_formation
        self.pending_payload = None

    def _tick(self):
        now = self.get_clock().now()
        elapsed = (now - self.state_started).nanoseconds / 1e9

        if self.state == "TAKEOFF":
            all_ready = True
            for i in range(1, self.drone_count + 1):
                self._publish_setpoint(i, 0.0, 0.0, self.transit_z, 0.0)
                pose = self.last_pose.get(i)
                if pose is None or abs(pose.pose.position.z - self.transit_z) > self.transit_tolerance_m:
                    all_ready = False

            if all_ready:
                if self.takeoff_timer_start is None:
                    self.takeoff_timer_start = elapsed
                elif elapsed - self.takeoff_timer_start >= 3.0:
                    self._set_state("TRANSIT_TO_HUB")
            return

        elif self.state == "TRANSIT_TO_HUB":
            self.active_formation = "column"
            self.d1_target_z = self.transit_z
            tx, ty = self._global_to_local(1, 0.0, 0.0)
            self.d1_target_x, self.d1_target_y = tx, ty
            if self._drone_near_target(1, tx, ty, self.transit_z):
                self._update_wall_targets(safety_gap=True)
                self._set_state("TRANSIT_HIGH")

        elif self.state == "TRANSIT_HIGH":
            if self._drone_near_target(1, self.d1_target_x, self.d1_target_y, self.transit_z):
                self._set_state("STABILIZE_HIGH")

        elif self.state == "STABILIZE_HIGH":
            anchor_px4_x, anchor_px4_y = self.d1_target_x, self.d1_target_y
            self.overwatch_gz_x, self.overwatch_gz_y = self._local_to_global(
                1, anchor_px4_x, anchor_px4_y
            )
            if elapsed >= 3.0:
                self.d1_target_z = self.wall_z
                self._set_state("DESCEND_VERTICAL")

        elif self.state == "DESCEND_VERTICAL":
            if self._drone_near_target(1, self.d1_target_x, self.d1_target_y, self.wall_z):
                self._update_wall_targets(safety_gap=False)
                self._set_state("NUDGE_TO_SCAN")

        elif self.state == "NUDGE_TO_SCAN":
            if self._drone_near_target(1, self.d1_target_x, self.d1_target_y, self.wall_z):
                self._set_state("HOLD_AND_SCAN")

        elif self.state == "HOLD_AND_SCAN":
            if self.qr_seen:
                self._apply_buffered_payload()
                self._set_state("PERFORM_TASK")
            elif elapsed >= 10.0:
                self._prepare_next_wall()

        elif self.state == "PERFORM_TASK":
            if elapsed >= self.payload_h:
                self._prepare_next_wall()

        elif self.state == "RETURN_TO_HUB_FOR_PAD_SEARCH":
            self.active_formation = "line"
            self.current_spacing = self.pad_search_spacing_m
            tx, ty = self._global_to_local(1, 0.0, 0.0)
            self.d1_target_x, self.d1_target_y = tx, ty
            self.d1_target_z = self.transit_z
            self.d1_target_yaw = math.pi / 2.0
            if self._drone_near_target(1, tx, ty, self.transit_z):
                self._set_state("PAD_SEARCH_HOVER")

        elif self.state == "PAD_SEARCH_HOVER":
            tx, ty = self._global_to_local(1, 0.0, 0.0)
            self.d1_target_x, self.d1_target_y = tx, ty
            self.d1_target_z = self.landing_pad_search_z
            self.d1_target_yaw = math.pi / 2.0
            if self._try_lock_selected_pad():
                self._build_final_landing_plan()
                self._set_state("FINAL_STAGE_RING")
            elif elapsed >= self.landing_pad_search_timeout_s:
                if self._try_fallback_pad_lock():
                    self._build_final_landing_plan()
                    self._set_state("FINAL_STAGE_RING")
                elif self.landing_abort_on_missing_lock:
                    self.get_logger().warn(
                        "Final landing aborted: selected pad could not be locked live or from fallback track."
                    )
                    self._set_state("FINAL_LANDING_ABORTED")

        elif self.state == "FINAL_STAGE_RING":
            if self._all_drones_ready_for_targets(
                self.final_stage_targets_world,
                self.transit_z,
                self.transit_tolerance_m,
            ):
                self._set_state("FINAL_STAGE_SLOTS")

        elif self.state == "FINAL_STAGE_SLOTS":
            if not self._current_swarm_safe():
                self._start_final_retry("unsafe swarm geometry while committing slot approach")
            elif self._all_drones_ready_for_targets(
                self.final_slot_targets_world,
                self.landing_slot_hover_z,
                self.landing_slot_tolerance_m,
            ) and self._descent_clear_to_start():
                self._set_state("FINAL_DESCEND_SYNC")

        elif self.state == "FINAL_DESCEND_SYNC":
            if not self._descent_safety_ok():
                self._start_final_retry("sync descent safety invariant violated")
            elif self._sync_descent_complete(elapsed):
                self._send_land_commands()
                self._set_state("LANDING_COMPLETE")

        elif self.state == "FINAL_RETRY_HOLD":
            if (
                elapsed >= self.landing_retry_backoff_s
                and self._all_drones_ready_for_targets(
                    self.final_stage_targets_world,
                    self.transit_z,
                    self.transit_tolerance_m,
                )
                and self._current_swarm_safe()
            ):
                self._set_state("FINAL_STAGE_SLOTS")

        elif self.state == "LANDING_COMPLETE":
            if self._all_drones_landed():
                self._set_state("MISSION_DONE")

        if self.state in {"MISSION_DONE", "LANDING_COMPLETE"}:
            return

        if self.state in {"PAD_SEARCH_HOVER", "FINAL_LANDING_ABORTED"}:
            self._publish_pad_search_setpoints()
            return

        if self.state in {
            "FINAL_STAGE_RING",
            "FINAL_STAGE_SLOTS",
            "FINAL_DESCEND_SYNC",
            "FINAL_RETRY_HOLD",
        }:
            self._publish_final_landing_setpoints(elapsed)
            return

        l_x, l_y, l_z, l_yaw = (
            self.d1_target_x,
            self.d1_target_y,
            self.d1_target_z,
            self.d1_target_yaw,
        )
        l_gz_x, l_gz_y = self._local_to_global(1, l_x, l_y)
        self._publish_setpoint(1, l_x, l_y, l_z, l_yaw)

        is_overwatch = self.state in ["DESCEND_VERTICAL", "NUDGE_TO_SCAN", "HOLD_AND_SCAN"]
        anchor_x = self.overwatch_gz_x if is_overwatch else l_gz_x
        anchor_y = self.overwatch_gz_y if is_overwatch else l_gz_y

        current_form = (
            "column"
            if self.state in ["TRANSIT_HIGH", "TRANSIT_TO_HUB", "STABILIZE_HIGH"]
            else self.active_formation
        )
        if is_overwatch:
            current_form = "column"

        offsets = formation_math.get_offsets(current_form, self.current_spacing, self.drone_count)
        for i in range(2, self.drone_count + 1):
            if self.state == "TAKEOFF":
                self._publish_setpoint(i, 0.0, 0.0, self.transit_z, 0.0)
                continue

            f_x, f_y = offsets[i - 1]
            t_gz_x = anchor_x + f_x * math.cos(l_yaw) - f_y * math.sin(l_yaw)
            t_gz_y = anchor_y + f_x * math.sin(l_yaw) + f_y * math.cos(l_yaw)
            t_px4_x, t_px4_y = self._global_to_local(i, t_gz_x, t_gz_y)
            follower_z = self.transit_z if is_overwatch else l_z
            self._publish_setpoint(i, t_px4_x, t_px4_y, follower_z, l_yaw)

    def _prepare_next_wall(self):
        if self.next_wall_ovr is not None:
            self.current_wp_idx = int(self.next_wall_ovr) - 1
            self.next_wall_ovr = None
        else:
            self.current_wp_idx += 1

        if self.current_wp_idx < len(self.wall_waypoints):
            self.qr_seen = False
            self.payload_h = 0.0
            self._set_state("TRANSIT_TO_HUB")
            return

        if self.landing_pad_color in {"red", "blue"}:
            self.get_logger().info(
                f"Swarm journey complete. Regrouping for final landing on {self.landing_pad_color}."
            )
            self._reset_final_landing_plan()
            self._set_state("RETURN_TO_HUB_FOR_PAD_SEARCH")
        else:
            self.get_logger().info("Swarm journey complete. Keeping final station position.")
            self._set_state("MISSION_DONE")

    def _drone_near_target(self, idx, x, y, z, tolerance=None):
        pose = self.last_pose.get(idx)
        if pose is None:
            return False
        tol = self.transit_tolerance_m if tolerance is None else float(tolerance)
        dist = math.sqrt(
            (pose.pose.position.x - x) ** 2
            + (pose.pose.position.y - y) ** 2
            + (pose.pose.position.z - z) ** 2
        )
        return dist <= tol

    def _set_state(self, new_state):
        if new_state != self.state:
            self.state = new_state
            self.state_started = self.get_clock().now()
            self.get_logger().info(f"State -> {new_state}")

    def _update_wall_targets(self, safety_gap=True):
        wp = self.wall_waypoints[self.current_wp_idx]
        gz_x, gz_y, gz_yaw = wp["x"], wp["y"], wp["gz_yaw"]

        if safety_gap:
            gz_x -= self.safety_offset_m * math.cos(gz_yaw)
            gz_y -= self.safety_offset_m * math.sin(gz_yaw)

        px4_x, px4_y = self._global_to_local(1, gz_x, gz_y)
        self.d1_target_x, self.d1_target_y = px4_x, px4_y
        self.d1_target_yaw = (math.pi / 2.0) - gz_yaw
        self.d1_target_z = self.transit_z if safety_gap else self.wall_z

    def _pose_cb(self, msg, idx):
        self.last_pose[idx] = msg
        self.last_pose_time[idx] = self._now_seconds()

    def _pad_registry_cb(self, msg):
        now_s = self._now_seconds()
        for pad in msg.pads:
            color = str(pad.color).strip().lower()
            if color not in {"red", "blue"}:
                continue

            x = float(pad.pose.position.x)
            y = float(pad.pose.position.y)
            confidence = float(pad.confidence)
            self.detected_pad_world[color] = {
                "x": x,
                "y": y,
                "confidence": confidence,
                "detection_count": int(pad.detection_count),
                "last_seen_sec": now_s,
            }

            history = self.pad_track_history[color]
            history.append((now_s, x, y, confidence))
            while history and (now_s - history[0][0]) > max(self.landing_pad_lock_dwell_s * 2.0, 4.0):
                history.popleft()

    def _selected_pad_world_xy(self):
        if self.final_pad_locked_xy is not None:
            return self.final_pad_locked_xy
        if self.landing_pad_color not in self.detected_pad_world:
            return None
        selected = self.detected_pad_world[self.landing_pad_color]
        return (float(selected["x"]), float(selected["y"]))

    def _pad_search_targets_world(self):
        return {
            1: (0.0, 0.0),
            2: (3.0, 0.0),
            3: (-3.0, 0.0),
        }

    def _log_pad_search_status(self, reason):
        now_s = self._now_seconds()
        if (now_s - self.last_pad_search_log_s) < self.landing_pad_search_log_period_s:
            return
        self.last_pad_search_log_s = now_s
        if self.landing_pad_color in {"red", "blue"}:
            self.get_logger().info(
                f"Waiting for {self.landing_pad_color} pad lock: {reason}"
            )

    def _live_pad_lock_status(self):
        if self.final_pad_locked_xy is not None:
            return True, "already locked"
        if self.landing_pad_color not in {"red", "blue"}:
            return False, "no selected landing color"

        now_s = self._now_seconds()
        history = list(self.pad_track_history.get(self.landing_pad_color, ()))
        if not history:
            return False, "missing selected-color history"

        window = [
            sample
            for sample in history
            if (now_s - sample[0]) <= self.landing_pad_lock_dwell_s
        ]
        if len(window) < 2:
            return False, "insufficient dwell duration"
        if (window[-1][0] - window[0][0]) < (self.landing_pad_lock_dwell_s * 0.5):
            return False, "insufficient dwell duration"

        min_confidence = min(sample[3] for sample in window)
        if min_confidence < self.landing_pad_lock_confidence_min:
            return False, f"insufficient confidence ({min_confidence:.2f})"

        mean_x = sum(sample[1] for sample in window) / len(window)
        mean_y = sum(sample[2] for sample in window) / len(window)
        max_jitter = max(
            math.hypot(sample[1] - mean_x, sample[2] - mean_y) for sample in window
        )
        if max_jitter > self.landing_pad_lock_jitter_max_m:
            return False, f"excessive jitter ({max_jitter:.2f} m)"

        self.final_pad_locked_xy = (mean_x, mean_y)
        self.final_pad_lock_confidence = sum(sample[3] for sample in window) / len(window)
        self.get_logger().info(
            "Locked %s pad at (%.2f, %.2f), conf=%.2f, jitter=%.2f m"
            % (
                self.landing_pad_color,
                mean_x,
                mean_y,
                self.final_pad_lock_confidence,
                max_jitter,
            )
        )
        return True, "locked"

    def _try_lock_selected_pad(self):
        success, reason = self._live_pad_lock_status()
        if not success:
            self._log_pad_search_status(reason)
        return success

    def _try_fallback_pad_lock(self):
        if self.final_pad_locked_xy is not None:
            return True
        if self.landing_pad_color not in {"red", "blue"}:
            return False

        selected = self.detected_pad_world.get(self.landing_pad_color)
        if not selected:
            self._log_pad_search_status("no fallback selected-color track")
            return False

        now_s = self._now_seconds()
        age_s = now_s - float(selected["last_seen_sec"])
        confidence = float(selected["confidence"])
        detection_count = int(selected["detection_count"])
        if age_s > self.landing_pad_fallback_max_age_s:
            self._log_pad_search_status(f"fallback track stale ({age_s:.1f} s)")
            return False
        if confidence < self.landing_pad_fallback_confidence_min:
            self._log_pad_search_status(f"fallback confidence too low ({confidence:.2f})")
            return False
        if detection_count < self.landing_pad_fallback_min_detections:
            self._log_pad_search_status(
                f"fallback detections too low ({detection_count})"
            )
            return False

        self.final_pad_locked_xy = (float(selected["x"]), float(selected["y"]))
        self.final_pad_lock_confidence = confidence
        self.get_logger().warn(
            "Using fallback %s pad lock at (%.2f, %.2f), conf=%.2f, age=%.1f s"
            % (
                self.landing_pad_color,
                self.final_pad_locked_xy[0],
                self.final_pad_locked_xy[1],
                confidence,
                age_s,
            )
        )
        return True

    def _reset_final_landing_plan(self):
        self.final_pad_locked_xy = None
        self.final_pad_lock_confidence = 0.0
        self.final_stage_targets_world = {}
        self.final_slot_targets_world = {}
        self.land_commands_sent = False
        self.last_pad_search_log_s = 0.0

    def _build_final_landing_plan(self):
        selected_pad = self._selected_pad_world_xy()
        if selected_pad is None:
            return False

        if self.landing_slot_radius_m > self.landing_zone_radius_m:
            raise ValueError("landing_slot_radius_m must be inside landing_zone_radius_m")

        center_x, center_y = float(selected_pad[0]), float(selected_pad[1])
        stage_candidates = self._generate_ring_targets(
            center_x, center_y, self.landing_stage_radius_m
        )
        slot_candidates = self._generate_ring_targets(
            center_x, center_y, self.landing_slot_radius_m
        )
        assignment = self._assign_candidate_indices_to_drones(stage_candidates)
        self.final_stage_targets_world = {
            idx: stage_candidates[position_idx] for idx, position_idx in assignment.items()
        }
        self.final_slot_targets_world = {
            idx: slot_candidates[position_idx] for idx, position_idx in assignment.items()
        }

        if not self._target_geometry_safe(self.final_slot_targets_world):
            raise ValueError("final slot geometry violates minimum separation")

        self.get_logger().info(
            "Final landing plan ready: ring=%.2f m, slot=%.2f m, min_sep=%.2f m"
            % (
                self.landing_stage_radius_m,
                self.landing_slot_radius_m,
                self.landing_min_separation_m,
            )
        )
        return True

    def _generate_ring_targets(self, center_x, center_y, radius):
        targets = []
        angle_offset = math.pi / 2.0
        for index in range(self.drone_count):
            angle = angle_offset + (2.0 * math.pi * index / float(self.drone_count))
            targets.append(
                (
                    center_x + radius * math.cos(angle),
                    center_y + radius * math.sin(angle),
                )
            )
        return targets

    def _drone_world_xy(self, idx):
        pose = self.last_pose.get(idx)
        if pose is None:
            return None
        return self._local_to_global(idx, pose.pose.position.x, pose.pose.position.y)

    def _assign_candidate_indices_to_drones(self, candidates):
        drone_ids = list(range(1, self.drone_count + 1))
        best_assignment = {}
        best_cost = None

        candidate_indices = list(range(len(candidates)))
        for perm in itertools.permutations(candidate_indices, len(drone_ids)):
            cost = 0.0
            assignment = {}
            for idx, candidate_index in zip(drone_ids, perm):
                target = candidates[candidate_index]
                current = self._drone_world_xy(idx)
                if current is None:
                    current = (self.gz_spawns[idx]["x"], self.gz_spawns[idx]["y"])
                cost += math.dist(current, target)
                assignment[idx] = candidate_index

            if best_cost is None or cost < best_cost:
                best_cost = cost
                best_assignment = assignment

        return best_assignment

    def _target_geometry_safe(self, targets_world):
        targets = list(targets_world.values())
        for i in range(len(targets)):
            for j in range(i + 1, len(targets)):
                if math.dist(targets[i], targets[j]) < self.landing_min_separation_m:
                    return False
        if self.final_pad_locked_xy is None:
            return True
        for target_x, target_y in targets:
            if math.hypot(
                target_x - self.final_pad_locked_xy[0],
                target_y - self.final_pad_locked_xy[1],
            ) > self.landing_zone_radius_m + 1e-6:
                return False
        return True

    def _poses_fresh(self):
        now_s = self._now_seconds()
        for idx in range(1, self.drone_count + 1):
            if idx not in self.last_pose_time:
                return False
            if (now_s - self.last_pose_time[idx]) > self.landing_pose_timeout_s:
                return False
        return True

    def _current_swarm_safe(self):
        if not self._poses_fresh():
            return False
        world_positions = []
        for idx in range(1, self.drone_count + 1):
            current = self._drone_world_xy(idx)
            if current is None:
                return False
            world_positions.append(current)

        for i in range(len(world_positions)):
            for j in range(i + 1, len(world_positions)):
                if math.dist(world_positions[i], world_positions[j]) < self.landing_min_separation_m:
                    return False
        return True

    def _all_drones_ready_for_targets(self, targets_world, target_z, tolerance):
        if len(targets_world) != self.drone_count or not self._poses_fresh():
            return False

        for idx in range(1, self.drone_count + 1):
            target = targets_world.get(idx)
            if target is None:
                return False
            target_px4_x, target_px4_y = self._global_to_local(idx, target[0], target[1])
            if not self._drone_near_target(idx, target_px4_x, target_px4_y, target_z, tolerance):
                return False
        return True

    def _descent_clear_to_start(self):
        return self._target_geometry_safe(self.final_slot_targets_world) and self._current_swarm_safe()

    def _current_sync_descent_z(self, elapsed):
        if self.state != "FINAL_DESCEND_SYNC" or self.landing_sync_duration_s <= 0.0:
            return self.landing_slot_hover_z
        progress = max(0.0, min(1.0, elapsed / self.landing_sync_duration_s))
        return self.landing_slot_hover_z + progress * (
            self.landing_sync_release_z - self.landing_slot_hover_z
        )

    def _descent_safety_ok(self):
        if not self._current_swarm_safe():
            return False
        current_z = self._current_sync_descent_z(
            (self.get_clock().now() - self.state_started).nanoseconds / 1e9
        )
        return self._all_drones_ready_for_targets(
            self.final_slot_targets_world,
            current_z,
            max(self.landing_slot_tolerance_m, self.landing_release_tolerance_m),
        )

    def _sync_descent_complete(self, elapsed):
        if elapsed < self.landing_sync_duration_s:
            return False
        return self._all_drones_ready_for_targets(
            self.final_slot_targets_world,
            self.landing_sync_release_z,
            self.landing_release_tolerance_m,
        )

    def _start_final_retry(self, reason):
        self.get_logger().warn(f"Final landing retry: {reason}")
        self.land_commands_sent = False
        self._set_state("FINAL_RETRY_HOLD")

    def _publish_final_landing_setpoints(self, elapsed):
        yaw = self.d1_target_yaw
        if self.state in {"FINAL_STAGE_RING", "FINAL_RETRY_HOLD"}:
            targets = self.final_stage_targets_world
            target_z = self.transit_z
        elif self.state == "FINAL_STAGE_SLOTS":
            targets = self.final_slot_targets_world
            target_z = self.landing_slot_hover_z
        else:
            targets = self.final_slot_targets_world
            target_z = self._current_sync_descent_z(elapsed)

        for idx in range(1, self.drone_count + 1):
            target = targets.get(idx)
            if target is None:
                continue
            target_px4_x, target_px4_y = self._global_to_local(idx, target[0], target[1])
            self._publish_setpoint(idx, target_px4_x, target_px4_y, target_z, yaw)

    def _publish_pad_search_setpoints(self):
        yaw = math.pi / 2.0
        targets = self._pad_search_targets_world()
        for idx in range(1, self.drone_count + 1):
            target = targets.get(idx)
            if target is None:
                continue
            target_px4_x, target_px4_y = self._global_to_local(idx, target[0], target[1])
            self._publish_setpoint(
                idx, target_px4_x, target_px4_y, self.landing_pad_search_z, yaw
            )

    def _send_land_commands(self):
        if self.land_commands_sent:
            return
        timestamp = int(self.get_clock().now().nanoseconds / 1000)
        for idx in range(1, self.drone_count + 1):
            msg = VehicleCommand()
            msg.timestamp = timestamp
            msg.command = int(VehicleCommand.VEHICLE_CMD_NAV_LAND)
            msg.target_system = 0
            msg.target_component = 1
            msg.source_system = 255
            msg.source_component = 1
            msg.from_external = True
            self.land_command_pubs[idx].publish(msg)
        self.land_commands_sent = True
        self.get_logger().info("Issued synchronized PX4 land commands for all drones.")

    def _all_drones_landed(self):
        if not self._poses_fresh():
            return False
        for idx in range(1, self.drone_count + 1):
            pose = self.last_pose.get(idx)
            if pose is None:
                return False
            if float(pose.pose.position.z) < self.landing_complete_z_m:
                return False
        return True

    def _publish_setpoint(self, idx, x, y, z, yaw):
        m = PoseStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = "map"
        m.pose.position.x = float(x)
        m.pose.position.y = float(y)
        m.pose.position.z = float(z)
        m.pose.orientation = yaw_to_quaternion(yaw)
        self.setpoint_pubs[idx].publish(m)


def main():
    rclpy.init()
    node = MissionSequencer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
