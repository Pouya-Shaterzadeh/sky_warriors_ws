#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Quaternion
from std_msgs.msg import String


def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.w = math.cos(yaw * 0.5)
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw * 0.5)
    return q


class MissionSequencer(Node):
    """
    High-level mission sequencer for a 3-drone QR scenario.
    - Keeps all drones in offboard by streaming setpoints.
    - Takes off all drones to a shared altitude.
    - Sends drone1 (x500_mono) to wall_1 and holds for QR scan.
    """

    def __init__(self):
        super().__init__('mission_sequencer')

        self.declare_parameter('drone_count', 3)
        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('takeoff_hold_s', 8.0)
        self.declare_parameter('transit_tolerance_m', 0.5)
        self.declare_parameter('target_hold_s', 20.0)

        # PX4 local frame default is NED, so altitude up is negative Z.
        self.declare_parameter('takeoff_z', -2.5)
        self.declare_parameter('wall_x', 5.0)
        self.declare_parameter('wall_y', 0.0)
        self.declare_parameter('wall_z', -1.0)
        self.declare_parameter('wall_yaw', 1.57)

        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', False)

        self.drone_count = int(self.get_parameter('drone_count').value)
        self.publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self.takeoff_hold_s = float(self.get_parameter('takeoff_hold_s').value)
        self.transit_tolerance_m = float(self.get_parameter('transit_tolerance_m').value)
        self.target_hold_s = float(self.get_parameter('target_hold_s').value)
        self.takeoff_z = float(self.get_parameter('takeoff_z').value)
        self.wall_x = float(self.get_parameter('wall_x').value)
        self.wall_y = float(self.get_parameter('wall_y').value)
        self.wall_z = float(self.get_parameter('wall_z').value)
        self.wall_yaw = float(self.get_parameter('wall_yaw').value)

        self.setpoint_pubs = {}
        self.last_pose = {}
        for i in range(1, self.drone_count + 1):
            self.setpoint_pubs[i] = self.create_publisher(
                PoseStamped, f'/drone{i}/setpoint_position', 10
            )
            self.create_subscription(
                PoseStamped, f'/drone{i}/pose', lambda msg, idx=i: self._pose_cb(msg, idx), 10
            )

        self.create_subscription(String, '/qr_decoded', self._qr_cb, 10)

        self.state = 'TAKEOFF'
        self.state_started = self.get_clock().now()
        self.last_qr_payload = ''
        self.qr_seen = False

        period = 1.0 / self.publish_rate_hz if self.publish_rate_hz > 0.0 else 0.05
        self.timer = self.create_timer(period, self._tick)
        self.get_logger().info('Mission sequencer started for QR wall mission.')

    def _pose_cb(self, msg: PoseStamped, idx: int):
        self.last_pose[idx] = msg

    def _qr_cb(self, msg: String):
        payload = msg.data.strip()
        if payload:
            self.qr_seen = True
            self.last_qr_payload = payload

    def _elapsed_s(self) -> float:
        return (self.get_clock().now() - self.state_started).nanoseconds / 1e9

    def _publish_setpoint(self, idx: int, x: float, y: float, z: float, yaw: float):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = float(z)
        msg.pose.orientation = yaw_to_quaternion(yaw)
        self.setpoint_pubs[idx].publish(msg)

    def _drone_near_target(self, idx: int, x: float, y: float, z: float) -> bool:
        pose = self.last_pose.get(idx)
        if pose is None:
            return False
        dx = pose.pose.position.x - x
        dy = pose.pose.position.y - y
        dz = pose.pose.position.z - z
        dist = math.sqrt(dx * dx + dy * dy + dz * dz)
        return dist <= self.transit_tolerance_m

    def _set_state(self, new_state: str):
        if new_state != self.state:
            self.state = new_state
            self.state_started = self.get_clock().now()
            self.get_logger().info(f'Mission state -> {new_state}')

    def _tick(self):
        # Drones 2..N hover at safe reference position while drone1 performs wall scan.
        for i in range(2, self.drone_count + 1):
            self._publish_setpoint(i, -7.0, 5.0 + (i - 2), self.takeoff_z, 0.0)

        if self.state == 'TAKEOFF':
            self._publish_setpoint(1, -7.0, 5.0, self.takeoff_z, 0.0)
            if self._elapsed_s() >= self.takeoff_hold_s:
                self._set_state('TRANSIT_TO_WALL')
            return

        if self.state == 'TRANSIT_TO_WALL':
            self._publish_setpoint(1, self.wall_x, self.wall_y, self.wall_z, self.wall_yaw)
            if self._drone_near_target(1, self.wall_x, self.wall_y, self.wall_z):
                self._set_state('HOLD_AND_SCAN')
            return

        if self.state == 'HOLD_AND_SCAN':
            self._publish_setpoint(1, self.wall_x, self.wall_y, self.wall_z, self.wall_yaw)
            if self.qr_seen:
                self.get_logger().info(f'QR decoded: {self.last_qr_payload}')
                self._set_state('MISSION_DONE')
                return
            if self._elapsed_s() >= self.target_hold_s:
                self.get_logger().warn('QR not decoded before timeout; holding position.')
                self._set_state('MISSION_DONE')
            return

        # MISSION_DONE: keep stable hold near wall for drone1.
        self._publish_setpoint(1, self.wall_x, self.wall_y, self.wall_z, self.wall_yaw)


def main():
    rclpy.init()
    node = MissionSequencer()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
