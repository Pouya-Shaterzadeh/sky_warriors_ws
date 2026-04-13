#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand
import math


class PX4OffboardBridge(Node):

    def __init__(self):
        super().__init__('px4_offboard_bridge')

        self.declare_parameter('drone_count', 3)
        self.declare_parameter('offboard_rate_hz', 20.0)
        self.declare_parameter('setpoint_timeout_s', 1.0)
        self.declare_parameter('auto_arm', True)
        self.declare_parameter('auto_offboard', True)
        self.declare_parameter('target_system_start', 1)
        self.declare_parameter('target_component', 1)
        self.declare_parameter('px4_ns_prefix', '/px4_')
        self.declare_parameter('drone_ns_prefix', '/drone')
        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', False)

        self.drone_count = int(self.get_parameter('drone_count').value)
        self.offboard_rate_hz = float(self.get_parameter('offboard_rate_hz').value)
        self.setpoint_timeout_s = float(self.get_parameter('setpoint_timeout_s').value)
        self.auto_arm = bool(self.get_parameter('auto_arm').value)
        self.auto_offboard = bool(self.get_parameter('auto_offboard').value)
        self.target_system_start = int(self.get_parameter('target_system_start').value)
        self.target_component = int(self.get_parameter('target_component').value)
        self.px4_ns_prefix = self._normalize_prefix(self.get_parameter('px4_ns_prefix').value)
        self.drone_ns_prefix = self._normalize_prefix(self.get_parameter('drone_ns_prefix').value)

        self.last_setpoint = {}
        self.last_setpoint_time = {}
        self.offboard_ticks = {}
        self.sent_arm = set()
        self.sent_offboard = set()

        qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.offboard_pubs = {}
        self.setpoint_pubs = {}
        self.command_pubs = {}

        for i in range(1, self.drone_count + 1):
            drone_topic = f'{self.drone_ns_prefix}{i}/setpoint_position'
            self.create_subscription(
                PoseStamped,
                drone_topic,
                lambda msg, idx=i: self._setpoint_callback(msg, idx),
                10,
            )

            px4_ns = f'{self.px4_ns_prefix}{i}'
            self.offboard_pubs[i] = self.create_publisher(
                OffboardControlMode,
                f'{px4_ns}/fmu/in/offboard_control_mode',
                qos_profile,
            )
            self.setpoint_pubs[i] = self.create_publisher(
                TrajectorySetpoint,
                f'{px4_ns}/fmu/in/trajectory_setpoint',
                qos_profile,
            )
            self.command_pubs[i] = self.create_publisher(
                VehicleCommand,
                f'{px4_ns}/fmu/in/vehicle_command',
                qos_profile,
            )

        period = 1.0 / self.offboard_rate_hz if self.offboard_rate_hz > 0 else 0.05
        self.timer = self.create_timer(period, self._publish_offboard)
        self.get_logger().info('PX4 offboard bridge ready.')

    @staticmethod
    def _normalize_prefix(prefix):
        return prefix[:-1] if prefix.endswith('/') else prefix

    def _setpoint_callback(self, msg, idx):
        self.last_setpoint[idx] = msg
        self.last_setpoint_time[idx] = self.get_clock().now()

    def _publish_offboard(self):
        now = self.get_clock().now()
        timestamp = int(now.nanoseconds / 1000)

        for i in range(1, self.drone_count + 1):
            if i not in self.last_setpoint:
                continue

            age = (now - self.last_setpoint_time[i]).nanoseconds / 1e9
            if age > self.setpoint_timeout_s:
                continue

            offboard_msg = OffboardControlMode()
            offboard_msg.timestamp = timestamp
            offboard_msg.position = True
            offboard_msg.velocity = False
            offboard_msg.acceleration = False
            offboard_msg.attitude = False
            offboard_msg.body_rate = False
            self.offboard_pubs[i].publish(offboard_msg)

            setpoint_msg = TrajectorySetpoint()
            setpoint_msg.timestamp = timestamp
            setpoint_msg.position = [
                float(self.last_setpoint[i].pose.position.x),
                float(self.last_setpoint[i].pose.position.y),
                float(self.last_setpoint[i].pose.position.z),
            ]
            setpoint_msg.yaw = self._yaw_from_quaternion(self.last_setpoint[i])
            self.setpoint_pubs[i].publish(setpoint_msg)

            self.offboard_ticks[i] = self.offboard_ticks.get(i, 0) + 1

            # Delay arming/offboard by 140 ticks (7 seconds total)
            if self.auto_offboard and i not in self.sent_offboard and self.offboard_ticks[i] >= 140:
                self.get_logger().info(f"Engaging offboard mode for drone {i} via ROS Topic...")
                self._send_vehicle_command(i, 176, 1.0, 6.0)
                self.sent_offboard.add(i)

            if self.auto_arm and i not in self.sent_arm and self.offboard_ticks[i] >= 140:
                self.get_logger().info(f"Arming drone {i} via ROS Topic...")
                self._send_vehicle_command(i, 400, 1.0, 0.0)
                self.sent_arm.add(i)

            # Periodic offboard re-engagement as GCS heartbeat (~every 5 s)
            if i in self.sent_offboard and self.offboard_ticks[i] % 100 == 0:
                self._send_vehicle_command(i, 176, 1.0, 6.0)

    @staticmethod
    def _yaw_from_quaternion(msg):
        q = msg.pose.orientation
        # Quaternion to yaw (Z axis) conversion.
        siny_cosp = 2.0 * ((q.w * q.z) + (q.x * q.y))
        cosy_cosp = 1.0 - 2.0 * ((q.y * q.y) + (q.z * q.z))
        return float(math.atan2(siny_cosp, cosy_cosp))

    def _send_vehicle_command(self, idx, command, param1, param2):
        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.command = int(command)
        # Target system 0 (Broadcast) ensures the command is accepted within the drone's namespace
        msg.target_system = 0
        msg.target_component = 1
        # Source system 255 (GCS) ensures the drone trusts the command
        msg.source_system = 255
        msg.source_component = 1
        msg.from_external = True
        self.command_pubs[idx].publish(msg)


def main():
    rclpy.init()
    node = PX4OffboardBridge()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
