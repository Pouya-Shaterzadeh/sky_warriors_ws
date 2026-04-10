#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Quaternion
import math
import json
import numpy as np

# Import our formation math
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
        super().__init__('mission_sequencer')

        self.declare_parameter('drone_count', 3)
        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('transit_z', -6.0) 
        self.declare_parameter('wall_z', -2.0)
        self.declare_parameter('transit_tolerance_m', 0.5) 
        self.declare_parameter('safety_offset_m', 2.0) 

        self.drone_count = int(self.get_parameter('drone_count').value)
        self.publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self.transit_z = float(self.get_parameter('transit_z').value)
        self.wall_z = float(self.get_parameter('wall_z').value)
        self.transit_tolerance_m = float(self.get_parameter('transit_tolerance_m').value)
        self.safety_offset_m = float(self.get_parameter('safety_offset_m').value)

        self.takeoff_timer_start = None

        # Hardcode Gazebo Spawns
        self.gz_spawns = {
            1: {'x': -7.0, 'y': 5.0},
            2: {'x': -7.0, 'y': 4.0},
            3: {'x': -7.0, 'y': 6.0},
        }

        # 6 fixed targets
        self.wall_waypoints = [
            {'x':  4.00, 'y':  0.00, 'gz_yaw':  0.000},
            {'x':  2.00, 'y':  3.46, 'gz_yaw':  1.047},
            {'x': -2.00, 'y':  3.46, 'gz_yaw':  2.094},
            {'x': -4.00, 'y':  0.00, 'gz_yaw':  3.142},
            {'x': -2.00, 'y': -3.46, 'gz_yaw': -2.094},
            {'x':  2.00, 'y': -3.46, 'gz_yaw': -1.047},
        ]
        self.current_wp_idx = 0

        # Dynamics (Leader Target)
        self.d1_target_x = 0.0
        self.d1_target_y = 0.0
        self.d1_target_z = self.transit_z
        self.d1_target_yaw = 0.0
        
        self.overwatch_gz_x = 0.0
        self.overwatch_gz_y = 0.0
        
        # Swarm Config
        self.current_spacing = 2.0
        self.target_formation = 'v'
        self.active_formation = 'column' 
        self.payload_h = 0.0
        self.next_wall_ovr = None
        
        # Payload Buffer
        self.pending_payload = None

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
        self.qr_seen = False

        period = 1.0 / self.publish_rate_hz if self.publish_rate_hz > 0.0 else 0.05
        self.timer = self.create_timer(period, self._tick)
        self.get_logger().info('Mission sequencer started! Final-Station Hover mode active.')

    def _local_to_global(self, idx: int, px4_x: float, px4_y: float):
        gz_x = px4_y + self.gz_spawns[idx]['x']
        gz_y = px4_x + self.gz_spawns[idx]['y']
        return gz_x, gz_y

    def _global_to_local(self, idx: int, gz_x: float, gz_y: float):
        px4_x = gz_y - self.gz_spawns[idx]['y']
        px4_y = gz_x - self.gz_spawns[idx]['x']
        return px4_x, px4_y

    def _qr_cb(self, msg: String):
        if self.state == 'MISSION_DONE':
            return
        try:
            payload = msg.data.strip()
            self.pending_payload = json.loads(payload)
            self.qr_seen = True
        except:
            self.qr_seen = True

    def _apply_buffered_payload(self):
        if not self.pending_payload:
            return
            
        config = self.pending_payload
        
        # 1. Spacing & Formation
        self.current_spacing = float(config.get('s', self.current_spacing))
        self.target_formation = str(config.get('f', self.target_formation)).lower()
        
        # 2. Timing
        self.payload_h = float(config.get('h', 0.0))
        
        # 3. Next Wall (Robust parsing)
        next_n = config.get('n')
        if next_n is not None:
            try:
                self.next_wall_ovr = int(next_n)
            except (ValueError, TypeError):
                self.next_wall_ovr = None # Ignore "N/A" strings
        
        # 4. Altitude (a)
        new_a = config.get('a')
        if new_a is not None:
            self.d1_target_z = -abs(float(new_a))
            
        # 5. Handle nested Turkish gorev keys
        gorev = config.get('gorev', {})
        form = gorev.get('formasyon', {})
        if form.get('aktif'):
            self.current_spacing = float(form.get('spacing', self.current_spacing))
            self.target_formation = str(form.get('tip', self.target_formation)).lower()
        
        irtifa = gorev.get('irtifa_degisim', {})
        if irtifa.get('aktif'):
            self.d1_target_z = -abs(float(irtifa.get('deger', abs(self.d1_target_z))))
            
        self.active_formation = self.target_formation
        self.pending_payload = None

    def _tick(self):
        now = self.get_clock().now()
        elapsed = (now - self.state_started).nanoseconds / 1e9

        if self.state == 'TAKEOFF':
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
                    self._set_state('TRANSIT_TO_HUB')
            return

        elif self.state == 'TRANSIT_TO_HUB':
            self.active_formation = 'column' 
            self.d1_target_z = self.transit_z
            tx, ty = self._global_to_local(1, 0.0, 0.0)
            self.d1_target_x, self.d1_target_y = tx, ty
            
            if self._drone_near_target(1, tx, ty, self.transit_z):
                self._update_wall_targets(safety_gap=True)
                self._set_state('TRANSIT_HIGH')

        elif self.state == 'TRANSIT_HIGH':
            if self._drone_near_target(1, self.d1_target_x, self.d1_target_y, self.transit_z):
                self._set_state('STABILIZE_HIGH')

        elif self.state == 'STABILIZE_HIGH':
            anchor_px4_x, anchor_px4_y = self.d1_target_x, self.d1_target_y
            self.overwatch_gz_x, self.overwatch_gz_y = self._local_to_global(1, anchor_px4_x, anchor_px4_y)
            if elapsed >= 3.0:
                self.d1_target_z = self.wall_z
                self._set_state('DESCEND_VERTICAL')

        elif self.state == 'DESCEND_VERTICAL':
            if self._drone_near_target(1, self.d1_target_x, self.d1_target_y, self.wall_z):
                self._update_wall_targets(safety_gap=False)
                self._set_state('NUDGE_TO_SCAN')

        elif self.state == 'NUDGE_TO_SCAN':
            if self._drone_near_target(1, self.d1_target_x, self.d1_target_y, self.wall_z):
                self._set_state('HOLD_AND_SCAN')

        elif self.state == 'HOLD_AND_SCAN':
            if self.qr_seen:
                self._apply_buffered_payload()
                self._set_state('PERFORM_TASK')
            elif elapsed >= 10.0:
                self._prepare_next_wall()

        elif self.state == 'PERFORM_TASK':
            if elapsed >= self.payload_h:
                self._prepare_next_wall()

        # --- Setpoint Publishing ---
        l_x, l_y, l_z, l_yaw = self.d1_target_x, self.d1_target_y, self.d1_target_z, self.d1_target_yaw
        l_gz_x, l_gz_y = self._local_to_global(1, l_x, l_y)
        self._publish_setpoint(1, l_x, l_y, l_z, l_yaw)

        # Decide which reference to use for followers
        is_overwatch = self.state in ['DESCEND_VERTICAL', 'NUDGE_TO_SCAN', 'HOLD_AND_SCAN']
        anchor_x = self.overwatch_gz_x if is_overwatch else l_gz_x
        anchor_y = self.overwatch_gz_y if is_overwatch else l_gz_y
        
        current_form = 'column' if self.state in ['TRANSIT_HIGH', 'TRANSIT_TO_HUB', 'STABILIZE_HIGH'] else self.active_formation
        if is_overwatch:
             current_form = 'column' # Narrow Overwatch

        offsets = formation_math.get_offsets(current_form, self.current_spacing, self.drone_count)

        for i in range(2, self.drone_count + 1):
            if self.state == 'TAKEOFF':
                self._publish_setpoint(i, 0.0, 0.0, self.transit_z, 0.0)
            else:
                f_x, f_y = offsets[i-1]
                t_gz_x = anchor_x + f_x * math.cos(l_yaw) - f_y * math.sin(l_yaw)
                t_gz_y = anchor_y + f_x * math.sin(l_yaw) + f_y * math.cos(l_yaw)
                t_px4_x, t_px4_y = self._global_to_local(i, t_gz_x, t_gz_y)
                
                if is_overwatch:
                    self._publish_setpoint(i, t_px4_x, t_px4_y, self.wall_z, l_yaw)
                else:
                    self._publish_setpoint(i, t_px4_x, t_px4_y, l_z, l_yaw)

    def _prepare_next_wall(self):
        # Check for next wall override (n)
        if self.next_wall_ovr is not None:
            self.current_wp_idx = int(self.next_wall_ovr) - 1
            self.next_wall_ovr = None
        else:
            self.current_wp_idx += 1
            
        if self.current_wp_idx < len(self.wall_waypoints):
            self.qr_seen = False 
            self.payload_h = 0.0 
            self._set_state('TRANSIT_TO_HUB') 
        else:
            self.get_logger().info('Swarm journey complete. Keeping final station position.')
            self._set_state('MISSION_DONE')

    def _drone_near_target(self, idx, x, y, z):
        pose = self.last_pose.get(idx)
        if not pose: return False
        dist = math.sqrt((pose.pose.position.x - x)**2 + (pose.pose.position.y - y)**2 + (pose.pose.position.z - z)**2)
        return dist <= self.transit_tolerance_m

    def _set_state(self, new_state):
        if new_state != self.state:
            self.state = new_state
            self.state_started = self.get_clock().now()
            self.get_logger().info(f'State -> {new_state}')

    def _update_wall_targets(self, safety_gap=True):
        wp = self.wall_waypoints[self.current_wp_idx]
        gz_x, gz_y, gz_yaw = wp['x'], wp['y'], wp['gz_yaw']

        if safety_gap:
            gz_x -= self.safety_offset_m * math.cos(gz_yaw)
            gz_y -= self.safety_offset_m * math.sin(gz_yaw)

        px4_x, px4_y = self._global_to_local(1, gz_x, gz_y)
        self.d1_target_x, self.d1_target_y = px4_x, px4_y
        self.d1_target_yaw = (math.pi / 2.0) - gz_yaw
        self.d1_target_z = self.transit_z if safety_gap else self.wall_z

    def _pose_cb(self, msg, idx):
        self.last_pose[idx] = msg

    def _publish_setpoint(self, idx, x, y, z, yaw):
        m = PoseStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'map'
        m.pose.position.x, m.pose.position.y, m.pose.position.z = float(x), float(y), float(z)
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

if __name__ == '__main__':
    main()
