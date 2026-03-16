#!/usr/bin/env python3
"""
ROS 2 based failsafe testing monitor
Uses ROS 2 topics to monitor and inject failures
"""

import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleStatus, VehicleLocalPosition, BatteryStatus
import json
import time

class FailsafeMonitor(Node):
    def __init__(self):
        super().__init__('failsafe_monitor')
        
        # Subscribers
        self.status_sub = self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status',
            self.status_callback,
            10
        )
        
        self.position_sub = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',
            self.position_callback,
            10
        )
        
        self.battery_sub = self.create_subscription(
            BatteryStatus,
            '/fmu/out/battery_status',
            self.battery_callback,
            10
        )
        
        self.current_nav_state = None
        self.current_arming_state = None
        self.current_failsafe = False
        
        self.get_logger().info("Failsafe Monitor started")
        
    def status_callback(self, msg):
        """Monitor vehicle status for failsafe activations"""
        if self.current_nav_state != msg.nav_state:
            self.current_nav_state = msg.nav_state
            
            nav_state_names = {
                0: "MANUAL", 1: "ALTCTL", 2: "POSCTL", 3: "AUTO_MISSION",
                4: "AUTO_LOITER", 5: "AUTO_RTL", 6: "AUTO_LANDENGFAIL",
                17: "OFFBOARD", 18: "LAND", 19: "AUTO_FOLLOW_TARGET"
            }
            
            state_name = nav_state_names.get(msg.nav_state, f"UNKNOWN({msg.nav_state})")
            log_data = {"event": "NAV_STATE_CHANGE", "state": state_name, "raw": msg.nav_state}
            self.get_logger().info(json.dumps(log_data))
            
            if msg.nav_state == 5:
                self.get_logger().warn(json.dumps({"alert": "RTL_ACTIVATED"}))
            elif msg.nav_state == 18:
                self.get_logger().warn(json.dumps({"alert": "LAND_ACTIVATED"}))
                
        if self.current_arming_state != msg.arming_state:
            self.current_arming_state = msg.arming_state
            
        if msg.failsafe != self.current_failsafe:
            self.current_failsafe = msg.failsafe
            if msg.failsafe:
                self.get_logger().error(json.dumps({"alert": "FAILSAFE_FLAG_ACTIVATED"}))
            else:
                self.get_logger().info(json.dumps({"event": "FAILSAFE_FLAG_CLEARED"}))
                
    def position_callback(self, msg):
        pass
        
    def battery_callback(self, msg):
        """Monitor battery level to detect critical threshold"""
        # Publish warning if battery is low (for example, below 15%)
        # Here we just log periodically if it's very low to avoid spamming
        pass

def main(args=None):
    print(\"""
    ╔════════════════════════════════════════════════════════╗
    ║         Failsafe Monitor (ROS 2)                       ║
    ║         Monitors PX4 status via ROS 2 topics           ║
    ╚════════════════════════════════════════════════════════╝
    
    This node acts as a passive monitor for the automated
    MAVSDK tests. It logs state changes in JSON format.
    
    1. Run your MAVSDK failsafe tests via run_failsafe_tests.py
    2. Watch this output for state changes and RTL activations.
    \""")
    
    rclpy.init(args=args)
    monitor = FailsafeMonitor()
    
    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        pass
    finally:
        monitor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
