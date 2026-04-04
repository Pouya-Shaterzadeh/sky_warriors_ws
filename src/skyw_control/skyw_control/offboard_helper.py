#!/usr/bin/env python3
"""
QGroundControl Offboard Mode Helper
This script automatically sends offboard control commands to enable offboard mode
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand
import time
import subprocess

class OffboardModeHelper(Node):
    def __init__(self):
        super().__init__('offboard_helper')
        
        # PX4 compatible QoS profile
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Create publishers for each drone
        self.offboard_pubs = []
        self.setpoint_pubs = []
        self.command_pubs = []
        
        self.nb_drones = 3
        
        for i in range(1, self.nb_drones + 1):
            ns = f"/px4_{i}"
            
            # OffboardControlMode publisher
            self.offboard_pubs.append(
                self.create_publisher(OffboardControlMode, f"{ns}/fmu/in/offboard_control_mode", qos_profile)
            )
            
            # TrajectorySetpoint publisher  
            self.setpoint_pubs.append(
                self.create_publisher(TrajectorySetpoint, f"{ns}/fmu/in/trajectory_setpoint", qos_profile)
            )
            
            # VehicleCommand publisher for arming/mode changes
            self.command_pubs.append(
                self.create_publisher(VehicleCommand, f"{ns}/fmu/in/vehicle_command", qos_profile)
            )
        
        self.offboard_setpoint_counter = 0
        
        # Timer at 10Hz to publish offboard messages (PX4 requires >2Hz)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info("Offboard mode helper started - sending offboard commands at 10Hz")
    
    def timer_callback(self):
        """Publish offboard control mode and setpoints"""
        
        # Need to send several setpoints before PX4 accepts offboard mode
        if self.offboard_setpoint_counter == 10:
            # After 1 second of setpoints, engage offboard mode via commander
            self.get_logger().info("Switching all drones to offboard mode and arming...")
            self.engage_offboard_mode_via_commander()
        
        # Continuously publish offboard commands
        self.publish_offboard_control_mode()
        self.publish_trajectory_setpoint()
        
        self.offboard_setpoint_counter += 1
    
    def publish_offboard_control_mode(self):
        """Publish offboard control mode for all drones"""
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        
        for pub in self.offboard_pubs:
            pub.publish(msg)
    
    def publish_trajectory_setpoint(self):
        """Publish trajectory setpoint (hover) for all drones"""
        msg = TrajectorySetpoint()
        msg.position = [0.0, 0.0, -2.0]  # Hover at 2m altitude (NED frame)
        msg.velocity = [float('nan'), float('nan'), float('nan')]
        msg.yaw = 0.0
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        
        for pub in self.setpoint_pubs:
            pub.publish(msg)
    
    def engage_offboard_mode_via_commander(self):
        """Switch all drones to offboard mode using PX4 commander (the working method)"""
        try:
            for i in range(self.nb_drones):
                # Execute commander commands via tmux for each drone
                tmux_window = f"px4-sitl:{i}"
                
                # Set offboard mode
                subprocess.run([
                    'tmux', 'send-keys', '-t', tmux_window, 
                    'commander mode offboard', 'Enter'
                ], check=True)
                
                # Small delay between mode and arm commands  
                time.sleep(0.1)
                
                # Arm the drone
                subprocess.run([
                    'tmux', 'send-keys', '-t', tmux_window,
                    'commander arm', 'Enter'
                ], check=True)
                
                self.get_logger().info(f"Set drone {i+1} to offboard mode and armed via commander")
                
        except subprocess.CalledProcessError as e:
            self.get_logger().error(f"Failed to execute commander commands: {e}")
        except Exception as e:
            self.get_logger().error(f"Error in engage_offboard_mode_via_commander: {e}")
    
    def engage_offboard_mode(self, drone_idx):
        """Switch to offboard mode (kept for compatibility but not used)"""
        msg = VehicleCommand()
        msg.command = VehicleCommand.VEHICLE_CMD_DO_SET_MODE
        msg.param1 = 1.0  # Custom mode
        msg.param2 = 6.0  # Offboard mode
        msg.target_system = drone_idx + 1
        msg.target_component = 1
        msg.source_system = drone_idx + 1  # Source should match target
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        
        self.command_pubs[drone_idx].publish(msg)
        self.get_logger().info(f"Engaging offboard mode for drone {drone_idx + 1}")
    
    def arm(self, drone_idx):
        """Arm the drone (kept for compatibility but not used)"""
        msg = VehicleCommand()
        msg.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
        msg.param1 = 1.0  # Arm
        msg.target_system = drone_idx + 1
        msg.target_component = 1
        msg.source_system = drone_idx + 1  # Source should match target
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        
        self.command_pubs[drone_idx].publish(msg)
        self.get_logger().info(f"Arming drone {drone_idx + 1}")

def main(args=None):
    rclpy.init(args=args)
    node = OffboardModeHelper()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()