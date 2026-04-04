#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import subprocess
import time
import os


class SimulationScript(Node):
    def __init__(self):
        super().__init__('simulation_node')
        self.declare_parameter('nb_vehicles', 0)
        self.declare_parameter('drone_model', 'gz_x500')
        self.declare_parameter('world', 'default')
        self.declare_parameter('initial_pose', '')
        
        # Get parameters
        nb_vehicles = self.get_parameter('nb_vehicles').value
        drone_model = self.get_parameter('drone_model').value
        world = self.get_parameter('world').value
        initial_pose_str = self.get_parameter('initial_pose').value
        
        # Parse initial poses
        initial_poses = []
        if initial_pose_str:
            pose_pairs = initial_pose_str.strip('"').split('|')
            for pair in pose_pairs:
                if pair:
                    x, y = pair.split(',')
                    initial_poses.append((float(x), float(y)))
        
        self.get_logger().info(f"Launching {nb_vehicles} drones with model {drone_model}")
        
        # PX4-Autopilot path
        px4_dir = os.path.expanduser("~/PX4-Autopilot")
        
        # Start Micro-XRCE-DDS Agent
        self.get_logger().info("Starting Micro-XRCE-DDS Agent...")
        subprocess.Popen(
            ["MicroXRCEAgent", "udp4", "-p", "8888"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL
        )
        time.sleep(2)
        
        # Start MAVLink router for QGroundControl
        self.get_logger().info("Starting MAVLink router for QGC...")
        mavlink_config = self.create_mavlink_router_config(nb_vehicles)
        with open('/tmp/mavlink-router.conf', 'w') as f:
            f.write(mavlink_config)
        
        # Try to start mavlink-routerd, but don't fail if not available
        try:
            subprocess.Popen(
                ["mavlink-routerd", "-c", "/tmp/mavlink-router.conf"],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL
            )
            self.get_logger().info("MAVLink router started successfully")
        except FileNotFoundError:
            self.get_logger().warn("mavlink-routerd not found - QGC may not connect properly. Install with: sudo apt install mavlink-router")
        
        time.sleep(1)     

        # Launch Gazebo with the world FIRST
        self.get_logger().info(f"Launching Gazebo with world: {world}")
        subprocess.Popen(
            ["gz", "sim", "-v4", "-r", world],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE
        )
        time.sleep(5)  # Give Gazebo time to fully start
        
        # Launch PX4 instances using PX4's multi-vehicle approach
        for i in range(nb_vehicles):
            instance_id = i + 1
            
            # Create working directory for this instance
            instance_dir = os.path.join(px4_dir, f"build/px4_sitl_default/instance_{instance_id}")
            os.makedirs(instance_dir, exist_ok=True)
            
            # Environment variables
            env = os.environ.copy()
            env['PX4_SIM_MODEL'] = drone_model
            env['PX4_SYS_AUTOSTART'] = '4001'
            env['PX4_GZ_WORLD'] = world
            
            # Set initial pose if provided
            if i < len(initial_poses):
                x, y = initial_poses[i]
                env['PX4_GZ_MODEL_POSE'] = f"{x},{y},0.5,0,0,0"  # x,y,z,roll,pitch,yaw
                self.get_logger().info(f"Instance {instance_id}: Position x={x}, y={y}")
            
            # Launch PX4 instance
            px4_binary = os.path.join(px4_dir, "build/px4_sitl_default/bin/px4")
            cmd = [
                px4_binary,
                "-i", str(instance_id),
                "-d", os.path.join(px4_dir, "build/px4_sitl_default/etc"),
                "-w", instance_dir
            ]
            
            self.get_logger().info(f"Starting PX4 instance {instance_id}")
            subprocess.Popen(
                cmd,
                cwd=instance_dir,
                env=env,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE
            )
            
            time.sleep(3)  # Wait between launches
        
        self.get_logger().info("All PX4 instances started successfully!")
    
    def create_mavlink_router_config(self, nb_vehicles):
        """Create MAVLink router configuration for multiple vehicles"""
        config = "[General]\n"
        config += "TcpServerPort=5760\n"
        config += "ReportStats=false\n\n"
        
        # QGC endpoint
        config += "[UdpEndpoint QGC]\n"
        config += "Mode = Normal\n"
        config += "Address = 127.0.0.1\n"
        config += "Port = 14550\n\n"
        
        # Add endpoint for each vehicle
        for i in range(nb_vehicles):
            instance_id = i + 1
            udp_port = 14540 + instance_id
            config += f"[UdpEndpoint PX4_{instance_id}]\n"
            config += "Mode = Normal\n"
            config += "Address = 127.0.0.1\n"
            config += f"Port = {udp_port}\n\n"
        return config


def main(args=None):
    rclpy.init(args=args)
    node = SimulationScript()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean shutdown
        subprocess.run(["pkill", "-9", "px4"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        subprocess.run(["pkill", "-9", "gz"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        subprocess.run(["pkill", "-9", "MicroXRCEAgent"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        rclpy.shutdown()


if __name__ == "__main__":
    main()