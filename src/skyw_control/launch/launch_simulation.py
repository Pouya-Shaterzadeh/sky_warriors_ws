#!/usr/bin/env python3
"""
Sky Warriors Simulation Launch File
====================================
Phase 1 – Takeoff and Stabilization  (clock_bridge + offboard_helper)
Phase 2 – QR Scanning Pass            (camera bridge, pose bridge,
                                        mission sequencer, offboard bridge,
                                        QR detector)

Run:
  ros2 launch skyw_control launch_simulation.py

Phase 2 behaviour
-----------------
* Drone 1 (x500_mono_cam_1): After takeoff hold, flies to each QR wall
  waypoint and hovers while the QR detector reads the code.
* Drones 2 & 3 (x500_2, x500_3): Kept in a safe follower/hover position
  behind the start zone until the QR payload is decoded.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # ── Shared launch configurations ──────────────────────────────────────
    log_level    = LaunchConfiguration('log_level')
    use_sim_time = LaunchConfiguration('use_sim_time')
    drone_count  = LaunchConfiguration('drone_count')
    hover_z      = LaunchConfiguration('hover_z')

    # Phase 2 scan-path parameters (all overridable from CLI)
    world_name   = LaunchConfiguration('world_name')
    cam_model    = LaunchConfiguration('cam_model')
    wall_x       = LaunchConfiguration('wall_x')
    wall_y       = LaunchConfiguration('wall_y')
    wall_z       = LaunchConfiguration('wall_z')
    wall_yaw     = LaunchConfiguration('wall_yaw')
    takeoff_z    = LaunchConfiguration('takeoff_z')
    takeoff_hold = LaunchConfiguration('takeoff_hold_s')
    target_hold  = LaunchConfiguration('target_hold_s')
    camera_topic = LaunchConfiguration('camera_topic')
    decoded_topic = LaunchConfiguration('decoded_topic')

    # ══════════════════════════════════════════════════════════════════════
    # PHASE 1 – Shared Infrastructure
    # ══════════════════════════════════════════════════════════════════════

    # Bridge /clock from Gazebo so all ROS nodes can use sim time.
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen',
    )

    # ══════════════════════════════════════════════════════════════════════
    # PHASE 2 – QR Scanning Pass
    # ══════════════════════════════════════════════════════════════════════

    # ── 2a. Gazebo → ROS camera image bridge ──────────────────────────────
    # Bridges the front-facing mono camera on Drone 1 from Gazebo into
    # ROS 2 as sensor_msgs/Image on /camera/image_raw.
    #
    # Gz topic pattern:
    #   /world/<world_name>/model/<cam_model>/link/camera_link/sensor/imager/image
    # where cam_model = x500_mono_cam_1 (PX4_SIM_MODEL=gz_x500_mono_cam, -i 1)
    #
    # The ros_gz_bridge parameter_bridge accepts the full Gz→ROS mapping as a
    # single argument string:  <gz_topic>@<ros_type>[<gz_type>
    # Topic contains substitutions so we pass it as a list that launch will join.
    # Construct the full Gazebo topic string
    camera_gz_topic = ['/world/', world_name, '/model/', cam_model, '/link/camera_link/sensor/imager/image']

    camera_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        name='camera_image_bridge',
        output='screen',
        arguments=[camera_gz_topic],
        remappings=[
            (camera_gz_topic, camera_topic),
        ],
    )

    # ── 2b. PX4 Pose Bridge ───────────────────────────────────────────────
    # Converts /px4_N/fmu/out/vehicle_local_position_v1 into
    # /droneN/pose (geometry_msgs/PoseStamped) so the mission sequencer
    # can use position feedback for arrival detection.
    pose_bridge = Node(
        package='skyw_swarm',
        executable='px4_pose_bridge.py',
        name='px4_pose_bridge',
        output='screen',
        arguments=['--ros-args', '--log-level', log_level],
        parameters=[{
            'drone_count':  drone_count,
            'use_sim_time': use_sim_time,
        }],
    )

    # ── 2c. Mission Sequencer ─────────────────────────────────────────────
    # State machine:   TAKEOFF → TRANSIT_TO_WALL → HOLD_AND_SCAN → MISSION_DONE
    #
    # Drone 1: flies to the QR wall scan point and hovers while the detector runs.
    # Drones 2 & 3: held at (-7, 5+offset, takeoff_z) – safe follower/hover
    #               behind the start zone until QR is decoded.
    #
    # Publishes /droneN/setpoint_position (geometry_msgs/PoseStamped).
    # Subscribes to /qr_decoded (std_msgs/String) to know when to finish.
    mission_sequencer = Node(
        package='skyw_swarm',
        executable='mission_sequencer.py',
        name='mission_sequencer',
        output='screen',
        arguments=['--ros-args', '--log-level', log_level],
        parameters=[{
            'drone_count':          drone_count,
            'use_sim_time':         use_sim_time,
            # Altitude used during takeoff hold and follower hover (NED, up = negative).
            'takeoff_z':            takeoff_z,
            # Altitude when scanning a wall
            'wall_z':               wall_z,
            # How long to hold at takeoff altitude before starting transit.
            'takeoff_hold_s':       takeoff_hold,
            # Max time to wait for a QR decode before declaring timeout.
            'target_hold_s':        target_hold,
        }],
    )

    # ── 2d. PX4 Offboard Bridge ───────────────────────────────────────────
    # Converts /droneN/setpoint_position → PX4 offboard topics:
    #   /px4_N/fmu/in/offboard_control_mode
    #   /px4_N/fmu/in/trajectory_setpoint
    # and sends arm + offboard VehicleCommand messages when setpoints arrive.
    offboard_bridge = Node(
        package='skyw_swarm',
        executable='px4_offboard_bridge.py',
        name='px4_offboard_bridge',
        output='screen',
        arguments=['--ros-args', '--log-level', log_level],
        parameters=[{
            'drone_count':    drone_count,
            'use_sim_time':   use_sim_time,
            'auto_arm':       True,
            'auto_offboard':  True,
            'px4_ns_prefix':  '/px4_',
            'drone_ns_prefix': '/drone',
        }],
    )

    # ── 2e. QR Code Detector ──────────────────────────────────────────────
    # Subscribes to /camera/image_raw, decodes QR codes via pyzbar,
    # and publishes the payload to /qr_decoded (std_msgs/String).
    # The mission sequencer listens to /qr_decoded to transition out of
    # HOLD_AND_SCAN → MISSION_DONE.
    qr_detector = Node(
        package='skyw_detection',
        executable='qrcode_detector',
        name='qrcode_detector',
        output='screen',
        parameters=[{
            'camera_topic':            camera_topic,
            'decoded_topic':           decoded_topic,
            # Show OpenCV overlay window with detected QR bounding box.
            'enable_visualization':    True,
            # Grayscale binary-threshold before pyzbar (tune if detection is poor).
            'binary_threshold':        45,
            # Only re-publish when the decoded string changes.
            'publish_only_on_change':  True,
        }],
    )

    # ══════════════════════════════════════════════════════════════════════
    # Launch Description
    # ══════════════════════════════════════════════════════════════════════
    return LaunchDescription([

        # ── Shared arguments ──────────────────────────────────────────────
        DeclareLaunchArgument(
            'log_level',
            default_value='info',
            description='Logging level for all mission nodes.',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use Gazebo simulation time.',
        ),
        DeclareLaunchArgument(
            'drone_count',
            default_value='3',
            description='Number of PX4 vehicles in the simulation.',
        ),
        DeclareLaunchArgument(
            'hover_z',
            default_value='-5.0',
            description='Initial hover altitude in PX4 local NED frame (up = negative).',
        ),

        # ── Phase 2 arguments ─────────────────────────────────────────────
        DeclareLaunchArgument(
            'world_name',
            default_value='skyw_hexagon',
            description='Gazebo world name (must match <world name=...> in world.sdf).',
        ),
        DeclareLaunchArgument(
            'cam_model',
            default_value='x500_mono_cam_1',
            description=(
                'Gazebo model name for Drone 1 camera UAV. '
                'Derived from PX4_SIM_MODEL=gz_x500_mono_cam and -i 1 → '
                '"x500_mono_cam_1".'
            ),
        ),
        DeclareLaunchArgument(
            'camera_topic',
            default_value='/camera/image_raw',
            description='ROS topic where the bridged camera image is published.',
        ),
        DeclareLaunchArgument(
            'decoded_topic',
            default_value='/qr_decoded',
            description='ROS topic where the QR detector publishes decoded payloads.',
        ),
        DeclareLaunchArgument(
            'takeoff_z',
            default_value='-2.5',
            description=(
                'Altitude (PX4 NED, up = negative) for takeoff hold and '
                'Drones 2 & 3 follower hover.'
            ),
        ),
        DeclareLaunchArgument(
            'wall_x',
            default_value='3.0',
            description=(
                'Drone 1 QR scan X position in PX4 local frame. '
                'wall_1 is at Gz X=5; drone hovers ~2 m in front.'
            ),
        ),
        DeclareLaunchArgument(
            'wall_y',
            default_value='0.0',
            description='Drone 1 QR scan Y position in PX4 local frame.',
        ),
        DeclareLaunchArgument(
            'wall_z',
            default_value='-1.5',
            description=(
                'Drone 1 QR scan altitude in PX4 NED frame (up = negative). '
                'QR panel centre is at Gz Z=1.5 m.'
            ),
        ),
        DeclareLaunchArgument(
            'wall_yaw',
            default_value='1.5708',
            description='Drone 1 heading at scan point (rad). 1.5708 rad faces +X → wall_1.',
        ),
        DeclareLaunchArgument(
            'takeoff_hold_s',
            default_value='8.0',
            description='Seconds to hold at takeoff altitude before starting QR transit.',
        ),
        DeclareLaunchArgument(
            'target_hold_s',
            default_value='30.0',
            description='Max seconds to wait for a QR decode before timeout.',
        ),

        # ── Phase 1 nodes ─────────────────────────────────────────────────
        clock_bridge,

        # ── Phase 2 nodes ─────────────────────────────────────────────────
        camera_bridge,
        pose_bridge,
        mission_sequencer,
        offboard_bridge,
        qr_detector,
    ])
