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
* Drones 2 & 3 (x500_mono_cam_down_2, x500_mono_cam_down_3): carry
  downward-facing cameras, build a shared red/blue pad map, and feed
  the final landing decision.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


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
    # Scan waypoints assume the walls are on a wider radius-7.5 hexagon and keep
    # roughly 1 m inward stand-off from each wall for QR readability.
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
            # Final simultaneous landing zone geometry and safety thresholds.
            'landing_zone_radius_m': 1.5,
            'landing_slot_radius_m': 0.75,
            'landing_stage_radius_m': 2.25,
            'landing_min_separation_m': 1.0,
            'landing_slot_tolerance_m': 0.50,
            'landing_pad_lock_confidence_min': 0.60,
            'landing_pad_lock_jitter_max_m': 0.40,
            'landing_pad_lock_dwell_s': 2.0,
            'landing_retry_backoff_s': 2.0,
            'landing_pad_search_z': -5.0,
            'landing_pad_search_timeout_s': 30.0,
            'landing_pad_fallback_max_age_s': 60.0,
            'landing_pad_fallback_confidence_min': 0.40,
            'landing_pad_fallback_min_detections': 2,
            'landing_pad_search_log_period_s': 1.0,
            'landing_abort_on_missing_lock': True,
            'landing_slot_hover_z': -2.0,
            'landing_sync_release_z': -0.75,
            'landing_sync_duration_s': 3.0,
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
            # Keep the Drone 1 QR feed compact on screen.
            'visualization_scale':     0.35,
            # Grayscale binary-threshold before pyzbar (tune if detection is poor).
            'binary_threshold':        45,
            # Only re-publish when the decoded string changes.
            'publish_only_on_change':  True,
        }],
    )

    # ── 2f. Downward-camera landing-pad detection stack ──────────────────
    # Drones 2 and 3 bridge their downward mono cameras into ROS, detect
    # red/blue pads in world coordinates, and fuse them into a shared
    # registry that the mission sequencer uses after the last QR wall.
    pad_detection_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('skyw_detection'), 'launch', 'swarm_pad_detection.launch.py']
            )
        ),
        launch_arguments={
            'world_name': world_name,
        }.items(),
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
                'Legacy per-wall override only; mission_sequencer uses its own '
                'hexagon waypoint table with ~1 m stand-off from the widened walls.'
            ),
        ),
        DeclareLaunchArgument(
            'wall_y',
            default_value='0.0',
            description='Legacy per-wall override; mission_sequencer uses its internal waypoint table.',
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
            description='Legacy heading override; mission_sequencer uses per-wall yaw values internally.',
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
        pad_detection_stack,
    ])
