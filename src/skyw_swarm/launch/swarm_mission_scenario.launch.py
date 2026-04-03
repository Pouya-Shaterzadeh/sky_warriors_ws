#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    log_level = LaunchConfiguration('log_level')
    use_sim_time = LaunchConfiguration('use_sim_time')
    drone_count = LaunchConfiguration('drone_count')

    takeoff_z = LaunchConfiguration('takeoff_z')
    wall_x = LaunchConfiguration('wall_x')
    wall_y = LaunchConfiguration('wall_y')
    wall_z = LaunchConfiguration('wall_z')
    wall_yaw = LaunchConfiguration('wall_yaw')

    pose_bridge = Node(
        package='skyw_swarm',
        executable='px4_pose_bridge.py',
        name='px4_pose_bridge',
        output='screen',
        arguments=['--ros-args', '--log-level', log_level],
        parameters=[{'drone_count': drone_count, 'use_sim_time': use_sim_time}],
    )

    mission_sequencer = Node(
        package='skyw_swarm',
        executable='mission_sequencer.py',
        name='mission_sequencer',
        output='screen',
        arguments=['--ros-args', '--log-level', log_level],
        parameters=[{
            'drone_count': drone_count,
            'use_sim_time': use_sim_time,
            'takeoff_z': takeoff_z,
            'wall_x': wall_x,
            'wall_y': wall_y,
            'wall_z': wall_z,
            'wall_yaw': wall_yaw,
        }],
    )

    # Converts /droneX/setpoint_position into PX4 offboard topics and sends arm/offboard commands.
    offboard_bridge = Node(
        package='skyw_swarm',
        executable='px4_offboard_bridge.py',
        name='px4_offboard_bridge',
        output='screen',
        arguments=['--ros-args', '--log-level', log_level],
        parameters=[{
            'drone_count': drone_count,
            'use_sim_time': use_sim_time,
            'auto_arm': True,
            'auto_offboard': True,
            'px4_ns_prefix': '/px4_',
            'drone_ns_prefix': '/drone',
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'log_level',
            default_value='info',
            description='Logging level for all swarm mission nodes',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time if true',
        ),
        DeclareLaunchArgument(
            'drone_count',
            default_value='3',
            description='Number of PX4 vehicles in the mission',
        ),
        DeclareLaunchArgument(
            'takeoff_z',
            default_value='-2.5',
            description='Takeoff Z setpoint in PX4 local frame (NED: up is negative)',
        ),
        DeclareLaunchArgument(
            'wall_x',
            default_value='5.0',
            description='Wall target X in PX4 local frame',
        ),
        DeclareLaunchArgument(
            'wall_y',
            default_value='0.0',
            description='Wall target Y in PX4 local frame',
        ),
        DeclareLaunchArgument(
            'wall_z',
            default_value='-1.0',
            description='Wall target Z in PX4 local frame (NED: up is negative)',
        ),
        DeclareLaunchArgument(
            'wall_yaw',
            default_value='1.57',
            description='Wall target yaw in radians',
        ),
        pose_bridge,
        mission_sequencer,
        offboard_bridge,
    ])
