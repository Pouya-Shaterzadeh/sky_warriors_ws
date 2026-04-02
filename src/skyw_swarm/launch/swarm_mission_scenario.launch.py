#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    log_level = LaunchConfiguration('log_level')
    use_sim_time = LaunchConfiguration('use_sim_time')
    drone_count = LaunchConfiguration('drone_count')

    takeoff_altitude = LaunchConfiguration('takeoff_altitude')
    mission_altitude = LaunchConfiguration('mission_altitude')
    mission_spacing = LaunchConfiguration('mission_spacing')
    mission_delay_s = LaunchConfiguration('mission_delay_s')
    initial_goal_delay_s = LaunchConfiguration('initial_goal_delay_s')

    pose_bridge = Node(
        package='skyw_swarm',
        executable='px4_pose_bridge.py',
        name='px4_pose_bridge',
        output='screen',
        arguments=['--ros-args', '--log-level', log_level],
        parameters=[{'drone_count': drone_count, 'use_sim_time': use_sim_time}],
    )

    # Keeps publishing formation setpoints to /droneX/setpoint_position.
    formation_server = Node(
        package='skyw_swarm',
        executable='formation_server.py',
        name='formation_server',
        output='screen',
        arguments=['--ros-args', '--log-level', log_level],
        parameters=[{'drone_count': drone_count, 'use_sim_time': use_sim_time}],
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

    # Step 1: command all drones to climb to takeoff_altitude (line spacing = 0.0).
    takeoff_goal = ExecuteProcess(
        cmd=[
            'ros2',
            'action',
            'send_goal',
            '/set_formation',
            'skyw_swarm/action/SetFormation',
            [
                '{formation_type: "line", spacing: 0.0, altitude: ',
                takeoff_altitude,
                ', rotation: 0.0, drone_count: ',
                drone_count,
                '}',
            ],
        ],
        output='screen',
    )

    # Step 2 (delayed): transition to line formation with mission spacing.
    line_goal = ExecuteProcess(
        cmd=[
            'ros2',
            'action',
            'send_goal',
            '/set_formation',
            'skyw_swarm/action/SetFormation',
            [
                '{formation_type: "line", spacing: ',
                mission_spacing,
                ', altitude: ',
                mission_altitude,
                ', rotation: 0.0, drone_count: ',
                drone_count,
                '}',
            ],
        ],
        output='screen',
    )

    delayed_takeoff = TimerAction(period=initial_goal_delay_s, actions=[takeoff_goal])
    delayed_line = TimerAction(period=mission_delay_s, actions=[line_goal])

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
            'takeoff_altitude',
            default_value='5.0',
            description='Initial climb altitude in meters',
        ),
        DeclareLaunchArgument(
            'mission_altitude',
            default_value='5.0',
            description='Formation flight altitude in meters',
        ),
        DeclareLaunchArgument(
            'mission_spacing',
            default_value='2.0',
            description='Line formation spacing in meters',
        ),
        DeclareLaunchArgument(
            'initial_goal_delay_s',
            default_value='3.0',
            description='Delay before first goal (startup settling time)',
        ),
        DeclareLaunchArgument(
            'mission_delay_s',
            default_value='8.0',
            description='Delay before line-formation goal (seconds from launch)',
        ),
        pose_bridge,
        formation_server,
        offboard_bridge,
        delayed_takeoff,
        delayed_line,
    ])
