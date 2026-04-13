from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    world_name_arg = DeclareLaunchArgument(
        "world_name",
        default_value="skyw_hexagon",
        description="Gazebo world name for camera topic bridging.",
    )
    drone2_model_arg = DeclareLaunchArgument(
        "drone2_model",
        default_value="x500_mono_cam_down_2",
        description="Gazebo model name for drone 2.",
    )
    drone3_model_arg = DeclareLaunchArgument(
        "drone3_model",
        default_value="x500_mono_cam_down_3",
        description="Gazebo model name for drone 3.",
    )
    camera_fx_arg = DeclareLaunchArgument("camera_fx", default_value="541.7")
    camera_fy_arg = DeclareLaunchArgument("camera_fy", default_value="541.7")
    camera_cx_arg = DeclareLaunchArgument("camera_cx", default_value="-1.0")
    camera_cy_arg = DeclareLaunchArgument("camera_cy", default_value="-1.0")

    drone2_gz_topic = [
        "/world/",
        LaunchConfiguration("world_name"),
        "/model/",
        LaunchConfiguration("drone2_model"),
        "/link/camera_link/sensor/imager/image",
    ]
    drone3_gz_topic = [
        "/world/",
        LaunchConfiguration("world_name"),
        "/model/",
        LaunchConfiguration("drone3_model"),
        "/link/camera_link/sensor/imager/image",
    ]

    drone2_image_topic = "/drone2/camera_down/image_raw"
    drone3_image_topic = "/drone3/camera_down/image_raw"

    drone2_bridge = Node(
        package="ros_gz_image",
        executable="image_bridge",
        name="drone2_down_camera_bridge",
        output="screen",
        arguments=[drone2_gz_topic],
        remappings=[(drone2_gz_topic, drone2_image_topic)],
    )

    drone3_bridge = Node(
        package="ros_gz_image",
        executable="image_bridge",
        name="drone3_down_camera_bridge",
        output="screen",
        arguments=[drone3_gz_topic],
        remappings=[(drone3_gz_topic, drone3_image_topic)],
    )

    detector_drone2 = Node(
        package="skyw_detection",
        executable="swarm_pad_detector",
        name="swarm_pad_detector_drone2",
        output="screen",
        parameters=[
            {
                "drone_id": 2,
                "camera_topic": drone2_image_topic,
                "position_topic": "/px4_2/fmu/out/vehicle_local_position_v1",
                "attitude_topic": "/px4_2/fmu/out/vehicle_attitude",
                "spawn_xy": "[-7.0, 4.0]",
                "camera_fx": LaunchConfiguration("camera_fx"),
                "camera_fy": LaunchConfiguration("camera_fy"),
                "camera_cx": LaunchConfiguration("camera_cx"),
                "camera_cy": LaunchConfiguration("camera_cy"),
                "camera_translation_body_frd": "[0.0, 0.0, -0.10]",
                "thresholds_file": "config/color_thresholds.yaml",
                "enable_visualization": True,
                "visualization_window_name": "Drone 2 Down Camera",
                "visualization_scale": 0.45,
            }
        ],
    )

    detector_drone3 = Node(
        package="skyw_detection",
        executable="swarm_pad_detector",
        name="swarm_pad_detector_drone3",
        output="screen",
        parameters=[
            {
                "drone_id": 3,
                "camera_topic": drone3_image_topic,
                "position_topic": "/px4_3/fmu/out/vehicle_local_position_v1",
                "attitude_topic": "/px4_3/fmu/out/vehicle_attitude",
                "spawn_xy": "[-7.0, 6.0]",
                "camera_fx": LaunchConfiguration("camera_fx"),
                "camera_fy": LaunchConfiguration("camera_fy"),
                "camera_cx": LaunchConfiguration("camera_cx"),
                "camera_cy": LaunchConfiguration("camera_cy"),
                "camera_translation_body_frd": "[0.0, 0.0, -0.10]",
                "thresholds_file": "config/color_thresholds.yaml",
                "enable_visualization": False,
                "visualization_window_name": "Drone 3 Down Camera",
                "visualization_scale": 0.45,
            }
        ],
    )

    tracker = Node(
        package="skyw_detection",
        executable="landing_pad_tracker",
        name="landing_pad_tracker",
        output="screen",
    )

    return LaunchDescription(
        [
            world_name_arg,
            drone2_model_arg,
            drone3_model_arg,
            camera_fx_arg,
            camera_fy_arg,
            camera_cx_arg,
            camera_cy_arg,
            drone2_bridge,
            drone3_bridge,
            detector_drone2,
            detector_drone3,
            tracker,
        ]
    )
