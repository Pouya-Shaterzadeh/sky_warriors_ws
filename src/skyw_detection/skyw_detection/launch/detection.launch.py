from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    camera_topic_arg = DeclareLaunchArgument(
        "camera_topic",
        default_value="/camera/image_raw",
        description="Topic publishing sensor_msgs/Image for the UAV camera.",
    )
    pose_topic_arg = DeclareLaunchArgument(
        "pose_topic",
        default_value="/drone1/pose",
        description="Topic publishing geometry_msgs/PoseStamped for the UAV.",
    )
    use_drone_pose_transform_arg = DeclareLaunchArgument(
        "use_drone_pose_transform",
        default_value="true",
        description="If true, place markers using the latest drone pose.",
    )
    marker_frame_id_arg = DeclareLaunchArgument(
        "marker_frame_id",
        default_value="map",
        description="Frame id used in MarkerArray/PoseArray output.",
    )

    thresholds_file_arg = DeclareLaunchArgument(
        "thresholds_file",
        default_value="config/color_thresholds.yaml",
        description="Relative path (within skyw_detection share dir) to the HSV threshold YAML.",
    )

    camera_fx_arg = DeclareLaunchArgument("camera_fx", default_value="320.0")
    camera_fy_arg = DeclareLaunchArgument("camera_fy", default_value="320.0")
    camera_cx_arg = DeclareLaunchArgument("camera_cx", default_value="-1.0")
    camera_cy_arg = DeclareLaunchArgument("camera_cy", default_value="-1.0")

    camera_to_drone_translation_arg = DeclareLaunchArgument(
        "camera_to_drone_translation",
        default_value="[0.0, 0.0, 0.0]",
        description="[x,y,z] meters (camera optical frame -> drone/body frame)",
    )
    camera_to_drone_rpy_arg = DeclareLaunchArgument(
        "camera_to_drone_rpy",
        default_value="[0.0, 0.0, 0.0]",
        description="[roll,pitch,yaw] radians (camera optical frame -> drone/body frame)",
    )

    node = Node(
        package="skyw_detection",
        executable="color_detector_node",
        name="color_detector_node",
        output="screen",
        parameters=[
            {
                "camera_topic": LaunchConfiguration("camera_topic"),
                "pose_topic": LaunchConfiguration("pose_topic"),
                "use_drone_pose_transform": LaunchConfiguration("use_drone_pose_transform"),
                "marker_frame_id": LaunchConfiguration("marker_frame_id"),
                "thresholds_file": LaunchConfiguration("thresholds_file"),
                "camera_fx": LaunchConfiguration("camera_fx"),
                "camera_fy": LaunchConfiguration("camera_fy"),
                "camera_cx": LaunchConfiguration("camera_cx"),
                "camera_cy": LaunchConfiguration("camera_cy"),
                "camera_to_drone_translation": LaunchConfiguration(
                    "camera_to_drone_translation"
                ),
                "camera_to_drone_rpy": LaunchConfiguration("camera_to_drone_rpy"),
            }
        ],
    )

    return LaunchDescription(
        [
            camera_topic_arg,
            pose_topic_arg,
            use_drone_pose_transform_arg,
            marker_frame_id_arg,
            thresholds_file_arg,
            camera_fx_arg,
            camera_fy_arg,
            camera_cx_arg,
            camera_cy_arg,
            camera_to_drone_translation_arg,
            camera_to_drone_rpy_arg,
            node,
        ]
    )

