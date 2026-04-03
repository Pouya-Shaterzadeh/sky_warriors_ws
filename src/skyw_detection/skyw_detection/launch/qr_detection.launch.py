from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    camera_topic_arg = DeclareLaunchArgument(
        "camera_topic",
        default_value="/camera/image_raw",
        description="Camera image topic (sensor_msgs/Image).",
    )
    decoded_topic_arg = DeclareLaunchArgument(
        "decoded_topic",
        default_value="/qr_decoded",
        description="Published decoded QR payload (std_msgs/String).",
    )
    enable_visualization_arg = DeclareLaunchArgument(
        "enable_visualization",
        default_value="true",
        description="If true, show OpenCV window overlay.",
    )
    binary_threshold_arg = DeclareLaunchArgument(
        "binary_threshold",
        default_value="45",
        description="Grayscale fixed threshold for pyzbar (0-255).",
    )

    qr_node = Node(
        package="skyw_detection",
        executable="qrcode_detector",
        name="qrcode_detector",
        output="screen",
        parameters=[
            {
                "camera_topic": LaunchConfiguration("camera_topic"),
                "decoded_topic": LaunchConfiguration("decoded_topic"),
                "enable_visualization": LaunchConfiguration("enable_visualization"),
                "binary_threshold": LaunchConfiguration("binary_threshold"),
            }
        ],
    )

    return LaunchDescription(
        [
            camera_topic_arg,
            decoded_topic_arg,
            enable_visualization_arg,
            binary_threshold_arg,
            qr_node,
        ]
    )
