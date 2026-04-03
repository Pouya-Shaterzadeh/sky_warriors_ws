from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    world_name_arg = DeclareLaunchArgument(
        "world_name",
        default_value="skyw_hexagon",
        description="Must match <world name=...> in world.sdf (Gazebo /world/<name>/... topics).",
    )
    model_name_arg = DeclareLaunchArgument(
        "model_name",
        default_value="x500_mono_cam_0",
        description="Gazebo model name for the camera UAV.",
    )

    image_out_topic_arg = DeclareLaunchArgument(
        "image_out_topic",
        default_value="/camera/image_raw",
        description="ROS image topic consumed by detection node.",
    )

    camera_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="camera_image_bridge",
        output="screen",
        arguments=[
            [
                "/world/",
                LaunchConfiguration("world_name"),
                "/model/",
                LaunchConfiguration("model_name"),
                "/link/camera_link/sensor/imager/image@sensor_msgs/msg/Image[gz.msgs.Image",
            ]
        ],
        remappings=[
            (
                [
                    "/world/",
                    LaunchConfiguration("world_name"),
                    "/model/",
                    LaunchConfiguration("model_name"),
                    "/link/camera_link/sensor/imager/image",
                ],
                LaunchConfiguration("image_out_topic"),
            )
        ],
    )

    pose_bridge = Node(
        package="skyw_swarm",
        executable="px4_pose_bridge.py",
        name="px4_pose_bridge",
        output="screen",
        parameters=[{"drone_count": 3}],
    )

    return LaunchDescription(
        [
            world_name_arg,
            model_name_arg,
            image_out_topic_arg,
            camera_bridge,
            pose_bridge,
        ]
    )
