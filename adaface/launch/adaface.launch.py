import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

def generate_launch_description():

    input_image_topic = LaunchConfiguration("input_image_topic", default="/camera/camera/color/image_raw")
    input_image_topic_cmd = DeclareLaunchArgument(
        "input_image_topic",
        default_value="/camera/camera/color/image_raw",
        description="Name of the input image topic")

    image_reliability = LaunchConfiguration("image_reliability", default="2")
    image_reliability_cmd = DeclareLaunchArgument(
        "image_reliability",
        default_value="2",
        choices=["0", "1", "2"],
        description="Specific reliability QoS of the input image topic (0=system default, 1=Reliable, 2=Best Effort)")

    target_frame = LaunchConfiguration("target_frame", default="camera_link")
    target_frame_cmd = DeclareLaunchArgument(
        "target_frame",
        default_value="camera_link",
        description="Target frame to transform the 3D boxes")

    # NODES
    face_recogntion_cmd = Node(
        package="adaface",
        executable="face_recognition",
        name="face_recognition",
        # namespace='adaface',
        output='screen',
        parameters=[{
            "target_frame": target_frame,
            "image_reliability": image_reliability
        }],
        remappings=[
            ("image_raw", input_image_topic),
            ("tracking","/yolo/tracking")]
    )

    return LaunchDescription([
        input_image_topic_cmd,
        image_reliability_cmd,
        target_frame_cmd,
        face_recogntion_cmd
    ])
