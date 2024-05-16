import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare launch arguments
    launch_arguments = [
        DeclareLaunchArgument(name, default_value=value, description=description)
        for name, value, description in [
            ("fr_weight", "ir_50", "Model name or path"),
            ("device", "cuda:0", "Device to use (GPU/CPU)"),
            ("option", "1", "0: Save embeddings, 1: Run video from webcam/video"),
            ("thresh", "0.2", "Minimum probability of faces' similarity to be published"),
            ("max_obj", "6", "Maximum counts of Face Detection"),
            ("dataset", "face_dataset/test", "Path to face dataset & face ID"),
            ("video", "0", "0: Webcam, 1: Specific video path"),
            # ("input_image_topic", "/camera/camera/color/image_raw", "Input image topic"),
            ("image_reliability", "2", "Specific reliability QoS of the input image topic (0=system default, 1=Reliable, 2=Best Effort)"),
            ("namespace_adaface", "adaface", "Namespace for the nodes"),
            ("detections", "/yolo/tracking", "YOLOv8 tracking"),
            ("person_name","Uninitialzed","Write the name you want to detect")
        ]
    ]
    webcam_topic_cmd = DeclareLaunchArgument(
        "input_image_topic",
        default_value="/camera/camera/color/image_raw",
        description="Name of the input image topic",
        condition = IfCondition(PythonExpression([
            LaunchConfiguration("video"),
            '== 0',
            ])))
    video_topic_cmd = DeclareLaunchArgument(
        "input_image_topic",
        default_value="/adaface/video_topic",
        description="Name of the input image topic",
        condition = IfCondition(PythonExpression([
            LaunchConfiguration("video"),
            '== 1',
            ])))

    # Node: face_recognition
    face_recognition_node = Node(
        package="adaface",
        executable="face_recognition",
        name="face_recongnition",
        namespace=LaunchConfiguration("namespace_adaface"),
        parameters=[{
            "fr_weight": LaunchConfiguration("fr_weight"),
            "device": LaunchConfiguration("device"),
            "option": LaunchConfiguration("option"),
            "thresh": LaunchConfiguration("thresh"),
            "max_obj": LaunchConfiguration("max_obj"),
            "dataset": LaunchConfiguration("dataset"),
            "video": LaunchConfiguration("video"),
            "image_reliability": LaunchConfiguration("image_reliability"),
        }],
        remappings=[
            ("image_raw", LaunchConfiguration("input_image_topic")),
            ("detections", LaunchConfiguration("detections")) # /yolo/tracking node
        ],
        output='screen',
    )

    # Node: video_publisher
    video_publisher_node = Node(
        package="adaface",
        executable="video_publisher",
        name="video_publisher",
        namespace=LaunchConfiguration("namespace_adaface"),
        output='screen',
        condition=IfCondition(PythonExpression([
            LaunchConfiguration("video"),
            '== 1',
        ]))
    )

    # Include launch descriptions
    realsense_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([FindPackageShare("realsense2_camera"), '/launch/rs_launch.py']),
        launch_arguments=[
            ('depth_module.profile', '640x480x60'),
            ('rgb_camera.profile', '640x480x60'),
            ('pointcloud.enable', 'true')
        ] if LaunchConfiguration("video") == "0" else [],
        condition=IfCondition(PythonExpression([LaunchConfiguration("video"), '== 0'])),
    )

    yolov8_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([FindPackageShare("yolov8_bringup"), '/launch/yolov8.launch.py']),
        launch_arguments=[
            ('model', 'yolov8m-pose.pt'),
            ('input_image_topic', LaunchConfiguration("input_image_topic")),
            ('target_frame', 'camera_link'),
            ('person_name',LaunchConfiguration('person_name')),
        ]
    )

    # Construct launch description
    ld = LaunchDescription()

    # Add actions to launch description
    [ld.add_action(action) for action in launch_arguments]
    [ld.add_action(action) for action in [webcam_topic_cmd,video_topic_cmd]]
    [ld.add_action(action) for action in [realsense_launch_cmd, video_publisher_node, yolov8_launch_cmd, face_recognition_node]]

    return ld