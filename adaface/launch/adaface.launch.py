import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,IncludeLaunchDescription
from launch.actions import LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
def generate_launch_description():

    # input_image_topic = DeclareLaunchArgument(
    #     "input_image_topic",
    #     default_value="/camera/camera/color/image_raw",
    #     description="Name of the input image topic")
    
    # image_reliability = DeclareLaunchArgument(
    #     "image_reliability",
    #     default_value="2",
    #     choices=["0", "1", "2"],
    #     description="Specific reliability QoS of the input image topic (0=system default, 1=Reliable, 2=Best Effort)")
    

    # NODES
    face_recogntion_cmd = Node(
        package="adaface",
        executable="face_recognition",
        name="face_recognition",
        output='screen',
        # parameters=[{
        #     "image_raw" : LaunchConfiguration("input_image_topic"),
        #     "image_reliability": LaunchConfiguration("image_reliability"),
        # }],
    )

    return LaunchDescription([
        LogInfo(msg=['Execute launch files! (realsenseROS2 | YOLOv8 | Adaface)']),
          # realsense package launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [get_package_share_directory('realsense2_camera'), '/launch/rs_launch.py']),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [get_package_share_directory('yolov8_bringup'), '/launch/yolov8_3d.launch.py']),
        ),
        # input_image_topic,
        # image_reliability,
        face_recogntion_cmd
    ])
