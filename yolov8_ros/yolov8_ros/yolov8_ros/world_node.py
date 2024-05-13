import rclpy
from rclpy.node import Node
import message_filters

from yolov8_msgs.msg import DetectionInfo
from sensor_msgs.msg import CameraInfo, Image
from geometry_msgs.msg import Pose,PoseStamped

from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSReliabilityPolicy

import time
import tf2_ros
import transforms3d.quaternions as txq
import numpy as np
from cv_bridge import CvBridge


class WorldNode(Node):

    def __init__(self) -> None:
        super().__init__("world_node")
        self.camera_link = "camera_link"
        # self.serial_port = serial.Serial('/dev/ttyACM0', 115200)  # Modify port and baudrate as needed

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.cv_bridge = CvBridge()

        # params
        self.declare_parameter("depth_image_reliability",
                               QoSReliabilityPolicy.BEST_EFFORT)
        depth_image_qos_profile = QoSProfile(
            reliability=self.get_parameter(
                "depth_image_reliability").get_parameter_value().integer_value,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1
        )
         # pub
        self.pose_publisher = self.create_publisher(PoseStamped, 'person_position', 10)
        self.x = 0
        self.y = 0
        # self.name = ''
        self.depth = 0
        self.non_detected = True

        # subs
        self.depth_sub = message_filters.Subscriber(
            self, Image, "depth_image",
            qos_profile=depth_image_qos_profile)
        self.center_sub = message_filters.Subscriber(
            self, DetectionInfo, "center_point")
        
        self._synchronizer = message_filters.ApproximateTimeSynchronizer(
            (self.depth_sub, self.center_sub), 10, 0.5)
        self._synchronizer.registerCallback(self.world_publisher)

    def world_publisher(self, depth_msg: Image, point_msg: DetectionInfo):
        depth_frame = self.cv_bridge.imgmsg_to_cv2(depth_msg)
        self.x = int(point_msg.x)
        self.y = int(point_msg.y)
        self.name = point_msg.name
        self.depth = depth_frame[self.y][self.x]
        if self.x != 0:
            self.non_detected = False

    def pixel_to_camera_coordinates(self, x_pixel, y_pixel, depth, focal_length, image_center):
        x_camera = (x_pixel - image_center[0]) * depth / focal_length
        y_camera = (y_pixel - image_center[1]) * depth / focal_length
        z_camera = depth

        return np.array([x_camera, y_camera, z_camera])

    def run(self):
        while rclpy.ok():
            rclpy.spin_once(self)
            if not self.non_detected:
                # Convert pixel coordinates to camera coordinates
                image_center = [320.1998291 , 238.22215271]  # Assuming image center at (640, 360)
                focal_length = 381.98086548  # Focal length in pixels (example value)
                camera_coords = self.pixel_to_camera_coordinates(self.x, self.y, self.depth,
                                                                 focal_length, image_center)

                # Get the transform from camera_link to world
                try:
                    transform = self.tf_buffer.lookup_transform('camera_link', self.camera_link, rclpy.time.Time())
                    # transform = self.tf_buffer.lookup_transform('base_link', self.camera_link, rclpy.time.Time())
                    camera_position = np.array([transform.transform.translation.x,
                                                transform.transform.translation.y,
                                                transform.transform.translation.z])
                    camera_orientation = transform.transform.rotation

                    # Camera 보정 ( D435 카메라 중심과 Color 카메라의 위치가 일치하지 않음 )
                    if (camera_position[0] >= 0.1): camera_position[0] *= 0.6
                    elif (camera_position[0] >= 0.070): camera_position[0] -= 0.070
                    elif (camera_position[0] >= 0.035): camera_position[0] -= 0.035
                    elif (camera_position[0] > 0): camera_position[0] = 0.000
                    elif (camera_position[0] <= -0.1): camera_position[0] *= 0.8

                    if (abs(camera_position[1]) >= 0.1): camera_position[1] *= 0.8

                    # Convert quaternion to rotation matrix
                    R = self.quaternion_to_rotation_matrix(camera_orientation)

                    # Transform object position from camera to world coordinates
                    object_position_camera_frame = camera_coords
                    object_position_world_frame = np.dot(R, object_position_camera_frame) + camera_position

                    # Publish the object position in world coordinates
                    pose_msg = Pose()
                    pose_msg.position.x = object_position_world_frame[0]
                    pose_msg.position.y = object_position_world_frame[1]
                    pose_msg.position.z = object_position_world_frame[2]
                    self.get_logger().info(f' ({self.name})  x:{pose_msg.position.x} | y:{pose_msg.position.y} | z:{pose_msg.position.z} ')

                    self.publish_pose(pose_msg)  # Publish the transformed pose

                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                    print(f"Failed to lookup transform: {e}")

                self.non_detected = True  # Reset the detection flag after processing

            time.sleep(0.2)

    def quaternion_to_rotation_matrix(self, quaternion):
        # Convert quaternion to rotation matrix
        q = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
        R = txq.quat2mat(q)
        return R

    def publish_pose(self, pose_msg):
        # Publish the pose in world coordinates
        pose_stamped_msg = PoseStamped()
        pose_stamped_msg.pose = pose_msg
        pose_stamped_msg.header.frame_id = 'person_frame'
        self.pose_publisher.publish(pose_stamped_msg)

def main(args=None):
    rclpy.init(args=args)
    world_position = WorldNode()
    try:
        world_position.run()
    except KeyboardInterrupt:
        pass

    world_position.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
