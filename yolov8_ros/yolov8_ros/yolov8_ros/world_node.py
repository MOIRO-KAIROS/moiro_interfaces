import rclpy
from rclpy.node import Node
import message_filters
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from yolov8_msgs.msg import DetectionInfo
from geometry_msgs.msg import PoseStamped, TransformStamped, Pose
import transforms3d.quaternions as txq
import numpy as np
import tf2_ros

class WorldNode(Node):

    def __init__(self):
        super().__init__('world_node')

        self.camera_link = 'camera_link'
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        ###
        self.static_broadcaster = tf2_ros.StaticTransformBroadcaster(self)

        # Define the static transform from base_link to camera_link
        self.static_transform_stamped = TransformStamped()
        self.static_transform_stamped.header.stamp = self.get_clock().now().to_msg()
        self.static_transform_stamped.header.frame_id = 'base_link'
        self.static_transform_stamped.child_frame_id = 'camera_link'

        # Define translation (x, y, z)
        self.static_transform_stamped.transform.translation.x = 0.1  # Example translation in x-axis
        self.static_transform_stamped.transform.translation.y = 0.0  # Example translation in y-axis
        self.static_transform_stamped.transform.translation.z = 0.0  # Example translation in z-axis

        # Define rotation (quaternion: x, y, z, w)
        self.static_transform_stamped.transform.rotation.x = 0.0
        self.static_transform_stamped.transform.rotation.y = 0.0
        self.static_transform_stamped.transform.rotation.z = 0.0
        self.static_transform_stamped.transform.rotation.w = 1.0  # No rotation in this example

        # Publish the static transform
        self.static_broadcaster.sendTransform(self.static_transform_stamped)
        ###

        # Parameters
        self.declare_parameter('depth_image_reliability', 1)  # Default to BEST_EFFORT
        reliability = self.get_parameter('depth_image_reliability').get_parameter_value().integer_value
        self.depth_image_qos_profile = rclpy.qos.QoSProfile(depth=1, reliability=reliability)

        # Publishers
        self.pose_publisher = self.create_publisher(PoseStamped, 'person_position', 10)

        # Subscribers
        self.depth_sub = message_filters.Subscriber(self, Image, 'depth_image', qos_profile=self.depth_image_qos_profile)
        self.center_sub = message_filters.Subscriber(self, DetectionInfo, 'center_point')

        # ApproximateTimeSynchronizer
        self._synchronizer = message_filters.ApproximateTimeSynchronizer(
            [self.depth_sub, self.center_sub], queue_size=20, slop=0.5)
        self._synchronizer.registerCallback(self.process_detection)

    def process_detection(self, depth_msg: Image, point_msg: DetectionInfo):
        bridge = CvBridge()
        depth_frame = bridge.imgmsg_to_cv2(depth_msg)

        u = int(point_msg.x)
        v = int(point_msg.y)
        depth = depth_frame[v][u]  # Access depth at (v, u) due to OpenCV array indexing

        if depth != 0:
            # Convert pixel coordinates to camera coordinates
            image_center = [depth_msg.width / 2.0, depth_msg.height / 2.0]
            focal_length = 381.98  # Focal length in pixels (example value)
            camera_coords = self.pixel_to_camera_coordinates(u, v, depth, focal_length, image_center)

            # Get transform from camera_link to base_link
            try:
                # transform = self.tf_buffer.lookup_transform('camera_link', self.camera_link, rclpy.time.Time())
                transform = self.tf_buffer.lookup_transform('base_link', self.camera_link, rclpy.time.Time())
                camera_position = np.array([transform.transform.translation.x,
                                            transform.transform.translation.y,
                                            transform.transform.translation.z])
                camera_orientation = transform.transform.rotation

                # Convert quaternion to rotation matrix
                R = self.quaternion_to_rotation_matrix(camera_orientation)

                # Transform camera coordinates to world coordinates
                object_position_camera_frame = camera_coords
                object_position_world_frame = np.dot(R, object_position_camera_frame) + camera_position

                # Publish the object position in world coordinates
                pose_msg = PoseStamped()
                pose_msg.header.stamp = self.get_clock().now().to_msg()
                pose_msg.header.frame_id = 'camera_link'
                pose_msg.pose.position.y = object_position_world_frame[0] / 1000.0
                pose_msg.pose.position.z = object_position_world_frame[1] / 1000.0
                pose_msg.pose.position.x = object_position_world_frame[2] / 1000.0
                pose_msg.pose.orientation.w = 1.0
                self.get_logger().info(f'depth : {depth} x:{pose_msg.pose.position.x} | y:{pose_msg.pose.position.y} | z:{pose_msg.pose.position.z} ')
                self.pose_publisher.publish(pose_msg)

            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                self.get_logger().error(f"Failed to lookup transform: {e}")

    def pixel_to_camera_coordinates(self, u, v, depth, focal_length, image_center):
        z_camera = depth
        x_camera = (u - image_center[0]) * depth / focal_length
        y_camera = (v - image_center[1]) * depth / focal_length
        return np.array([x_camera, y_camera, z_camera])

    def quaternion_to_rotation_matrix(self, quaternion):
        q = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
        return txq.quat2mat(q)

def main(args=None):
    rclpy.init(args=args)
    world_node = WorldNode()
    try:
        rclpy.spin(world_node)
    except KeyboardInterrupt:
        pass

    world_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
