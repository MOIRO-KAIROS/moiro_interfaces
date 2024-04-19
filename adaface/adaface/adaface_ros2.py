import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSReliabilityPolicy

import message_filters
from cv_bridge import CvBridge

from yolov8_msgs.msg import Detection
from yolov8_msgs.msg import DetectionArray
from sensor_msgs.msg._image import Image
# from visualization_msgs.msg import Marker
# from visualization_msgs.msg import MarkerArray

from .script.adaface import inference
'''
This Node publish data to ~yolov8_debug_node~
'''

class Adaface(Node):
  def __init__(self):
    super().__init__('adaface')
    self.get_logger().info('Adaface is now starting...')

    self._class_to_color = {}
    self.cv_bridge = CvBridge()
    
     # params
    self.declare_parameter("image_reliability",
                            QoSReliabilityPolicy.BEST_EFFORT)
    image_qos_profile = QoSProfile(
        reliability=self.get_parameter(
            "image_reliability").get_parameter_value().integer_value,
        history=QoSHistoryPolicy.KEEP_LAST,
        durability=QoSDurabilityPolicy.VOLATILE,
        depth=1
    )

    #pubs
    self._adaface_pub = self.create_publisher(DetectionArray, 'adaface_msg',10)

    ## subs
    # 이미지와 message를 subscribe
    tracking_sub = message_filters.Subscriber(
        self, DetectionArray, "tracking", qos_profile =10)
    image_sub = message_filters.Subscriber(
        self, Image, "image_raw", qos_profile=image_qos_profile)

    
    # 이미지와 message를 동기화
    self._synchronizer = message_filters.ApproximateTimeSynchronizer(
        (image_sub, tracking_sub), 10, 0.5)
    
    # Adaface_main 부름
    self._synchronizer.registerCallback(self.adaface_main)

  
  def adaface_main(self, img_msg: Image, tracking_msg: DetectionArray) -> None:
        self.get_logger().info('========================') 
        self.get_logger().info('Start face recognition!')
        self.get_logger().info('========================') 
        face_ids_for_frame = DetectionArray()
        face_ids_for_frame.header = img_msg.header

        # convert image for align
        cv_image = self.cv_bridge.imgmsg_to_cv2(img_msg)

        face_id_msg: Detection
        for face_id_msg in tracking_msg.detections:
          
          # 객체 이미지 위치 잡고 그걸 inference로 보낸다
          x1 = int(face_id_msg.bbox.center.position.x - face_id_msg.bbox.size.x / 2)
          y1 = int(face_id_msg.bbox.center.position.y - face_id_msg.bbox.size.y / 2)
          x2 = int(face_id_msg.bbox.center.position.x + face_id_msg.bbox.size.x / 2)
          y2 = int(face_id_msg.bbox.center.position.y + face_id_msg.bbox.size.y / 2)
          
          # score = face_id_msg.score
          # class_id = face_id_msg.class_id

          face_box, face_names = inference(cv_image[y1:y2,x1:x2])

          # Assume that one person box = one face
          face_id_msg.bbox.center.position.x = x1 + (face_box[0][2] + face_box[0][0])//2
          face_id_msg.bbox.size.x = face_box[0][2]- face_box[0][0]
          face_id_msg.bbox.center.position.y = y1 + (face_box[0][3] + face_box[0][1])//2
          face_id_msg.bbox.size.y = face_box[0][3]- face_box[0][1]
          face_id_msg.id = face_names

          face_ids_for_frame.detection.append(face_id_msg)

        # publish face information (id,bbox)
        self.get_logger().info('size [ x : {}, y : {}], face info [{}]'.format(face_id_msg.bbox.size.x ,face_id_msg.bbox.size.y,face_names)) # For Debugging
        self._adaface_pub.publish(face_ids_for_frame)

def main(args=None): 
  rclpy.init(args=None)
  node = Adaface()
  rclpy.spin(node)
  node.destroy_node()
  rclpy.shutdown()

if __name__ == '__main__':
  main()