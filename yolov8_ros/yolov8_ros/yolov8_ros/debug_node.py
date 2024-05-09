# Copyright (C) 2023  Miguel Ángel González Santamarta

# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.


import cv2
import numpy as np
from typing import Tuple

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSReliabilityPolicy

import message_filters
from cv_bridge import CvBridge
from ultralytics.utils.plotting import Annotator, colors
import time

from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from yolov8_msgs.msg import BoundingBox2D
from yolov8_msgs.msg import KeyPoint2D
from yolov8_msgs.msg import Detection
from yolov8_msgs.msg import DetectionArray, DetectionInfo
from yolov8_msgs.srv import Person

class DebugNode(Node):

    def __init__(self) -> None:
        super().__init__("debug_node")
        self.cv_bridge = CvBridge()
        self.shoulder_center = {}

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

        self.declare_parameter("person_name", 'Unintialized')
        self.person_name = self.get_parameter("person_name").get_parameter_value().string_value

        self.get_logger().info('-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-') 
        self.get_logger().error(f'The Person Who You Want To Detect Is {self.person_name} !!!!')
        self.get_logger().info('-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-')
        # pubs
        self._dbg_pub = self.create_publisher(Image, "dbg_image", 10)
        self._center_pub = self.create_publisher(DetectionInfo, "center_point", 10)

        # subs
        image_sub = message_filters.Subscriber(
            self, Image, "image_raw", qos_profile=image_qos_profile)
        detections_sub = message_filters.Subscriber(
            self, DetectionArray, "detections", qos_profile=10)
        self._synchronizer = message_filters.ApproximateTimeSynchronizer(
            (image_sub, detections_sub), 10, 0.5)
        
        #self._synchronizer = message_filters.ApproximateTimeSynchronizer((image_sub, detections_sub), 10, 0.5)
        self._synchronizer.registerCallback(self.detections_cb)

        # services
        self._srv = self.create_service(Person, 'person_name', self.person_setting)
    
    def person_setting(self, req: Person.Request, res: Person.Response ) -> Person.Response:
        self.person_name = req.person_name
        res.success_name = self.person_name
        return res

    def draw_box(self, cv_image: np.array, detection: Detection) -> np.array:
        # get detection info
        box_msg: BoundingBox2D = detection.bbox

        min_pt = (round(box_msg.center.position.x - box_msg.size.x / 2.0),
            round(box_msg.center.position.y - box_msg.size.y / 2.0))
        max_pt = (round(box_msg.center.position.x + box_msg.size.x / 2.0),
            round(box_msg.center.position.y + box_msg.size.y / 2.0))
        
        # write text
        font = cv2.FONT_HERSHEY_COMPLEX
        pos = (min_pt[0] + 25, max_pt[1] - 25)
        
        if detection.facebox.isdetect: # 얼굴 좌표가 없다 = 얼굴 이름도 없다 하지만 바운딩박스의 이름은 array 있으면 찾을 수 있다.
            self.get_logger().info('\033[93m =================================================== \033[0m')
            self.get_logger().info('\033[93m 얼굴 좌표 있을 때: {}  \033[0m'.format(detection)) # For Debugging
            self.get_logger().info('\033[93m =================================================== \033[0m')
            
            face_box_msg: BoundingBox2D = detection.facebox.bbox

            min_face = (round(face_box_msg.center.position.x - face_box_msg.size.x / 2.0),
                  round(face_box_msg.center.position.y - face_box_msg.size.y / 2.0))
            max_face = (round(face_box_msg.center.position.x + face_box_msg.size.x / 2.0),
                  round(face_box_msg.center.position.y + face_box_msg.size.y / 2.0))
            
            cv2.rectangle(cv_image, min_face, max_face, (255,255,255), 2)

        label = "({}) {}".format(detection.id, detection.facebox.name)
        if detection.facebox.name == "unknown":
            cv2.putText(cv_image, label, pos, font,
                0.6, (255,255,255), 1, cv2.LINE_AA)
        elif detection.facebox.name == "no face":
            cv2.putText(cv_image, label, pos, font,
                0.6, (135, 204, 255), 1, cv2.LINE_AA)
        else:
            cv2.putText(cv_image, label, pos, font,
                0.6, (0,255,0), 1, cv2.LINE_AA)

        # draw person box
        cv2.rectangle(cv_image, min_pt, max_pt, (135, 204, 255), 2)

        return cv_image

    def draw_mask(self, cv_image: np.array, detection: Detection) -> np.array:

        mask_msg = detection.mask
        mask_array = np.array([[int(ele.x), int(ele.y)]
                              for ele in mask_msg.data])

        if mask_msg.data:
            layer = cv_image.copy()
            layer = cv2.fillPoly(layer, pts=[mask_array], color=(135, 204, 255))
            cv2.addWeighted(cv_image, 0.4, layer, 0.6, 0, cv_image)
            cv_image = cv2.polylines(cv_image, [mask_array], isClosed=True,
                                     color=(135, 204, 255), thickness=2, lineType=cv2.LINE_AA)
        return cv_image

    def draw_keypoints(self, cv_image: np.array, detection: Detection) -> np.array:

        keypoints_msg = detection.keypoints

        ann = Annotator(cv_image)
        sh_point = [0,0]
        kp: KeyPoint2D
        for kp in keypoints_msg.data:
            color_k = [int(x) for x in ann.kpt_color[kp.id - 1]
                       ] if len(keypoints_msg.data) == 17 else colors(kp.id - 1)

            cv2.circle(cv_image, (int(kp.point.x), int(kp.point.y)),
                       5, color_k, -1, lineType=cv2.LINE_AA)
            #### Shoulder middle point!
            if str(kp.id) == '7' or str(kp.id) == '6':
                sh_point[0] += kp.point.x
                sh_point[1] += kp.point.y
        sh_point[0],sh_point[1] = sh_point[0]//2,sh_point[1]//2
        self.shoulder_center[detection.id] = [sh_point[0],sh_point[1]]
        cv2.circle(cv_image, (int(sh_point[0]), int(sh_point[1])),
                       5, (255,255,255), -1, lineType=cv2.LINE_AA)

        return cv_image , sh_point


    def detections_cb(self, img_msg: Image, face_detection_msg: DetectionArray) -> None:
        start = time.time()

        cv_image = self.cv_bridge.imgmsg_to_cv2(img_msg,"rgb8")
        detection: Detection
        
        for i, detection in enumerate(face_detection_msg.detections):
            cv_image, sh_point = self.draw_keypoints(cv_image, detection)
            # When the input person is detected
            # if self._face_cache[detection.id] == self.person_name:
            #     person_center = DetectionInfo()
            #     person_center.header = face_detection_msg.header
            #     person_center.x = float(sh_point[0])
            #     person_center.y = float(sh_point[1])
            #     self._center_pub.publish(person_center)
                # self.get_logger().info('Person name : {} | depth point {}'.format(str(detection.name),[sh_point[0],sh_point[1]]))         
            cv_image = self.draw_box(cv_image, detection)
            
        # publish dbg image
        self._dbg_pub.publish(self.cv_bridge.cv2_to_imgmsg(cv_image, encoding=img_msg.encoding))

        end = time.time()
        self.get_logger().info(f"\033[93m >>>>>>>>>>>>>>>>>>>>>>>>>> {end - start:.5f} sec >>>>>>>>>>>>>>>>>>>>>>>>>> \033[0m")



def main():
    rclpy.init()
    node = DebugNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
