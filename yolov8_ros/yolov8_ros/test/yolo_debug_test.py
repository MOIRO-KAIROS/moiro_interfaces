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


import numpy as np
import cv2
from typing import Tuple

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSReliabilityPolicy

import message_filters
from cv_bridge import CvBridge
from ultralytics.utils.plotting import Annotator, colors

from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from yolov8_msgs.msg import BoundingBox2D
from yolov8_msgs.msg import KeyPoint2D
from yolov8_msgs.msg import KeyPoint3D
from yolov8_msgs.msg import Detection
from yolov8_msgs.msg import DetectionArray
from yolov8_msgs.msg import FaceBox
from yolov8_msgs.msg import FaceBoxArray
from yolov8_msgs.action import DetectionAction 


class DebugNode(Node):

    def __init__(self) -> None:
        super().__init__("debug_node")
        self._class_to_color = {}
        self._face_name = {}
        self._face_id = {}
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
        # action
        self._action_client = ActionClient(self, DetectionAction, 'person_action')

        # pubs
        self._dbg_pub = self.create_publisher(Image, "dbg_image", 10)

        # subs
        image_sub = message_filters.Subscriber(
            self, Image, "image_raw", qos_profile=image_qos_profile)
        detections_sub = message_filters.Subscriber(
            self, DetectionArray, "detections", qos_profile=10)
        ###
        # face_sub = message_filters.Subscriber(
        #     self, FaceBoxArray, "/adaface/adaface_msg",qos_profile=10)
        ###
        # self._synchronizer = message_filters.ApproximateTimeSynchronizer(
        #     (image_sub, detections_sub,face_sub), 10, 0.5)
        self._synchronizer = message_filters.ApproximateTimeSynchronizer((image_sub, detections_sub), 10, 0.5)
        self._synchronizer.registerCallback(self.detections_cb)
    
    def send_goal(self, goal, img):
        person_bbox = DetectionAction.Goal()
        person_bbox.detection = goal
        person_bbox.image = img
        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(person_bbox)
        self._send_goal_future.add_done_callback(self.goal_response_callback)
        return self._send_goal_future 
    
    def goal_response_callback(self, future):
        goal_handle = DetectionAction.Result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return
        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self):
        person_name = DetectionAction.Result().name
        self.get_logger().info('Person Name: {0}'.format(person_name))
        # rclpy.shutdown()

    def draw_box(self, cv_image: np.array, detection: Detection, face_detection: FaceBox) -> np.array:
        # get detection info
        score = detection.score
        box_msg: BoundingBox2D = detection.bbox

        min_pt = (round(box_msg.center.position.x - box_msg.size.x / 2.0),
            round(box_msg.center.position.y - box_msg.size.y / 2.0))
        max_pt = (round(box_msg.center.position.x + box_msg.size.x / 2.0),
            round(box_msg.center.position.y + box_msg.size.y / 2.0))
        
        # write text
        font = cv2.FONT_HERSHEY_COMPLEX
        pos = (min_pt[0] + 25, max_pt[1] - 25)
        label = "({}) {} ({:.3f})".format(detection.id, detection.name, score)

        if face_detection != None:
            face_box_msg: BoundingBox2D = face_detection.bbox

            min_face = (round(face_box_msg.center.position.x - face_box_msg.size.x / 2.0),
                  round(face_box_msg.center.position.y - face_box_msg.size.y / 2.0))
            max_face = (round(face_box_msg.center.position.x + face_box_msg.size.x / 2.0),
                  round(face_box_msg.center.position.y + face_box_msg.size.y / 2.0))
            
            cv2.rectangle(cv_image, min_face, max_face, (255,255,255), 2)
            
            cv2.putText(cv_image, label, pos, font,
                    0.6, (0,255,0), 1, cv2.LINE_AA)
        else:       
            if detection.id in self._face_name:
               detection.name = self._face_name[detection.id]
               label = "{} ({:.3f})".format(detection.name, score)
               cv2.putText(cv_image, label, pos, font,
                        0.6, (255,255,255), 1, cv2.LINE_AA)
            else:
                # label = "{} ({:.3f})".format(detection.name, score)
                cv2.putText(cv_image, label, pos, font,
                        0.6, (135, 204, 255), 1, cv2.LINE_AA)

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
        self.get_logger().info('Person id : {} | depth point {}'.format(str(detection.id),[sh_point[0],sh_point[1]]))         

        return cv_image

    def detections_cb(self, img_msg: Image, detection_msg: DetectionArray) -> None:
        cv_image = self.cv_bridge.imgmsg_to_cv2(img_msg,"rgb8")

        detection: Detection
        face_detection : FaceBox

        for i, detection in enumerate(detection_msg.detections):
            if detection:
                # 해당 사람 박스에 이름이 부여된 적이 없으며,
                if detection.id not in self._face_name:
                    # Client face recognition
                    face_detection = self.send_goal(detection, img_msg)
                    # 감지된 얼굴이 unknown이 아니고, 감지된 얼굴이 이미 등장한 인물 아닐때
                    if face_detection.name != 'unknown' and face_detection.name not in self._face_id:
                        detection.score = face_detection.score
                        self._face_name[detection.id] = face_detection.name
                        self._face_id[face_detection.name] = detection.id
                        detection.name = face_detection.name
                else:
                    detection.name = self._face_name[detection.id]
            
            cv_image = self.draw_mask(cv_image, detection)
            cv_image = self.draw_keypoints(cv_image, detection)
            cv_image = self.draw_box(cv_image, detection,face_detection)

        # publish dbg image
        self._dbg_pub.publish(self.cv_bridge.cv2_to_imgmsg(cv_image,encoding=img_msg.encoding))


def main():
    rclpy.init()
    node = DebugNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
