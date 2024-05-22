import rclpy
from rclpy.node import Node
import cv2
import time
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class ImagePublisher(Node):
    def __init__(self):
        super().__init__("image_publisher")
        file_path = '/home/minha/moiro_ws/src/moiro_vision/adaface/adaface/script/video/iAM.mp4'
        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture(file_path)
        self.pub = self.create_publisher(Image, "video_topic", 10)

    def run(self):
        while(self.cap.isOpened()):
            ret, frame = self.cap.read() 
            if ret:
                self.pub.publish(self.bridge.cv2_to_imgmsg(frame,"rgb8"))
            else:
                self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            time.sleep(1/60) 

        self.cap.release()

def main(args=None):
    rclpy.init(args=None)
    ip = ImagePublisher()
    print("Publishing...")
    ip.run()

    ip.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()