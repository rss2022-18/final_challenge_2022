import cv2
import rospy

import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from detector import StopSignDetector

class SignDetector:
    def __init__(self):
        self.detector = StopSignDetector()
        self.publisher = rospy.Publisher("/stop_sign_location_px", StopSignLocationPixel, queue_size=10)
        self.subscriber = rospy.Subscriber("/zed/zed_node/rgb/image_rect_color", Image, self.callback)

    def callback(self, img_msg):
        # Process image without CV Bridge
        np_img = np.frombuffer(img_msg.data, dtype=np.uint8).reshape(img_msg.height, img_msg.width, -1)
        bgr_img = np_img[:,:,:-1]
        rgb_img = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2RGB)
        ssd = StopSignDetector()

        is_stop,rectangle_coord = ssd.predict(rgb_img)
        stop_msg = Float32MultiArray()
        stop_msg.label = "Stop_data"
        stop_msg.size = 5
        stop_msg.stride = 1
        stop_msg.data = [is_stop,rectangle_coord[0],rectangle_coord[1],rectangle_coord[2],rectangle_coord[3]]
        self.publisher.publish(stop_msg)

if __name__=="__main__":
    rospy.init_node("stop_sign_detector")
    detect = SignDetector()
    rospy.spin()
