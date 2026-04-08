#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import cv2 as cv
import os
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from datetime import datetime

class ImageSaver:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.image_callback)
        self.save_folder = os.path.expanduser("~/save_image")

    def image_callback(self, msg):
        if not isinstance(msg, Image):
            return
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        frame = cv.resize(frame, (640, 480))
        cv.imshow("color_image", frame)
        key = cv.waitKey(10)
        if key == ord('s'):
            self.save_image(frame)
        elif key == ord('q'):
            rospy.signal_shutdown("User requested shutdown")

    def save_image(self, frame):
        if not os.path.exists(self.save_folder):
            os.makedirs(self.save_folder)
        current_time = datetime.now().strftime("%Y%m%d_%H%M%S")
        image_name = os.path.join(self.save_folder, "color_image_{}.jpg".format(current_time))
        cv.imwrite(image_name, frame)
        print("Image saved: {}".format(image_name))


if __name__ == '__main__':
    rospy.init_node("save_image")
    image_saver = ImageSaver()
    rospy.spin()
