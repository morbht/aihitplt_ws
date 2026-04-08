#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import cv2 as cv
import numpy as np
from sensor_msgs.msg import Image


def topic(msg):
    if not isinstance(msg, Image):
        return
    
    # Convert ROS Image message to OpenCV image without cv_bridge
    try:
        # Assuming 'bgr8' encoding
        frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
        
        # If the image is in RGB format, convert to BGR
        if msg.encoding == 'rgb8':
            frame = cv.cvtColor(frame, cv.COLOR_RGB2BGR)
        
        # Standardize the input image size
        frame = cv.resize(frame, (640, 480))
        cv.imshow("color_image", frame)
        cv.waitKey(10)
    except Exception as e:
        rospy.logerr("Error processing image: %s", str(e))


if __name__ == '__main__':
    rospy.init_node("astra_rgb_image_py")
    sub = rospy.Subscriber("/camera/rgb/image_raw", Image, topic)
    rospy.spin()
