#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import cv2 as cv
import numpy as np
from sensor_msgs.msg import Image

encoding = ['16UC1', '32FC1']

def topic(msg):
    if not isinstance(msg, Image):
        return
    
    try:
        # Convert ROS Image message to OpenCV image without cv_bridge
        if msg.encoding == '16UC1':
            # For 16-bit unsigned integers (common depth format)
            frame = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width)
        elif msg.encoding == '32FC1':
            # For 32-bit floating point
            frame = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width)
        else:
            rospy.logwarn("Unsupported encoding: %s", msg.encoding)
            return

        # Standardize the input image size
        frame = cv.resize(frame, (640, 480))
        
        # Optional: Print depth values (commented out as in original)
        # h, w = frame.shape[:2]
        # for row in range(h):
        #     for col in range(w):
        #         print ("x: {},y:{},z: {}".format(row, col, frame[row, col] / 1000.0))
        
        # Display the depth image
        cv.imshow("depth_image", frame)
        cv.waitKey(10)
        
    except Exception as e:
        rospy.logerr("Error processing depth image: %s", str(e))

if __name__ == '__main__':
    rospy.init_node("astra_depth_image_py", anonymous=False)
    sub = rospy.Subscriber("/camera/depth/image_raw", Image, topic)
    rospy.spin()
