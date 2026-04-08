#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os,sys
import rospy
import cv2
import time
import numpy as np
sys.path.insert(0, '/opt/ros/' + os.environ['ROS_DISTRO'] + '/lib/python3/dist-packages/')
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

def img_callback(msg):
    bridge = CvBridge()
    try:
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="mono16")
    except CvBridgeError as e:
        rospy.logerr("CvBridge Error: {0}".format(e))
        return

    img = cv2.normalize(cv_image, None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8U)
    try:
        pub.publish(bridge.cv2_to_imgmsg(img, encoding="mono8"))
    except CvBridgeError as e:
        rospy.logerr("CvBridge Error: {0}".format(e))

def main():
    rospy.init_node("IR_transform", anonymous=True)
    rospy.Subscriber("/camera/ir/image", Image, img_callback)
    global pub
    pub = rospy.Publisher("/camera/ir/image_mono8", Image, queue_size=1)
    rospy.spin()

if __name__ == "__main__":
    main()

