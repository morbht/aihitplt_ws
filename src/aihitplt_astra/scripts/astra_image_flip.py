#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import cv2 as cv
import numpy as np
from sensor_msgs.msg import Image, CompressedImage

def topic(msg):
    if not isinstance(msg, Image):
        return
    
    try:
        # Convert ROS Image to OpenCV format without cv_bridge
        frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
        
        # Convert RGB to BGR if needed
        if msg.encoding == 'rgb8':
            frame = cv.cvtColor(frame, cv.COLOR_RGB2BGR)
        
        # Process image
        frame = cv.resize(frame, (640, 480))
        frame = cv.flip(frame, 1)
        
        # Convert back to ROS Image message
        img_msg = Image()
        img_msg.header = msg.header
        img_msg.height = frame.shape[0]
        img_msg.width = frame.shape[1]
        img_msg.encoding = 'bgr8'
        img_msg.is_bigendian = False
        img_msg.step = frame.shape[1] * 3  # 3 bytes per pixel for BGR
        img_msg.data = frame.tobytes()
        
        pub_img.publish(img_msg)
        
    except Exception as e:
        rospy.logerr("Error processing image: %s", str(e))

def compressed_topic(msg):
    if not isinstance(msg, CompressedImage):
        return
    
    try:
        # Convert compressed image to OpenCV format
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv.imdecode(np_arr, cv.IMREAD_COLOR)
        
        # Process image
        frame = cv.resize(frame, (640, 480))
        frame = cv.flip(frame, 1)
        
        # Create CompressedImage message
        com_msg = CompressedImage()
        com_msg.header = msg.header
        com_msg.format = 'jpeg'
        _, img_data = cv.imencode('.jpg', frame)
        com_msg.data = img_data.tobytes()
        
        pub_comimg.publish(com_msg)
        
    except Exception as e:
        rospy.logerr("Error processing compressed image: %s", str(e))

if __name__ == '__main__':
    rospy.init_node("astra_image_flip", anonymous=False)
    sub_img = rospy.Subscriber("/camera/rgb/image_raw", Image, topic)
    pub_img = rospy.Publisher("/camera/rgb/image_flip", Image, queue_size=10)
    sub_comimg = rospy.Subscriber("/camera/rgb/image_raw/compressed", CompressedImage, compressed_topic)
    pub_comimg = rospy.Publisher("/camera/rgb/image_flip/compressed", CompressedImage, queue_size=10)
    rospy.spin()
