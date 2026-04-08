#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import base64
import cv2 as cv
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from aihitplt_msgs.msg import Image_Msg

class msgToimg:
    def __init__(self):
        rospy.init_node("msgToimg", anonymous=False)
        rospy.on_shutdown(self.cancel)
        self.bridge = CvBridge()
        # 初始化为None，在回调中动态创建
        self.img = None
        self.img_flip = rospy.get_param("~img_flip", False)
        self.image_sub = rospy.Subscriber("Detect/image_msg", Image_Msg, self.image_sub_callback)
        self.pub_img = rospy.Publisher("yoloDetect/image", Image, queue_size=10)

    def image_sub_callback(self, data):
        if not isinstance(data, Image_Msg): 
            return
        
        try:
            # 将自定义图像消息转化为图像
            # Convert custom image messages to images
            image = np.ndarray(shape=(data.height, data.width, data.channels), 
                              dtype=np.uint8,
                              buffer=base64.b64decode(data.data))
            
            # 首先将BGR转换为RGB
            image_rgb = cv.cvtColor(image, cv.COLOR_BGR2RGB)
            
            # 然后调整图像大小到目标尺寸
            self.img = cv.resize(image_rgb, (640, 480))
            
            # 如果需要翻转图像
            if self.img_flip == True: 
                self.img = cv.flip(self.img, 1)
            
            # opencv mat -> ros msg
            msg = self.bridge.cv2_to_imgmsg(self.img, "bgr8")
            self.pub_img.publish(msg)
            
        except Exception as e:
            rospy.logerr(f"Error in image_sub_callback: {str(e)}")

    def cancel(self):
        self.image_sub.unregister()
        self.pub_img.unregister()

if __name__ == '__main__':
    msgToimg()
    rospy.spin()