#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import cv2 as cv
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class ImageProcessor:
    def __init__(self):
        self.bridge = CvBridge()
        self.img_flip = rospy.get_param("~img_flip", False)
        
        # RGB图像订阅和发布
        self.sub_rgb = rospy.Subscriber("/camera/rgb/image_raw", Image, self.rgb_callback)
        self.pub_rgb = rospy.Publisher("/image", Image, queue_size=10)
        
        # 深度图订阅和发布（新增）
        self.sub_depth = rospy.Subscriber("/camera/depth/image_raw", Image, self.depth_callback)
        self.pub_depth = rospy.Publisher("/depth_image", Image, queue_size=10)

    def rgb_callback(self, msg):
        """处理RGB图像"""
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            frame = cv.resize(frame, (640, 480))
            if self.img_flip:
                frame = cv.flip(frame, 1)
            self.pub_rgb.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))
        except Exception as e:
            rospy.logerr("RGB处理错误: %s" % str(e))

    def depth_callback(self, msg):
        """处理深度图（新增）"""
        try:
            # 转换16UC1深度图
            depth_frame = self.bridge.imgmsg_to_cv2(msg, "16UC1")
            
            # 可选：归一化并转换为伪彩色（便于可视化）
            depth_colormap = cv.applyColorMap(
                cv.convertScaleAbs(depth_frame, alpha=0.03), 
                cv.COLORMAP_JET
            )
            
            # 调整大小和翻转（与RGB同步）
            depth_colormap = cv.resize(depth_colormap, (640, 480))
            if self.img_flip:
                depth_colormap = cv.flip(depth_colormap, 1)
                
            # 发布深度图（可选择发布原始数据或伪彩色）
            self.pub_depth.publish(
                self.bridge.cv2_to_imgmsg(depth_colormap, "bgr8")  # 发布伪彩色
                # 或者发布原始数据（需订阅端支持16UC1）：
                # self.bridge.cv2_to_imgmsg(depth_frame, "16UC1")
            )
        except Exception as e:
            rospy.logerr("深度图处理错误: %s" % str(e))

if __name__ == '__main__':
    rospy.init_node("pub_img_processor")
    processor = ImageProcessor()
    rospy.spin()