#!/usr/bin/env python
# coding: utf-8
import os
import rospy
import rospkg
import threading
from astra_common import *
from geometry_msgs.msg import Twist
from car_msgs.msg import Position
from sensor_msgs.msg import CompressedImage, Image
from dynamic_reconfigure.server import Server
from dynamic_reconfigure.client import Client
from car_astra.cfg import ColorHSVConfig
from std_msgs.msg import  Int64

import numpy as np
import cv2 as cv
from cv_bridge import CvBridge, CvBridgeError


class Color_Identify:
    def __init__(self):
        rospy.init_node("colorHSV", anonymous=False)
        rospy.on_shutdown(self.cancel)
        print(1)
        self.index = 2
        self.Roi_init = ()
        self.hsv_range = ()

        self.circle = (0, 0, 0)
        self.point_pose = (0, 0, 0)
        self.dyn_update = True

        self.select_flags = False
        self.gTracker_state = False
        self.windows_name = 'frame'
        self.Track_state = 'identify'
        self.color = color_follow()
        self.cols, self.rows = 0, 0
        self.Mouse_XY = (0, 0)

        self.srart_flag=False
        Server(ColorHSVConfig, self.dynamic_reconfigure_callback)
        self.bridge = CvBridge()
        self.detection_flag=_rospy.Publisher("/detection_flag",Int64,queue_size=10)
        self.sub_img = rospy.Subscriber("/camera/rgb/image_raw", Image, self.Image_callback)
        self.start_flags = rospy.Subscriber("/start_type", Int64, self.start_call_back)
        print("OpenCV Version: ", cv.__version__)


    def start_call_back(self,data):
        self.srart_flag =data.data

    def dynamic_reconfigure_callback(self, config, level):
        self.hsv_range = ((config['Hmin'], config['Smin'], config['Vmin']),
                          (config['Hmax'], config['Smax'], config['Vmax']))
        return config

    def Image_callback(self, data):
            if self.srart_flag==0:         cv.destroyAllWindows()

            if not isinstance(data, Image) or self.srart_flag==0 : return
             
            try:
                frame = self.bridge.imgmsg_to_cv2(data, "bgr8") 
            except CvBridgeError as e:
                print (e)
            start = time.time()
            action = cv.waitKey(10) & 0xFF
            rgb_img, binary = self.process(frame, action)
            if self.circle[2]>40 and self.srart_flag: 
                print('检测到物品')
                self.detection_flag.publish(1)
            end = time.time()
            fps = 1 / (end - start)
            text = "FPS : " + str(int(fps))
            cv.putText(rgb_img, text, (30, 30), cv.FONT_HERSHEY_SIMPLEX, 0.6, (100, 200, 200), 1)
            if len(binary) != 0: 
                  cv.imshow(self.windows_name, ManyImgs(1, ([rgb_img, binary])))

            else: 
                  cv.imshow(self.windows_name, rgb_img)


    def onMouse(self, event, x, y, flags, param):
        if event == 1:
            self.Track_state = 'init'
            self.select_flags = True
            self.Mouse_XY = (x, y)
        if event == 4:
            self.select_flags = False
            self.Track_state = 'mouse'
        if self.select_flags == True:
            self.cols = min(self.Mouse_XY[0], x), min(self.Mouse_XY[1], y)
            self.rows = max(self.Mouse_XY[0], x), max(self.Mouse_XY[1], y)
            self.Roi_init = (self.cols[0], self.cols[1], self.rows[0], self.rows[1])

    def process(self, rgb_img, action):
        rgb_img = cv.resize(rgb_img, (640, 480))
        binary = []
        if action == 32: self.Track_state = 'tracking'
        elif action == ord('i') or action == ord('I'): self.Track_state = "identify"
        elif action == ord('r') or action == ord('R'): self.Reset()
        elif action == ord('q') or action == ord('Q'): self.cancel()
        if self.Track_state == 'init':
            cv.namedWindow(self.windows_name, cv.WINDOW_AUTOSIZE)
            cv.setMouseCallback(self.windows_name, self.onMouse, 0)
            if self.select_flags == True:
                cv.line(rgb_img, self.cols, self.rows, (255, 0, 0), 2)
                cv.rectangle(rgb_img, self.cols, self.rows, (0, 255, 0), 2)
                if self.Roi_init[0] != self.Roi_init[2] and self.Roi_init[1] != self.Roi_init[3]:
                    rgb_img, self.hsv_range = self.color.Roi_hsv(rgb_img, self.Roi_init)
                    self.gTracker_state = True
                    self.dyn_update = True
                else: self.Track_state = 'init'
        elif self.Track_state == "identify":
             self.hsv_range= ((55, 129, 89), (125, 253, 255))

        if self.Track_state != 'init':
            if len(self.hsv_range) != 0:
                rgb_img, binary, self.circle = self.color.object_follow(rgb_img, self.hsv_range)
                if self.dyn_update == True:
                    params = {'Hmin': self.hsv_range[0][0], 'Hmax': self.hsv_range[1][0],
                              'Smin': self.hsv_range[0][1], 'Smax': self.hsv_range[1][1],
                              'Vmin': self.hsv_range[0][2], 'Vmax': self.hsv_range[1][2]}
                    self.dyn_update = False
        return rgb_img, binary

    def cancel(self):
        self.Reset()
        self.sub_img.unregister()
        print("Shutting down this node.")
        cv.destroyAllWindows()

    def Reset(self):
        self.hsv_range = ()
        self.circle = (0, 0, 0)
        self.Mouse_XY = (0, 0)
        self.Track_state = 'init'
        rospy.loginfo("init succes!!!")


if __name__ == '__main__':
    astra_tracker = Color_Identify()
    rospy.spin()
    cv.destroyAllWindows()

