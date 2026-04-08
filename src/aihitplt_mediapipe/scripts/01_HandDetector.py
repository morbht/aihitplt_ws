#!/usr/bin/env python3
# encoding: utf-8
import rospy
import time
import cv2 as cv
import numpy as np
import mediapipe as mp
from geometry_msgs.msg import Point
from aihitplt_msgs.msg import PointArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class HandDetector:
    def __init__(self, mode=False, maxHands=2, detectorCon=0.5, trackCon=0.5):
        self.mpHand = mp.solutions.hands
        self.mpDraw = mp.solutions.drawing_utils
        self.hands = self.mpHand.Hands(
            static_image_mode=mode,
            max_num_hands=maxHands,
            min_detection_confidence=detectorCon,
            min_tracking_confidence=trackCon)
        
        # ROS相关初始化
        self.bridge = CvBridge()
        self.current_frame = None
        self.frame_received = False
        
        # 发布者
        self.pub_point = rospy.Publisher('/mediapipe/points', PointArray, queue_size=1000)
        
        # 订阅者 - 订阅相机话题
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.image_callback)
        
        self.lmDrawSpec = mp.solutions.drawing_utils.DrawingSpec(color=(0, 0, 255), thickness=-1, circle_radius=6)
        self.drawSpec = mp.solutions.drawing_utils.DrawingSpec(color=(0, 255, 0), thickness=2, circle_radius=2)
        
        rospy.loginfo("Hand detector initialized, waiting for camera images...")

    def image_callback(self, data):
        """相机图像回调函数"""
        try:
            # 将ROS图像消息转换为OpenCV格式
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.current_frame = cv_image
            self.frame_received = True
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {}".format(e))

    def pubHandsPoint(self, draw=True):
        """处理图像并发布手部关键点"""
        pointArray = PointArray()
        
        if self.current_frame is None:
            return None, None
            
        img = np.zeros(self.current_frame.shape, np.uint8)
        img_RGB = cv.cvtColor(self.current_frame, cv.COLOR_BGR2RGB)
        self.results = self.hands.process(img_RGB)
        
        if self.results.multi_hand_landmarks:
            for i in range(len(self.results.multi_hand_landmarks)):
                if draw: 
                    self.mpDraw.draw_landmarks(self.current_frame, 
                                             self.results.multi_hand_landmarks[i], 
                                             self.mpHand.HAND_CONNECTIONS, 
                                             self.lmDrawSpec, 
                                             self.drawSpec)
                self.mpDraw.draw_landmarks(img, 
                                         self.results.multi_hand_landmarks[i], 
                                         self.mpHand.HAND_CONNECTIONS, 
                                         self.lmDrawSpec, 
                                         self.drawSpec)
                for id, lm in enumerate(self.results.multi_hand_landmarks[i].landmark):
                    point = Point()
                    point.x, point.y, point.z = lm.x, lm.y, lm.z
                    pointArray.points.append(point)
        
        self.pub_point.publish(pointArray)
        return self.current_frame, img

    def frame_combine(self, frame, src):
        """合并两个图像"""
        if frame is None or src is None:
            return None
            
        if len(frame.shape) == 3:
            frameH, frameW = frame.shape[:2]
            srcH, srcW = src.shape[:2]
            dst = np.zeros((max(frameH, srcH), frameW + srcW, 3), np.uint8)
            dst[:, :frameW] = frame[:, :]
            dst[:, frameW:] = src[:, :]
        else:
            src = cv.cvtColor(src, cv.COLOR_BGR2GRAY)
            frameH, frameW = frame.shape[:2]
            imgH, imgW = src.shape[:2]
            dst = np.zeros((frameH, frameW + imgW), np.uint8)
            dst[:, :frameW] = frame[:, :]
            dst[:, frameW:] = src[:, :]
        return dst

if __name__ == '__main__':
    rospy.init_node('handDetector', anonymous=True)
    
    # 创建手部检测器
    hand_detector = HandDetector(maxHands=2)
    
    # 等待第一帧图像
    rospy.loginfo("Waiting for first camera image...")
    while not hand_detector.frame_received and not rospy.is_shutdown():
        rospy.sleep(0.1)
    
    if rospy.is_shutdown():
        exit(0)
        
    rospy.loginfo("Camera image received, starting hand detection...")
    
    pTime = cTime = 0
    rate = rospy.Rate(30)  # 30Hz
    
    while not rospy.is_shutdown():
        try:
            # 处理当前帧
            frame, img = hand_detector.pubHandsPoint(draw=False)
            
            if frame is not None and img is not None:
                # 计算FPS
                cTime = time.time()
                fps = 1 / (cTime - pTime) if (cTime - pTime) > 0 else 0
                pTime = cTime
                
                # 添加FPS文本
                text = "FPS : " + str(int(fps))
                cv.putText(frame, text, (20, 30), cv.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 1)
                
                # 合并并显示图像
                dist = hand_detector.frame_combine(frame, img)
                if dist is not None:
                    cv.imshow('Hand Detection - Combined View', dist)
                
                # 检查退出键
                if cv.waitKey(1) & 0xFF == ord('q'):
                    break
            
            rate.sleep()
            
        except Exception as e:
            rospy.logerr("Error in main loop: {}".format(e))
            continue
    
    cv.destroyAllWindows()
    rospy.loginfo("Hand detector shutdown successfully.")