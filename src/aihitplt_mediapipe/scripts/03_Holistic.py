#!/usr/bin/env python3
# encoding: utf-8
import time
import rospy
import cv2 as cv
import numpy as np
import mediapipe as mp
from geometry_msgs.msg import Point
from aihitplt_msgs.msg import PointArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class Holistic:
    def __init__(self, staticMode=False, landmarks=True, detectionCon=0.5, trackingCon=0.5):
        self.mpHolistic = mp.solutions.holistic
        self.mpFaceMesh = mp.solutions.face_mesh
        self.mpHands = mp.solutions.hands
        self.mpPose = mp.solutions.pose
        self.mpDraw = mp.solutions.drawing_utils
        self.mpholistic = self.mpHolistic.Holistic(
            static_image_mode=staticMode,
            smooth_landmarks=landmarks,
            min_detection_confidence=detectionCon,
            min_tracking_confidence=trackingCon)
        
        # ROS相关初始化
        self.bridge = CvBridge()
        self.current_frame = None
        self.frame_received = False
        
        # 发布者
        self.pub_point = rospy.Publisher('/mediapipe/points', PointArray, queue_size=1000)
        
        # 订阅者 - 订阅相机话题
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.image_callback)
        
        self.lmDrawSpec = mp.solutions.drawing_utils.DrawingSpec(color=(0, 0, 255), thickness=-1, circle_radius=3)
        self.drawSpec = mp.solutions.drawing_utils.DrawingSpec(color=(0, 255, 0), thickness=2, circle_radius=2)
        
        rospy.loginfo("Holistic detector initialized, waiting for camera images...")

    def image_callback(self, data):
        """相机图像回调函数"""
        try:
            # 将ROS图像消息转换为OpenCV格式
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.current_frame = cv_image
            self.frame_received = True
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {}".format(e))

    def findHolistic(self, draw=True):
        """处理图像并检测全身关键点"""
        pointArray = PointArray()
        
        if self.current_frame is None:
            return None, None
            
        img = np.zeros(self.current_frame.shape, np.uint8)
        img_RGB = cv.cvtColor(self.current_frame, cv.COLOR_BGR2RGB)
        self.results = self.mpholistic.process(img_RGB)
        
        # 检测面部关键点
        if self.results.face_landmarks:
            if draw: 
                self.mpDraw.draw_landmarks(self.current_frame, 
                                         self.results.face_landmarks, 
                                         self.mpFaceMesh.FACEMESH_CONTOURS, 
                                         self.lmDrawSpec, 
                                         self.drawSpec)
            self.mpDraw.draw_landmarks(img, 
                                     self.results.face_landmarks, 
                                     self.mpFaceMesh.FACEMESH_CONTOURS, 
                                     self.lmDrawSpec, 
                                     self.drawSpec)
            for id, lm in enumerate(self.results.face_landmarks.landmark):
                point = Point()
                point.x, point.y, point.z = lm.x, lm.y, lm.z
                pointArray.points.append(point)
        
        # 检测姿态关键点
        if self.results.pose_landmarks:
            if draw: 
                self.mpDraw.draw_landmarks(self.current_frame, 
                                         self.results.pose_landmarks, 
                                         self.mpPose.POSE_CONNECTIONS, 
                                         self.lmDrawSpec, 
                                         self.drawSpec)
            self.mpDraw.draw_landmarks(img, 
                                     self.results.pose_landmarks, 
                                     self.mpPose.POSE_CONNECTIONS, 
                                     self.lmDrawSpec, 
                                     self.drawSpec)
            for id, lm in enumerate(self.results.pose_landmarks.landmark):
                point = Point()
                point.x, point.y, point.z = lm.x, lm.y, lm.z
                pointArray.points.append(point)
        
        # 检测左手关键点
        if self.results.left_hand_landmarks:
            if draw: 
                self.mpDraw.draw_landmarks(self.current_frame, 
                                         self.results.left_hand_landmarks, 
                                         self.mpHands.HAND_CONNECTIONS, 
                                         self.lmDrawSpec, 
                                         self.drawSpec)
            self.mpDraw.draw_landmarks(img, 
                                     self.results.left_hand_landmarks, 
                                     self.mpHands.HAND_CONNECTIONS, 
                                     self.lmDrawSpec, 
                                     self.drawSpec)
            for id, lm in enumerate(self.results.left_hand_landmarks.landmark):
                point = Point()
                point.x, point.y, point.z = lm.x, lm.y, lm.z
                pointArray.points.append(point)
        
        # 检测右手关键点
        if self.results.right_hand_landmarks:
            if draw: 
                self.mpDraw.draw_landmarks(self.current_frame, 
                                         self.results.right_hand_landmarks, 
                                         self.mpHands.HAND_CONNECTIONS, 
                                         self.lmDrawSpec, 
                                         self.drawSpec)
            self.mpDraw.draw_landmarks(img, 
                                     self.results.right_hand_landmarks, 
                                     self.mpHands.HAND_CONNECTIONS, 
                                     self.lmDrawSpec, 
                                     self.drawSpec)
            for id, lm in enumerate(self.results.right_hand_landmarks.landmark):
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
    rospy.init_node('Holistic', anonymous=True)
    
    # 创建全身检测器
    holistic = Holistic()
    
    # 等待第一帧图像
    rospy.loginfo("Waiting for first camera image...")
    while not holistic.frame_received and not rospy.is_shutdown():
        rospy.sleep(0.1)
    
    if rospy.is_shutdown():
        exit(0)
        
    rospy.loginfo("Camera image received, starting holistic detection...")
    
    pTime = cTime = 0
    rate = rospy.Rate(30)  # 30Hz
    
    while not rospy.is_shutdown():
        try:
            # 处理当前帧
            frame, img = holistic.findHolistic(draw=False)
            
            if frame is not None and img is not None:
                # 计算FPS
                cTime = time.time()
                fps = 1 / (cTime - pTime) if (cTime - pTime) > 0 else 0
                pTime = cTime
                
                # 添加FPS文本
                text = "FPS : " + str(int(fps))
                cv.putText(frame, text, (20, 30), cv.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
                
                # 合并并显示图像
                dist = holistic.frame_combine(frame, img)
                if dist is not None:
                    cv.imshow('Holistic Detection - Combined View', dist)
                
                # 检查退出键
                if cv.waitKey(1) & 0xFF == ord('q'):
                    break
            
            rate.sleep()
            
        except Exception as e:
            rospy.logerr("Error in main loop: {}".format(e))
            continue
    
    cv.destroyAllWindows()
    rospy.loginfo("Holistic detector shutdown successfully.")