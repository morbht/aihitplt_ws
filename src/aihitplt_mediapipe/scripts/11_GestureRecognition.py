#!/usr/bin/env python3
# encoding: utf-8
import math
import time
import cv2 as cv
import numpy as np
import mediapipe as mp
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class HandDetector:
    def __init__(self, mode=False, maxHands=2, detectorCon=0.5, trackCon=0.5):
        self.tipIds = [4, 8, 12, 16, 20]
        self.mpHand = mp.solutions.hands
        self.mpDraw = mp.solutions.drawing_utils
        self.hands = self.mpHand.Hands(
            static_image_mode=mode,
            max_num_hands=maxHands,
            min_detection_confidence=detectorCon,
            min_tracking_confidence=trackCon
        )
        self.lmList = []
        self.lmDrawSpec = mp.solutions.drawing_utils.DrawingSpec(color=(0, 0, 255), thickness=-1, circle_radius=6)
        self.drawSpec = mp.solutions.drawing_utils.DrawingSpec(color=(0, 255, 0), thickness=2, circle_radius=2)

    def get_dist(self, point1, point2):
        x1, y1 = point1
        x2, y2 = point2
        return abs(math.sqrt(math.pow(abs(y1 - y2), 2) + math.pow(abs(x1 - x2), 2)))

    def calc_angle(self, pt1, pt2, pt3):
        if len(self.lmList) <= max(pt1, pt2, pt3):
            return 0
        point1 = self.lmList[pt1][1], self.lmList[pt1][2]
        point2 = self.lmList[pt2][1], self.lmList[pt2][2]
        point3 = self.lmList[pt3][1], self.lmList[pt3][2]
        a = self.get_dist(point1, point2)
        b = self.get_dist(point2, point3)
        c = self.get_dist(point1, point3)
        try:
            radian = math.acos((math.pow(a, 2) + math.pow(b, 2) - math.pow(c, 2)) / (2 * a * b))
            angle = radian / math.pi * 180
        except:
            angle = 0
        return abs(angle)

    def findHands(self, frame, draw=True):
        self.lmList = []
        if frame is None:
            return None, np.zeros((480, 640, 3), np.uint8)
            
        img = np.zeros(frame.shape, np.uint8)
        img_RGB = cv.cvtColor(frame, cv.COLOR_BGR2RGB)
        self.results = self.hands.process(img_RGB)
        if self.results.multi_hand_landmarks:
            for i in range(len(self.results.multi_hand_landmarks)):
                if draw: 
                    self.mpDraw.draw_landmarks(frame, self.results.multi_hand_landmarks[i], 
                                             self.mpHand.HAND_CONNECTIONS, self.lmDrawSpec, self.drawSpec)
                self.mpDraw.draw_landmarks(img, self.results.multi_hand_landmarks[i], 
                                         self.mpHand.HAND_CONNECTIONS, self.lmDrawSpec, self.drawSpec)
                for id, lm in enumerate(self.results.multi_hand_landmarks[i].landmark):
                    h, w, c = frame.shape
                    cx, cy = int(lm.x * w), int(lm.y * h)
                    self.lmList.append([id, cx, cy])
        return frame, img

    def frame_combine(self, frame, src):
        if frame is None or src is None:
            return np.zeros((480, 640, 3), np.uint8)
            
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

    def fingersUp(self):
        fingers = []
        if len(self.lmList) == 0:
            return fingers
            
        # Thumb
        if (self.calc_angle(self.tipIds[0],
                            self.tipIds[0] - 1,
                            self.tipIds[0] - 2) > 150.0) and (
                self.calc_angle(
                    self.tipIds[0] - 1,
                    self.tipIds[0] - 2,
                    self.tipIds[0] - 3) > 150.0): 
            fingers.append(1)
        else:
            fingers.append(0)
        # 4 finger
        for id in range(1, 5):
            if len(self.lmList) > self.tipIds[id] and len(self.lmList) > self.tipIds[id] - 2:
                if self.lmList[self.tipIds[id]][2] < self.lmList[self.tipIds[id] - 2][2]:
                    fingers.append(1)
                else:
                    fingers.append(0)
            else:
                fingers.append(0)
        return fingers

    def get_gesture(self):
        if len(self.lmList) == 0:
            return ""
            
        gesture = ""
        fingers = self.fingersUp()
        
        if len(fingers) < 5:
            return ""
            
        if self.lmList[self.tipIds[0]][2] > self.lmList[self.tipIds[1]][2] and \
                self.lmList[self.tipIds[0]][2] > self.lmList[self.tipIds[2]][2] and \
                self.lmList[self.tipIds[0]][2] > self.lmList[self.tipIds[3]][2] and \
                self.lmList[self.tipIds[0]][2] > self.lmList[self.tipIds[4]][2]: 
            gesture = "Thumb_down"

        elif self.lmList[self.tipIds[0]][2] < self.lmList[self.tipIds[1]][2] and \
                self.lmList[self.tipIds[0]][2] < self.lmList[self.tipIds[2]][2] and \
                self.lmList[self.tipIds[0]][2] < self.lmList[self.tipIds[3]][2] and \
                self.lmList[self.tipIds[0]][2] < self.lmList[self.tipIds[4]][2] and \
                self.calc_angle(self.tipIds[1] - 1, self.tipIds[1] - 2, self.tipIds[1] - 3) < 150.0: 
            gesture = "Thumb_up"
            
        if fingers.count(1) == 3 or fingers.count(1) == 4:
            if fingers[0] == 1 and (
                    self.get_dist(self.lmList[4][1:], self.lmList[8][1:]) < self.get_dist(self.lmList[4][1:], self.lmList[5][1:])
            ): 
                gesture = "OK"
            elif fingers[2] == fingers[3] == 0: 
                gesture = "Rock"
            elif fingers.count(1) == 3: 
                gesture = "Three"
            else: 
                gesture = "Four"
        elif fingers.count(1) == 0: 
            gesture = "Zero"
        elif fingers.count(1) == 1: 
            gesture = "One"
        elif fingers.count(1) == 2:
            if fingers[0] == 1 and fingers[4] == 1: 
                gesture = "Six"
            elif fingers[0] == 1 and self.calc_angle(4, 5, 8) > 90: 
                gesture = "Eight"
            elif fingers[0] == fingers[1] == 1 and self.get_dist(self.lmList[4][1:], self.lmList[8][1:]) < 50: 
                gesture = "Heart_single"
            else: 
                gesture = "Two"
        elif fingers.count(1) == 5: 
            gesture = "Five"
            
        if len(self.lmList) > 20:
            if self.get_dist(self.lmList[4][1:], self.lmList[8][1:]) < 60 and \
                    self.get_dist(self.lmList[4][1:], self.lmList[12][1:]) < 60 and \
                    self.get_dist(self.lmList[4][1:], self.lmList[16][1:]) < 60 and \
                    self.get_dist(self.lmList[4][1:], self.lmList[20][1:]) < 60: 
                gesture = "Seven"
                
        if self.lmList[self.tipIds[0]][2] < self.lmList[self.tipIds[1]][2] and \
                self.lmList[self.tipIds[0]][2] < self.lmList[self.tipIds[2]][2] and \
                self.lmList[self.tipIds[0]][2] < self.lmList[self.tipIds[3]][2] and \
                self.lmList[self.tipIds[0]][2] < self.lmList[self.tipIds[4]][2] and \
                self.calc_angle(self.tipIds[1] - 1, self.tipIds[1] - 2, self.tipIds[1] - 3) > 150.0: 
            gesture = "Eight"
            
        return gesture

class HandDetectorROS:
    def __init__(self):
        # ROS初始化
        rospy.init_node('hand_gesture_detector', anonymous=True)
        self.bridge = CvBridge()
        
        # 图像订阅者
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.image_callback)
        self.current_frame = None
        self.frame_ready = False
        
        # 手部检测器
        self.hand_detector = HandDetector(detectorCon=0.75)
        
        # 时间变量
        self.pTime = self.cTime = 0

    def image_callback(self, msg):
        """图像话题回调函数"""
        try:
            # 将ROS图像消息转换为OpenCV格式
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.current_frame = cv_image
            self.frame_ready = True
        except Exception as e:
            rospy.logerr("图像转换错误: %s", str(e))

    def run(self):
        """主循环"""
        rate = rospy.Rate(30)  # 30Hz
        
        while not rospy.is_shutdown():
            if self.frame_ready and self.current_frame is not None:
                frame = self.current_frame.copy()
                
                # 手部检测
                processed_frame, hand_img = self.hand_detector.findHands(frame, draw=False)
                
                if len(self.hand_detector.lmList) != 0:
                    totalFingers = self.hand_detector.get_gesture()
                    cv.rectangle(processed_frame, (0, 430), (230, 480), (0, 255, 0), cv.FILLED)
                    cv.putText(processed_frame, str(totalFingers), (10, 470), cv.FONT_HERSHEY_PLAIN, 2, (255, 0, 0), 2)
                
                # 计算FPS
                self.cTime = time.time()
                fps = 1 / (self.cTime - self.pTime) if (self.cTime - self.pTime) > 0 else 0
                self.pTime = self.cTime
                
                # 显示FPS
                text = "FPS : " + str(int(fps))
                cv.putText(processed_frame, text, (10, 30), cv.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 1)
                
                # 合并并显示图像
                dist = self.hand_detector.frame_combine(processed_frame, hand_img)
                cv.imshow('Hand Gesture Detection', dist)
                
                # 退出条件
                if cv.waitKey(1) & 0xFF == ord('q') or cv.getWindowProperty('Hand Gesture Detection', cv.WND_PROP_VISIBLE) < 1:
                    break

            rate.sleep()

        cv.destroyAllWindows()

if __name__ == '__main__':
    try:
        detector_ros = HandDetectorROS()
        detector_ros.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr("程序错误: %s", str(e))