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

class HandDetectorROS:
    def __init__(self, mode=False, maxHands=2, detectorCon=0.5, trackCon=0.5):
        # ROS初始化
        rospy.init_node('hand_detector', anonymous=True)
        self.bridge = CvBridge()
        
        # 图像订阅者
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.image_callback)
        self.current_frame = None
        self.frame_ready = False
        
        # MediaPipe手部检测初始化
        self.tipIds = [4, 8, 12, 16, 20]
        self.mpHand = mp.solutions.hands
        self.mpDraw = mp.solutions.drawing_utils
        self.hands = self.mpHand.Hands(
            static_image_mode=mode,
            max_num_hands=maxHands,
            min_detection_confidence=detectorCon,
            min_tracking_confidence=trackCon
        )
        self.lmDrawSpec = mp.solutions.drawing_utils.DrawingSpec(color=(0, 0, 255), thickness=-1, circle_radius=15)
        self.drawSpec = mp.solutions.drawing_utils.DrawingSpec(color=(0, 255, 0), thickness=10, circle_radius=10)
        
        # 变量初始化
        self.pTime = self.cTime = self.volPer = self.value = self.index = 0
        self.effect = ["color", "thresh", "blur", "hue", "enhance"]
        self.volBar = 400
        self.lmList = []
        self.results = None

    def image_callback(self, msg):
        """图像话题回调函数"""
        try:
            # 将ROS图像消息转换为OpenCV格式
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.current_frame = cv_image
            self.frame_ready = True
        except Exception as e:
            rospy.logerr("图像转换错误: %s", str(e))

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
        if frame is None:
            return np.zeros((480, 640, 3), np.uint8)
            
        img = np.zeros(frame.shape, np.uint8)
        img_RGB = cv.cvtColor(frame, cv.COLOR_BGR2RGB)
        self.results = self.hands.process(img_RGB)
        if self.results.multi_hand_landmarks:
            for handLms in self.results.multi_hand_landmarks:
                if draw: 
                    self.mpDraw.draw_landmarks(img, handLms, self.mpHand.HAND_CONNECTIONS)
        return img

    def findPosition(self, frame, draw=True):
        self.lmList = []
        if self.results and self.results.multi_hand_landmarks:
            for id, lm in enumerate(self.results.multi_hand_landmarks[0].landmark):
                h, w, c = frame.shape
                cx, cy = int(lm.x * w), int(lm.y * h)
                self.lmList.append([id, cx, cy])
                if draw and frame is not None: 
                    cv.circle(frame, (cx, cy), 15, (0, 0, 255), cv.FILLED)
        return self.lmList

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

    def run(self):
        """主循环"""
        rate = rospy.Rate(30)  # 30Hz
        
        while not rospy.is_shutdown():
            if self.frame_ready and self.current_frame is not None:
                frame = self.current_frame.copy()
                action = cv.waitKey(1) & 0xFF
                
                # 手部检测
                img = self.findHands(frame)
                lmList = self.findPosition(frame, draw=False)
                
                if len(lmList) != 0:
                    angle = self.calc_angle(4, 0, 8)
                    x1, y1 = lmList[4][1], lmList[4][2]
                    x2, y2 = lmList[8][1], lmList[8][2]
                    cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
                    cv.circle(img, (x1, y1), 15, (255, 0, 255), cv.FILLED)
                    cv.circle(img, (x2, y2), 15, (255, 0, 255), cv.FILLED)
                    cv.line(img, (x1, y1), (x2, y2), (255, 0, 255), 3)
                    cv.circle(img, (cx, cy), 15, (255, 0, 255), cv.FILLED)
                    if angle <= 10: 
                        cv.circle(img, (cx, cy), 15, (0, 255, 0), cv.FILLED)
                    
                    self.volBar = np.interp(angle, [0, 70], [400, 150])
                    self.volPer = np.interp(angle, [0, 70], [0, 100])
                    self.value = np.interp(angle, [0, 70], [0, 255])

                # 图像效果处理
                if self.effect[self.index] == "thresh":
                    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
                    frame = cv.threshold(gray, self.value, 255, cv.THRESH_BINARY)[1]
                elif self.effect[self.index] == "blur":
                    frame = cv.GaussianBlur(frame, (21, 21), np.interp(self.value, [0, 255], [0, 11]))
                elif self.effect[self.index] == "hue":
                    frame = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
                    frame[:, :, 0] += int(self.value)
                    frame = cv.cvtColor(frame, cv.COLOR_HSV2BGR)
                elif self.effect[self.index] == "enhance":
                    enh_val = self.value / 40
                    clahe = cv.createCLAHE(clipLimit=enh_val, tileGridSize=(8, 8))
                    lab = cv.cvtColor(frame, cv.COLOR_BGR2LAB)
                    lab[:, :, 0] = clahe.apply(lab[:, :, 0])
                    frame = cv.cvtColor(lab, cv.COLOR_LAB2BGR)

                # 切换效果
                if action == ord('f'):
                    self.index += 1
                    if self.index >= len(self.effect): 
                        self.index = 0

                # 计算FPS
                self.cTime = time.time()
                fps = 1 / (self.cTime - self.pTime) if (self.cTime - self.pTime) > 0 else 0
                self.pTime = self.cTime

                # 绘制UI
                text = "FPS : " + str(int(fps))
                cv.rectangle(img, (50, 150), (85, 400), (255, 0, 0), 3)
                cv.rectangle(img, (50, int(self.volBar)), (85, 400), (0, 255, 0), cv.FILLED)
                cv.putText(img, f'{int(self.volPer)}%', (40, 450), cv.FONT_HERSHEY_COMPLEX, 1, (0, 255, 0), 3)
                cv.putText(frame, text, (20, 30), cv.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 1)

                # 合并并显示图像
                dst = self.frame_combine(frame, img)
                cv.imshow('Hand Detector ROS', dst)

                # 退出条件
                if action == ord('q') or cv.getWindowProperty('Hand Detector ROS', cv.WND_PROP_VISIBLE) < 1:
                    break

            rate.sleep()

        cv.destroyAllWindows()

if __name__ == '__main__':
    try:
        detector = HandDetectorROS()
        detector.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr("程序错误: %s", str(e))