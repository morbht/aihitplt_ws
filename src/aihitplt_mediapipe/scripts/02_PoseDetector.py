#!/usr/bin/env python3
# encoding: utf-8
import time
import rospy
import cv2 as cv
import numpy as np
import mediapipe as mp
from geometry_msgs.msg import Point
from aihitplt_msgs.msg import PointArray

class PoseDetector:
    def __init__(self, mode=False, smooth=True, detectionCon=0.5, trackCon=0.5):
        self.mpPose = mp.solutions.pose
        self.mpDraw = mp.solutions.drawing_utils
        self.pose = self.mpPose.Pose(
            static_image_mode=mode,
            smooth_landmarks=smooth,
            min_detection_confidence=detectionCon,
            min_tracking_confidence=trackCon )
        self.pub_point = rospy.Publisher('/mediapipe/points', PointArray, queue_size=1000)
        self.lmDrawSpec = mp.solutions.drawing_utils.DrawingSpec(color=(0, 0, 255), thickness=-1, circle_radius=6)
        self.drawSpec = mp.solutions.drawing_utils.DrawingSpec(color=(0, 255, 0), thickness=2, circle_radius=2)

    def pubPosePoint(self, frame, draw=True):
        pointArray = PointArray()
        img = np.zeros(frame.shape, np.uint8)
        img_RGB = cv.cvtColor(frame, cv.COLOR_BGR2RGB)
        self.results = self.pose.process(img_RGB)
        if self.results.pose_landmarks:
            if draw: self.mpDraw.draw_landmarks(frame, self.results.pose_landmarks, self.mpPose.POSE_CONNECTIONS, self.lmDrawSpec, self.drawSpec)
            self.mpDraw.draw_landmarks(img, self.results.pose_landmarks, self.mpPose.POSE_CONNECTIONS, self.lmDrawSpec, self.drawSpec)
            for id, lm in enumerate(self.results.pose_landmarks.landmark):
                point = Point()
                point.x, point.y, point.z = lm.x, lm.y, lm.z
                pointArray.points.append(point)
        self.pub_point.publish(pointArray)
        return frame, img

    def frame_combine(slef,frame, src):
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
    rospy.init_node('PoseDetector', anonymous=True)
    capture = cv.VideoCapture(2)
    capture.set(6, cv.VideoWriter.fourcc('M', 'J', 'P', 'G'))
    capture.set(cv.CAP_PROP_FRAME_WIDTH, 640)
    capture.set(cv.CAP_PROP_FRAME_HEIGHT, 480)
    print("capture get FPS : ", capture.get(cv.CAP_PROP_FPS))
    pTime = cTime = 0
    pose_detector = PoseDetector()
    index = 3
    while capture.isOpened():
        ret, frame = capture.read()
        # frame = cv.flip(frame, 1)
        frame, img = pose_detector.pubPosePoint(frame,draw=False)
        if cv.waitKey(1) & 0xFF == ord('q'): break
        cTime = time.time()
        fps = 1 / (cTime - pTime)
        pTime = cTime
        text = "FPS : " + str(int(fps))
        cv.putText(frame, text, (20, 30), cv.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 1)
        dist = pose_detector.frame_combine(frame, img)
        cv.imshow('dist', dist)
        # cv.imshow('frame', frame)
        # cv.imshow('img', img)
    capture.release()
    cv.destroyAllWindows()
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

class PoseDetector:
    def __init__(self, mode=False, smooth=True, detectionCon=0.5, trackCon=0.5):
        self.mpPose = mp.solutions.pose
        self.mpDraw = mp.solutions.drawing_utils
        self.pose = self.mpPose.Pose(
            static_image_mode=mode,
            smooth_landmarks=smooth,
            min_detection_confidence=detectionCon,
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
        
        rospy.loginfo("Pose detector initialized, waiting for camera images...")

    def image_callback(self, data):
        """相机图像回调函数"""
        try:
            # 将ROS图像消息转换为OpenCV格式
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.current_frame = cv_image
            self.frame_received = True
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {}".format(e))

    def pubPosePoint(self, draw=True):
        """处理图像并发布姿态关键点"""
        pointArray = PointArray()
        
        if self.current_frame is None:
            return None, None
            
        img = np.zeros(self.current_frame.shape, np.uint8)
        img_RGB = cv.cvtColor(self.current_frame, cv.COLOR_BGR2RGB)
        self.results = self.pose.process(img_RGB)
        
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
    rospy.init_node('PoseDetector', anonymous=True)
    
    # 创建姿态检测器
    pose_detector = PoseDetector()
    
    # 等待第一帧图像
    rospy.loginfo("Waiting for first camera image...")
    while not pose_detector.frame_received and not rospy.is_shutdown():
        rospy.sleep(0.1)
    
    if rospy.is_shutdown():
        exit(0)
        
    rospy.loginfo("Camera image received, starting pose detection...")
    
    pTime = cTime = 0
    rate = rospy.Rate(30)  # 30Hz
    index = 3
    
    while not rospy.is_shutdown():
        try:
            # 处理当前帧
            frame, img = pose_detector.pubPosePoint(draw=False)
            
            if frame is not None and img is not None:
                # 计算FPS
                cTime = time.time()
                fps = 1 / (cTime - pTime) if (cTime - pTime) > 0 else 0
                pTime = cTime
                
                # 添加FPS文本
                text = "FPS : " + str(int(fps))
                cv.putText(frame, text, (20, 30), cv.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 1)
                
                # 合并并显示图像
                dist = pose_detector.frame_combine(frame, img)
                if dist is not None:
                    cv.imshow('Pose Detection - Combined View', dist)
                
                # 检查退出键
                if cv.waitKey(1) & 0xFF == ord('q'):
                    break
            
            rate.sleep()
            
        except Exception as e:
            rospy.logerr("Error in main loop: {}".format(e))
            continue
    
    cv.destroyAllWindows()
    rospy.loginfo("Pose detector shutdown successfully.")