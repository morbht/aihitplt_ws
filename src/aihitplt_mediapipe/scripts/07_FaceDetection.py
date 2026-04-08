#!/usr/bin/env python3
# encoding: utf-8
import rospy
import mediapipe as mp
import cv2 as cv
import time
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class FaceDetector:
    def __init__(self, minDetectionCon=0.5):
        self.mpFaceDetection = mp.solutions.face_detection
        self.mpDraw = mp.solutions.drawing_utils
        self.facedetection = self.mpFaceDetection.FaceDetection(min_detection_confidence=minDetectionCon)

    def findFaces(self, frame):
        img_RGB = cv.cvtColor(frame, cv.COLOR_BGR2RGB)
        self.results = self.facedetection.process(img_RGB)
        bboxs = []
        if self.results.detections:
            for id, detection in enumerate(self.results.detections):
                bboxC = detection.location_data.relative_bounding_box
                ih, iw, ic = frame.shape
                bbox = int(bboxC.xmin * iw), int(bboxC.ymin * ih), \
                       int(bboxC.width * iw), int(bboxC.height * ih)
                bboxs.append([id, bbox, detection.score])
                frame = self.fancyDraw(frame, bbox)
                cv.putText(frame, f'{int(detection.score[0] * 100)}%',
                           (bbox[0], bbox[1] - 20), cv.FONT_HERSHEY_PLAIN,
                           3, (255, 0, 255), 2)
        return frame, bboxs

    def fancyDraw(self, frame, bbox, l=30, t=10):
        x, y, w, h = bbox
        x1, y1 = x + w, y + h
        cv.rectangle(frame, (x, y),(x + w, y + h), (255, 0, 255), 2)
        # Top left x,y
        cv.line(frame, (x, y), (x + l, y), (255, 0, 255), t)
        cv.line(frame, (x, y), (x, y + l), (255, 0, 255), t)
        # Top right x1,y
        cv.line(frame, (x1, y), (x1 - l, y), (255, 0, 255), t)
        cv.line(frame, (x1, y), (x1, y + l), (255, 0, 255), t)
        # Bottom left x1,y1
        cv.line(frame, (x, y1), (x + l, y1), (255, 0, 255), t)
        cv.line(frame, (x, y1), (x, y1 - l), (255, 0, 255), t)
        # Bottom right x1,y1
        cv.line(frame, (x1, y1), (x1 - l, y1), (255, 0, 255), t)
        cv.line(frame, (x1, y1), (x1, y1 - l), (255, 0, 255), t)
        return frame

class FaceDetectorROS:
    def __init__(self, minDetectionCon=0.5):
        self.bridge = CvBridge()
        self.face_detector = FaceDetector(minDetectionCon)
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.image_callback)
        self.current_frame = None
        self.pTime = time.time()

    def image_callback(self, data):
        try:
            # 将ROS图像消息转换为OpenCV图像
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.current_frame = cv_image
        except CvBridgeError as e:
            rospy.logerr(e)

    def run(self):
        rate = rospy.Rate(30)  # 30Hz
        while not rospy.is_shutdown():
            if self.current_frame is not None:
                frame = self.current_frame.copy()
                
                # 检测人脸
                frame, bboxs = self.face_detector.findFaces(frame)
                
                # 计算并显示FPS
                cTime = time.time()
                fps = 1 / (cTime - self.pTime)
                self.pTime = cTime
                text = "FPS : " + str(int(fps))
                cv.putText(frame, text, (20, 30), cv.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 1)
                
                # 显示图像
                cv.imshow('Face Detection', frame)
                
                # 按q退出
                if cv.waitKey(1) & 0xFF == ord('q'):
                    break
            
            rate.sleep()
        
        cv.destroyAllWindows()

if __name__ == '__main__':
    rospy.init_node('face_detector_node', anonymous=True)
    
    # 从参数服务器获取检测置信度，如果没有则使用默认值0.75
    min_detection_con = rospy.get_param('~min_detection_confidence', 0.75)
    
    rospy.loginfo("Starting face detector with confidence threshold: %.2f", min_detection_con)
    
    face_detector_ros = FaceDetectorROS(min_detection_con)
    
    try:
        face_detector_ros.run()
    except rospy.ROSInterruptException:
        pass