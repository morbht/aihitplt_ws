#!/usr/bin/env python3
# encoding: utf-8
import time
import rospy
import rospkg
import cv2 as cv
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage, Image


class FaceEyeDetection:
    def __init__(self):
        self.bridge = CvBridge()
        rospy.on_shutdown(self.cancel)
        rospy.init_node("FaceEyeDetection", anonymous=False)
        
        # 加载分类器文件
        self.eyeDetect = cv.CascadeClassifier(
            rospkg.RosPack().get_path("aihitplt_mediapipe") + "/scripts/file/haarcascade_eye.xml")
        self.faceDetect = cv.CascadeClassifier(
            rospkg.RosPack().get_path("aihitplt_mediapipe") + "/scripts/file/haarcascade_frontalface_default.xml")
        
        # 发布处理后的图像
        self.pub_rgb = rospy.Publisher("/FaceEyeDetection/image", Image, queue_size=1)
        
        # 订阅相机话题
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.image_callback)
        
        # 当前帧和状态变量
        self.current_frame = None
        self.frame_received = False
        self.content_index = 0
        self.content = ["face", "eye", "face_eye"]
        
        rospy.loginfo("FaceEyeDetection initialized, waiting for camera images...")

    def image_callback(self, data):
        """相机图像回调函数"""
        try:
            # 将ROS图像消息转换为OpenCV格式
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.current_frame = cv_image
            self.frame_received = True
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {}".format(e))

    def cancel(self):
        """关闭回调"""
        self.pub_rgb.unregister()

    def face(self, frame):
        """人脸检测"""
        if frame is None:
            return None
            
        gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        faces = self.faceDetect.detectMultiScale(gray, 1.3)
        for face in faces: 
            frame = self.faceDraw(frame, face)
        return frame

    def eye(self, frame):
        """眼睛检测"""
        if frame is None:
            return None
            
        gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        eyes = self.eyeDetect.detectMultiScale(gray, 1.3)
        for eye in eyes:
            cv.circle(frame, (int(eye[0] + eye[2] / 2), int(eye[1] + eye[3] / 2)), (int(eye[3] / 2)), (0, 0, 255), 2)
        return frame

    def faceDraw(self, frame, bbox, l=30, t=10):
        """绘制人脸框"""
        if frame is None:
            return None
            
        x, y, w, h = bbox
        x1, y1 = x + w, y + h
        cv.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 255), 2)
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

    def pub_img(self, frame):
        """发布处理后的图像"""
        if frame is not None:
            try:
                self.pub_rgb.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))
            except CvBridgeError as e:
                rospy.logerr("CvBridge Error in pub_img: {}".format(e))

    def process_frame(self):
        """处理当前帧"""
        if self.current_frame is None:
            return None
            
        frame = self.current_frame.copy()
        
        # 根据当前模式进行处理
        if self.content[self.content_index] == "face": 
            frame = self.face(frame)
        elif self.content[self.content_index] == "eye": 
            frame = self.eye(frame)
        else: 
            frame_temp = self.face(frame)
            if frame_temp is not None:
                frame = self.eye(frame_temp)
        
        return frame


if __name__ == '__main__':
    face_eye_detection = FaceEyeDetection()
    
    # 等待第一帧图像
    rospy.loginfo("Waiting for first camera image...")
    while not face_eye_detection.frame_received and not rospy.is_shutdown():
        rospy.sleep(0.1)
    
    if rospy.is_shutdown():
        exit(0)
        
    rospy.loginfo("Camera image received, starting face and eye detection...")
    
    pTime, cTime = 0, 0
    rate = rospy.Rate(30)  # 30Hz
    
    while not rospy.is_shutdown():
        try:
            # 处理当前帧
            processed_frame = face_eye_detection.process_frame()
            
            if processed_frame is not None:
                # 检查键盘输入
                key = cv.waitKey(1) & 0xFF
                if key == ord("f") or key == ord("F"):
                    face_eye_detection.content_index += 1
                    if face_eye_detection.content_index >= len(face_eye_detection.content): 
                        face_eye_detection.content_index = 0
                    rospy.loginfo("Switched to mode: {}".format(face_eye_detection.content[face_eye_detection.content_index]))
                
                if key == ord('q') or key == ord("Q"): 
                    break
                
                # 计算FPS
                cTime = time.time()
                fps = 1 / (cTime - pTime) if (cTime - pTime) > 0 else 0
                pTime = cTime
                
                # 添加FPS文本和模式提示
                text = "FPS : " + str(int(fps))
                mode_text = "Mode: " + face_eye_detection.content[face_eye_detection.content_index] + " (Press F to switch)"
                cv.putText(processed_frame, text, (20, 30), cv.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 1)
                cv.putText(processed_frame, mode_text, (20, 60), cv.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 1)
                
                # 显示图像
                cv.imshow('Face and Eye Detection', processed_frame)
                
                # 发布处理后的图像（取消注释以启用发布）
                # face_eye_detection.pub_img(processed_frame)
            
            rate.sleep()
            
        except Exception as e:
            rospy.logerr("Error in main loop: {}".format(e))
            continue
    
    cv.destroyAllWindows()
    rospy.loginfo("FaceEyeDetection shutdown successfully.")