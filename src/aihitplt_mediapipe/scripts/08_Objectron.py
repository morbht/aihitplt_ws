#!/usr/bin/env python3
# encoding: utf-8
import rospy
import mediapipe as mp
import cv2 as cv
import time
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String

class Objectron:
    def __init__(self, staticMode=False, maxObjects=5, minDetectionCon=0.5, minTrackingCon=0.99):
        self.staticMode=staticMode
        self.maxObjects=maxObjects
        self.minDetectionCon=minDetectionCon
        self.minTrackingCon=minTrackingCon
        self.index=0
        self.modelNames = ['Shoe', 'Chair', 'Cup', 'Camera']
        self.mpObjectron = mp.solutions.objectron
        self.mpDraw = mp.solutions.drawing_utils
        self.mpobjectron = self.mpObjectron.Objectron(
            self.staticMode, self.maxObjects, self.minDetectionCon, self.minTrackingCon, self.modelNames[self.index])

    def findObjectron(self, frame):
        cv.putText(frame, self.modelNames[self.index], (int(frame.shape[1] / 2) - 30, 30),
                   cv.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 3)
        img_RGB = cv.cvtColor(frame, cv.COLOR_BGR2RGB)
        results = self.mpobjectron.process(img_RGB)
        if results.detected_objects:
            for id, detection in enumerate(results.detected_objects):
                self.mpDraw.draw_landmarks(frame, detection.landmarks_2d, self.mpObjectron.BOX_CONNECTIONS)
                self.mpDraw.draw_axis(frame, detection.rotation, detection.translation)
        return frame

    def configUP(self):
        self.index += 1
        if self.index>=4:self.index=0
        self.mpobjectron = self.mpObjectron.Objectron(
            self.staticMode, self.maxObjects, self.minDetectionCon, self.minTrackingCon, self.modelNames[self.index])
        return self.modelNames[self.index]

class ObjectronROS:
    def __init__(self, staticMode=False, maxObjects=5, minDetectionCon=0.5, minTrackingCon=0.99):
        self.bridge = CvBridge()
        self.objectron = Objectron(staticMode, maxObjects, minDetectionCon, minTrackingCon)
        
        # 订阅图像话题
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.image_callback)
        
        # 发布当前检测模式的话题
        # self.mode_pub = rospy.Publisher("/objectron/current_mode", String, queue_size=10)
        
        # 订阅切换模式的话题
        # self.mode_sub = rospy.Subscriber("/objectron/switch_mode", String, self.mode_callback)
        
        self.current_frame = None
        self.pTime = time.time()

    def image_callback(self, data):
        try:
            # 将ROS图像消息转换为OpenCV图像
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.current_frame = cv_image
        except CvBridgeError as e:
            rospy.logerr(e)

    # def mode_callback(self, msg):
    #     # 收到切换模式的消息
    #     if msg.data == "switch":
    #         new_mode = self.objectron.configUP()
    #         rospy.loginfo("Switched to detection mode: %s", new_mode)
    #         # 发布当前模式
    #         # self.mode_pub.publish(new_mode)

    def run(self):
        rate = rospy.Rate(30)  # 30Hz
        
        rospy.loginfo("Objectron node started. Current mode: %s", self.objectron.modelNames[self.objectron.index])
        rospy.loginfo("Publish to /objectron/switch_mode topic to switch detection mode")
        
        while not rospy.is_shutdown():
            if self.current_frame is not None:
                frame = self.current_frame.copy()
                
                # 进行物体检测
                frame = self.objectron.findObjectron(frame)
                
                # 计算并显示FPS
                cTime = time.time()
                fps = 1 / (cTime - self.pTime)
                self.pTime = cTime
                text = "FPS : " + str(int(fps))
                cv.putText(frame, text, (20, 30), cv.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
                
                # 显示操作提示
                cv.putText(frame, "Switch mode: F/f", 
                          (10, frame.shape[0] - 20), cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                
                # 显示图像
                cv.imshow('Objectron 3D Object Detection', frame)
                
                # 按q退出
                action = cv.waitKey(1) & 0xFF
                if action == ord('q'): break
                if action == ord('f') or action == ord('F') : 
                    new_mode = self.objectron.configUP()
                    rospy.loginfo("Switched to detection mode: %s", new_mode)
            
            rate.sleep()
        
        cv.destroyAllWindows()

if __name__ == '__main__':
    rospy.init_node('objectron_node', anonymous=True)
    
    # 从参数服务器获取配置参数
    static_mode = rospy.get_param('~static_mode', False)
    max_objects = rospy.get_param('~max_objects', 5)
    min_detection_con = rospy.get_param('~min_detection_confidence', 0.5)
    min_tracking_con = rospy.get_param('~min_tracking_confidence', 0.99)
    
    rospy.loginfo("Objectron parameters - static_mode: %s, max_objects: %d, min_detection_confidence: %.2f, min_tracking_confidence: %.2f", 
                  static_mode, max_objects, min_detection_con, min_tracking_con)
    
    objectron_ros = ObjectronROS(static_mode, max_objects, min_detection_con, min_tracking_con)
    
    try:
        objectron_ros.run()
    except rospy.ROSInterruptException:
        pass