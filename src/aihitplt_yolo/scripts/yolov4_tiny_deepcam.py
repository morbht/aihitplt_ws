#!/usr/bin/env python3
# encoding: utf-8
import sys
import time
import rospy
import rospkg
import base64
import cv2 as cv
import numpy as np
import tensorflow as tf
from utils.yolo import YOLO
from aihitplt_msgs.msg import *
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

# 设置TensorFlow全局图
global_graph = tf.Graph()
global_session = tf.compat.v1.Session(graph=global_graph)

gpus = tf.config.experimental.list_physical_devices(device_type='GPU')
for gpu in gpus:
    tf.config.experimental.set_memory_growth(gpu, True)

class YoloDetect:
    def __init__(self):
        rospy.on_shutdown(self.cancel)
        rospy.init_node("YoloDetect", anonymous=False)
        self.bridge = CvBridge()
        self.pTime = self.cTime = 0
        
        rospkg_path = rospkg.RosPack().get_path("aihitplt_yolo") + ''
        model_path = rospkg_path + '/param/yolov4_tiny_weights_coco.h5'
        classes_path = rospkg_path + '/param/coco.txt'
        
        # 在全局图中初始化YOLO
        with global_graph.as_default():
            with global_session.as_default():
                self.yolov4_tiny = YOLO(rospkg_path, model_path, classes_path)
        
        # 发布器
        self.pub_image = rospy.Publisher('Detect/image_msg', Image_Msg, queue_size=10)
        self.pub_msg = rospy.Publisher('DetectMsg', TargetArray, queue_size=10)
        
        # 订阅Astra相机的图像话题
        self.image_sub = rospy.Subscriber("/pan_tilt_camera/image", Image, self.image_callback)
        
        rospy.loginfo("YOLO检测节点已启动，订阅/pan_tilt_camera/image")

    def image_callback(self, msg):
        """接收Astra相机的图像并进行检测"""
        try:
            # 将ROS图像消息转换为OpenCV格式
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # 在全局会话中进行检测
            with global_graph.as_default():
                with global_session.as_default():
                    self.detect(frame)
            
        except CvBridgeError as e:
            rospy.logerr("CV Bridge error: %s" % str(e))
        except Exception as e:
            rospy.logerr("Detection error: %s" % str(e))

    def cancel(self):
        self.pub_image.unregister()
        self.pub_msg.unregister()
        global_session.close()
        cv.destroyAllWindows()

    def pub_imgMsg(self, frame):
        try:
            pic_base64 = base64.b64encode(frame)
            image = Image_Msg()
            size = frame.shape
            image.height = size[0]
            image.width = size[1]
            image.channels = size[2]
            image.data = pic_base64
            self.pub_image.publish(image)
        except Exception as e:
            rospy.logerr("Error publishing image: %s" % str(e))

    def detect(self, frame):
        try:
            target_array = TargetArray()
            # 格式转变，BGRtoRGB
            frame_rgb = cv.cvtColor(frame, cv.COLOR_BGR2RGB)
            frame_processed, out_boxes, out_scores, out_classes = self.yolov4_tiny.detect_image(frame_rgb)
            
            for i, c in list(enumerate(out_classes)):
                predicted_class = self.yolov4_tiny.class_names[c]
                box = out_boxes[i]
                score = out_scores[i]
                self.yolov4_tiny.draw_img(frame_processed, c, box, score, predicted_class)
                target = Target()
                target.frame_id = predicted_class
                target.stamp = rospy.Time.now()
                target.scores = score
                target.ptx = box[0]
                target.pty = box[1]
                target.distw = box[2] - box[0]
                target.disth = box[3] - box[1]
                target.centerx = (box[2] - box[0]) / 2
                target.centery = (box[3] - box[1]) / 2
                target_array.data.append(target)
                
            self.cTime = time.time()
            fps = 1 / (self.cTime - self.pTime) if self.pTime > 0 else 0
            self.pTime = self.cTime
            text = "FPS : " + str(int(fps))
            frame_processed = np.array(frame_processed)
            # RGBtoBGR满足opencv显示格式
            frame_processed = cv.cvtColor(frame_processed, cv.COLOR_RGB2BGR)
            cv.putText(frame_processed, text, (20, 30), cv.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 1)
            self.pub_msg.publish(target_array)
            self.pub_imgMsg(frame_processed)
            
            # 显示图像
            cv.imshow('YOLO Detection', frame_processed)
            cv.waitKey(1)
            
        except Exception as e:
            rospy.logerr("Error in detection: %s" % str(e))

if __name__ == "__main__":
    print("Python version: ", sys.version)
    
    # 直接启动ROS节点，不需要摄像头访问
    detect = YoloDetect()
    rospy.loginfo("YOLO检测节点运行中...")
    rospy.spin()
