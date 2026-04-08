#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
# 添加自定义消息类型
from aihitplt_yolo.msg import DetectResult

class FlameDetector:
    def __init__(self):
        # 初始化节点
        rospy.init_node('flame_detector', anonymous=True)
        
        # 加载模型
        self.model = YOLO('/home/aihit/aihitplt_ws/src/aihitplt_yolo/param/fire_detect.pt')
        
        # 创建CvBridge
        self.bridge = CvBridge()
        
        # 设置显示窗口大小
        self.display_scale = 0.4  # 缩小为原来的一半
        
        # 订阅图像话题
        self.image_sub = rospy.Subscriber('/pan_tilt_camera/image', Image, self.image_callback)
        
        # 添加发布器，发布检测结果
        self.result_pub = rospy.Publisher('/pan_tilt_camera/DetectMsg', DetectResult, queue_size=10)
        
    
    def image_callback(self, msg):
        try:
            # 将ROS图像消息转换为OpenCV图像
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # 使用YOLO进行预测
            results = self.model.predict(cv_image, conf=0.5)
            
            # 在图像上绘制检测结果
            annotated_frame = results[0].plot()
            
            # 缩小图像用于显示
            height, width = annotated_frame.shape[:2]
            new_width = int(width * self.display_scale)
            new_height = int(height * self.display_scale)
            display_frame = cv2.resize(annotated_frame, (new_width, new_height))
            
            # 显示结果
            cv2.imshow("Flame Detection", display_frame)
            cv2.waitKey(1)
            
            # 发布检测结果（只发布第一个检测到的目标）
            self.publish_detection_result(results[0], msg.header)
            
        except Exception as e:
            rospy.logerr(f"处理图像时出错: {e}")
    
    def publish_detection_result(self, result, header):
        """
        发布YOLO检测结果到话题（只发布第一个检测到的目标）
        :param result: YOLO检测结果对象
        :param header: 原始图像的消息头
        """
        # 创建检测结果消息
        detect_msg = DetectResult()
        detect_msg.header = header  # 使用原始图像的时间戳和frame_id
        
        # 初始化默认值
        detect_msg.detected = False
        detect_msg.box_count = 0
        detect_msg.x_min = 0
        detect_msg.y_min = 0
        detect_msg.x_max = 0
        detect_msg.y_max = 0
        detect_msg.confidence = 0.0
        detect_msg.class_name = ""
        
        # 获取检测到的目标信息
        if result.boxes is not None and len(result.boxes) > 0:
            # 获取第一个检测框（根据消息定义，只发布第一个）
            box = result.boxes[0]
            
            # 获取边界框坐标
            x1, y1, x2, y2 = box.xyxy[0].tolist()
            
            # 获取置信度
            conf = float(box.conf[0])
            
            # 获取类别ID
            cls_id = int(box.cls[0])
            
            # 获取类别名称
            cls_name = result.names[cls_id] if result.names else str(cls_id)
            
            # 设置检测结果
            detect_msg.detected = True
            detect_msg.box_count = len(result.boxes)  # 记录总检测框数量
            detect_msg.x_min = int(x1)
            detect_msg.y_min = int(y1)
            detect_msg.x_max = int(x2)
            detect_msg.y_max = int(y2)
            detect_msg.confidence = conf
            detect_msg.class_name = cls_name
            
            rospy.loginfo(f"检测到目标: {cls_name}, 置信度: {conf:.2f}")
        else:
            rospy.loginfo("未检测到目标")
        
        # 发布消息
        self.result_pub.publish(detect_msg)
    
    def run(self):
        rospy.spin()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        detector = FlameDetector()
        detector.run()
    except rospy.ROSInterruptException:
        pass