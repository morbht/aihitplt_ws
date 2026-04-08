#!/usr/bin/env python3
# -*-coding: utf-8 -*-
import time
import cv2 as cv
import numpy as np
import rospy
import os
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

# 获取当前文件所在目录
current_dir = os.path.dirname(os.path.abspath(__file__))

######################### Detection ##########################
# 使用绝对路径加载文件
coco_file_path = os.path.join(current_dir, 'object_detection_coco.txt')
model_path = os.path.join(current_dir, 'frozen_inference_graph.pb')
config_path = os.path.join(current_dir, 'ssd_mobilenet_v2_coco.txt')

# load the COCO class names
with open(coco_file_path, 'r') as f: 
    class_names = f.read().split('\n')
# get a different color array for each of the classes
COLORS = np.random.uniform(0, 255, size=(len(class_names), 3))
# load the DNN model
model = cv.dnn.readNet(model=model_path, config=config_path, framework='TensorFlow')

######################### openpose ##########################
BODY_PARTS = {"Nose": 0, "Neck": 1, "RShoulder": 2, "RElbow": 3, "RWrist": 4,
          "LShoulder": 5, "LElbow": 6, "LWrist": 7, "RHip": 8, "RKnee": 9,
          "RAnkle": 10, "LHip": 11, "LKnee": 12, "LAnkle": 13, "REye": 14,
          "LEye": 15, "REar": 16, "LEar": 17, "Background": 18}
POSE_PAIRS = [["Neck", "RShoulder"], ["Neck", "LShoulder"], ["RShoulder", "RElbow"],
          ["RElbow", "RWrist"], ["LShoulder", "LElbow"], ["LElbow", "LWrist"],
          ["Neck", "RHip"], ["RHip", "RKnee"], ["RKnee", "RAnkle"], ["Neck", "LHip"],
          ["LHip", "LKnee"], ["LKnee", "LAnkle"], ["Neck", "Nose"], ["Nose", "REye"],
          ["REye", "REar"], ["Nose", "LEye"], ["LEye", "LEar"]]

# 使用绝对路径加载OpenPose模型
openpose_model_path = os.path.join(current_dir, 'graph_opt.pb')
net = cv.dnn.readNetFromTensorflow(openpose_model_path)

class CombinedDetector:
    def __init__(self):
        # ROS初始化
        rospy.init_node('target_detector', anonymous=True)
        self.bridge = CvBridge()
        
        # 参数设置
        self.input_topic = rospy.get_param('~input_image_topic', '/usb_cam/image_raw')
        self.flip = rospy.get_param('~flip', False)
        self.display = rospy.get_param('~display', True)
        
        # 图像订阅
        self.image_sub = rospy.Subscriber(self.input_topic, Image, self.image_callback)
        
        # 处理后的图像发布
        self.output_pub = rospy.Publisher('/target_detector/output_image', Image, queue_size=1)
        
        # 状态变量
        self.current_frame = None
        self.state = True  # True: Detection, False: Openpose
        self.last_time = time.time()
        
        rospy.loginfo("target Detector initialized")
        rospy.loginfo("Current directory: %s", current_dir)
        rospy.loginfo("Input topic: %s", self.input_topic)
        rospy.loginfo("Press 'f' to toggle between Detection and Openpose")
        
    def image_callback(self, msg):
        try:
            # 转换ROS图像消息到OpenCV格式
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # 图像翻转处理
            if self.flip:
                cv_image = cv.flip(cv_image, 1)
                
            self.current_frame = cv_image
            
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {}".format(e))

    def target_detection(self, image):
        if image is None:
            return image
            
        image_height, image_width, _ = image.shape
        # create blob from image
        blob = cv.dnn.blobFromImage(image=image, size=(300, 300), mean=(104, 117, 123), swapRB=True)
        model.setInput(blob)
        output = model.forward()
        # loop over each of the detections
        for detection in output[0, 0, :, :]:
            # extract the confidence of the detection
            confidence = detection[2]
            # draw bounding boxes only if the detection confidence is above...
            # ... a certain threshold, else skip
            if confidence > .4:
                # get the class id
                class_id = detection[1]
                # map the class id to the class
                class_name = class_names[int(class_id) - 1]
                color = COLORS[int(class_id)]
                # get the bounding box coordinates
                box_x = detection[3] * image_width
                box_y = detection[4] * image_height
                # get the bounding box width and height
                box_width = detection[5] * image_width
                box_height = detection[6] * image_height
                # draw a rectangle around each detected object
                cv.rectangle(image, (int(box_x), int(box_y)), (int(box_width), int(box_height)), color, thickness=2)
                # put the class name text on the detected object
                cv.putText(image, class_name, (int(box_x), int(box_y - 5)), cv.FONT_HERSHEY_SIMPLEX, 1, color, 2)
        return image

    def openpose(self, frame):
        if frame is None:
            return frame
            
        frameHeight, frameWidth = frame.shape[:2]
        net.setInput(cv.dnn.blobFromImage(frame, 1.0, (368, 368), (127.5, 127.5, 127.5), swapRB=True, crop=False))
        out = net.forward()
        out = out[:, :19, :, :]  # MobileNet output [1, 57, -1, -1], we only need the first 19 elements
        assert (len(BODY_PARTS) == out.shape[1])
        points = []
        for i in range(len(BODY_PARTS)):
            # Slice heatmap of corresponging body's part.
            heatMap = out[0, i, :, :]
            # Originally, we try to find all the local maximums. To simplify a sample
            # we just find a global one. However only a single pose at the same time
            # could be detected this way.
            _, conf, _, point = cv.minMaxLoc(heatMap)
            x = (frameWidth * point[0]) / out.shape[3]
            y = (frameHeight * point[1]) / out.shape[2]
            # Add a point if it's confidence is higher than threshold.
            points.append((int(x), int(y)) if conf > 0.2 else None)
        for pair in POSE_PAIRS:
            partFrom = pair[0]
            partTo = pair[1]
            assert (partFrom in BODY_PARTS)
            assert (partTo in BODY_PARTS)
            idFrom = BODY_PARTS[partFrom]
            idTo = BODY_PARTS[partTo]
            if points[idFrom] and points[idTo]:
                cv.line(frame, points[idFrom], points[idTo], (0, 255, 0), 3)
                cv.ellipse(frame, points[idFrom], (3, 3), 0, 0, 360, (0, 0, 255), cv.FILLED)
                cv.ellipse(frame, points[idTo], (3, 3), 0, 0, 360, (0, 0, 255), cv.FILLED)
        return frame

    def run(self):
        rate = rospy.Rate(30)  # 30Hz
        
        while not rospy.is_shutdown():
            if self.current_frame is not None:
                start_time = time.time()
                
                # 处理当前帧
                processed_frame = self.current_frame.copy()
                
                if self.state:
                    processed_frame = self.target_detection(processed_frame)
                    mode_text = "Detection"
                else:
                    processed_frame = self.openpose(processed_frame)
                    mode_text = "Openpose"
                
                # 添加模式文本
                cv.putText(processed_frame, mode_text, (240, 30), cv.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)
                
                # 计算并显示FPS
                end_time = time.time()
                fps = 1 / (end_time - start_time)
                fps_text = "FPS: {:.1f}".format(fps)
                cv.putText(processed_frame, fps_text, (20, 30), cv.FONT_HERSHEY_SIMPLEX, 0.9, (100, 200, 200), 2)
                
                # 发布处理后的图像
                try:
                    output_msg = self.bridge.cv2_to_imgmsg(processed_frame, "bgr8")
                    self.output_pub.publish(output_msg)
                except CvBridgeError as e:
                    rospy.logerr("CvBridge Error: {}".format(e))
                
                # 显示图像（如果启用显示）
                if self.display:
                    cv.imshow('target Detector', processed_frame)
                    key = cv.waitKey(1) & 0xFF
                    
                    # 处理键盘输入
                    if key == ord('q') or key == ord('Q'):
                        break
                    elif key == ord('f') or key == ord('F'):
                        self.state = not self.state
                        rospy.loginfo("Switched to: %s", "Detection" if self.state else "Openpose")
            
            rate.sleep()
        
        if self.display:
            cv.destroyAllWindows()

if __name__ == '__main__':
    try:
        detector = CombinedDetector()
        detector.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr("Error: {}".format(e))