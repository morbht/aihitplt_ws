#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import cv2
import numpy as np
import sys
import threading
import os
import time
import torch
import signal
from collections import deque
from datetime import datetime

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
from std_msgs.msg import String
from aihitplt_yolo.msg import DetectResult

from PyQt5.QtWidgets import QApplication, QMainWindow, QLabel, QVBoxLayout, QWidget, QTextEdit
from PyQt5.QtCore import QTimer, Qt, pyqtSignal, QObject
from PyQt5.QtGui import QImage, QPixmap


class Communicate(QObject):
    update_frame = pyqtSignal(np.ndarray, str)


class YoloInferenceThread(threading.Thread):
    def __init__(self, model_path, frame_queue, result_queue, comm):
        super().__init__(daemon=True)
        
        self.device = 'cuda:0' if torch.cuda.is_available() else 'cpu'
        rospy.loginfo(f"使用设备: {self.device}")
        
        self.model = YOLO(model_path)
        self.model.to(self.device)
        
        self.bridge = CvBridge()
        self.frame_queue = frame_queue
        self.result_queue = result_queue
        self.comm = comm
        self.running = True
        self.inference_count = 0
        self.last_fps_time = time.time()
        self.current_fps = 0.0
        
        self.inference_size = 320 if self.device == 'cpu' else 640
        self.conf_threshold = 0.5
        self.half = False
        
        # 添加检测结果发布器
        self.result_pub = rospy.Publisher('/pan_tilt_camera/DetectMsg', DetectResult, queue_size=10)
        
        # 添加ROS订阅标志
        self.subscriber_initialized = False
        
    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            if len(self.frame_queue) < 2:
                self.frame_queue.append(cv_image)
            else:
                self.frame_queue[-1] = cv_image
        except Exception as e:
            rospy.logerr(f"图像接收错误: {e}")
    
    def publish_detection_result(self, result, header):
        # 创建检测结果消息
        detect_msg = DetectResult()
        detect_msg.header = header
        
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
            # 获取第一个检测框
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
            detect_msg.box_count = len(result.boxes)
            detect_msg.x_min = int(x1)
            detect_msg.y_min = int(y1)
            detect_msg.x_max = int(x2)
            detect_msg.y_max = int(y2)
            detect_msg.confidence = conf
            detect_msg.class_name = cls_name
            
            rospy.loginfo(f"检测到目标: {cls_name}, 置信度: {conf:.2f}")
        
        # 发布消息
        self.result_pub.publish(detect_msg)
    
    def run(self):
        # 创建订阅者
        self.image_sub = rospy.Subscriber(
            '/pan_tilt_camera/image', 
            Image, 
            self.image_callback,
            queue_size=1,
            buff_size=2**24
        )
        self.subscriber_initialized = True
        
        # 设置ROS回调处理速率
        rate = rospy.Rate(100)  # 100Hz
        
        while self.running and not rospy.is_shutdown():
            try:
                # 处理ROS回调（rospy.spinOnce()的替代方案）
                # 在ROS1中，回调是在单独的线程中处理的，我们只需要确保有足够的时间处理
                
                if not self.frame_queue:
                    rate.sleep()
                    continue
                
                frame = self.frame_queue.pop()
                self.frame_queue.clear()
                
                t0 = time.time()
                try:
                    results = self.model.predict(
                        frame,
                        imgsz=self.inference_size,
                        conf=self.conf_threshold,
                        half=self.half,
                        verbose=False,
                        device=self.device
                    )
                    inference_time = (time.time() - t0) * 1000
                    
                    annotated_frame = results[0].plot()
                    
                    # 发布检测结果
                    if hasattr(self, 'result_pub'):
                        from std_msgs.msg import Header
                        header = Header()
                        header.stamp = rospy.Time.now()
                        header.frame_id = "camera_frame"
                        self.publish_detection_result(results[0], header)
                    
                except Exception as e:
                    rospy.logerr(f"推理错误: {e}")
                    continue
                
                self.inference_count += 1
                current_time = time.time()
                if current_time - self.last_fps_time >= 1.0:
                    self.current_fps = self.inference_count
                    self.inference_count = 0
                    self.last_fps_time = current_time
                
                status = f"FPS:{self.current_fps} | 推理:{inference_time:.1f}ms "
                self.comm.update_frame.emit(annotated_frame, status)
                
            except Exception as e:
                rospy.logerr(f"YOLO线程错误: {e}")
                time.sleep(0.01)
    
    def stop(self):
        """安全停止线程"""
        self.running = False
        # 等待线程结束
        self.join(timeout=2.0)


class FlameDetectorGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        
        # 设置信号处理
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)
        
        # 初始化ROS节点（使用全局变量确保只初始化一次）
        if not rospy.core.is_initialized():
            # disable_signals=True 防止ROS接管信号处理
            rospy.init_node('flame_detector_gui', anonymous=True, disable_signals=True)
        
        self.comm = Communicate()
        self.comm.update_frame.connect(self.on_new_frame)
        
        self.frame_queue = deque(maxlen=2)
        self.result_queue = deque(maxlen=1)
        
        self._help_text = None
        
        model_path = '/home/aihit/aihitplt_ws/src/aihitplt_yolo/param/fire_detect.pt'
        self.yolo_thread = YoloInferenceThread(
            model_path, 
            self.frame_queue, 
            self.result_queue, 
            self.comm
        )
        self.yolo_thread.start()
        
        self.ptz_speed = 3
        self.moving_direction = None
        self.function_states = {'wiper': False}
        self.last_key_time = 0
        
        self.display_buffer = None
        self.latest_frame = None
        self.latest_status = "初始化中..."
        
        self.init_ui()
        
        self.display_timer = QTimer()
        self.display_timer.timeout.connect(self.update_display)
        self.display_timer.start(33)  # ~30fps
        
        # 添加退出标志
        self.is_closing = False
        
    def signal_handler(self, signum, frame):
        """处理系统信号"""
        rospy.loginfo(f"接收到信号 {signum}，正在关闭...")
        # 在主线程中关闭GUI
        QTimer.singleShot(0, self.close)
        
    def init_ui(self):
        self.setWindowTitle("火焰检测系统")
        self.setGeometry(100, 100, 800, 600)
        
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        layout = QVBoxLayout(central_widget)
        
        self.video_label = QLabel()
        self.video_label.setAlignment(Qt.AlignCenter)
        self.video_label.setMinimumSize(600, 400)
        self.video_label.setStyleSheet("border: 2px solid gray; background-color: black;")
        layout.addWidget(self.video_label)
        
        self.status_label = QLabel("初始化中...")
        self.status_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.status_label)
        
        self.speed_label = QLabel(f"云台速度: {self.ptz_speed}")
        self.speed_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.speed_label)
        
        self.help_text = QTextEdit()
        self.help_text.setReadOnly(True)
        self.help_text.setMaximumHeight(140)
        self.help_text.setText(self.get_help_content())
        layout.addWidget(self.help_text)
        
        self.setFocusPolicy(Qt.StrongFocus)
        self.setFocus()
        
        self.control_pub = rospy.Publisher('/pan_tilt_camera_control', String, queue_size=10)
        self.speed_pub = rospy.Publisher('/pan_tilt_camera_speed', String, queue_size=10)
        
        self.show()
    
    def get_help_content(self):
        if self._help_text is None:
            self._help_text = """控制说明:
1:焦距变大  2:焦距变小  3:灯光开启  4:灯光关闭  5:焦点前调
6:焦点后调  7:光圈扩大  8:光圈缩小  9:打开雨刷  0:关闭雨刷
w:云台上仰  s:云台下俯  a:云台左转  d:云台右转
+:增加速度  -:降低速度  f:截图      c:停止移动  q:退出程序
速度范围: 1-7"""
        return self._help_text
    
    def on_new_frame(self, frame, status_text):
        if not self.is_closing:
            self.latest_frame = frame
            self.latest_status = status_text
    
    def update_display(self):
        if self.is_closing or self.latest_frame is None:
            return
        
        frame = self.latest_frame
        
        if len(frame.shape) == 3:
            rgb_image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        else:
            rgb_image = frame
        
        h, w, ch = rgb_image.shape
        
        if (self.display_buffer is None or 
            self.display_buffer.height() != h or 
            self.display_buffer.width() != w):
            self.display_buffer = QImage(w, h, QImage.Format_RGB888)
        
        qt_image = QImage(
            rgb_image.data, 
            w, h, 
            ch * w,
            QImage.Format_RGB888
        )
        
        pixmap = QPixmap.fromImage(qt_image)
        
        label_size = self.video_label.size()
        if w > label_size.width() or h > label_size.height():
            scaled_pixmap = pixmap.scaled(
                label_size, 
                Qt.KeepAspectRatio, 
                Qt.SmoothTransformation
            )
        else:
            scaled_pixmap = pixmap
        
        self.video_label.setPixmap(scaled_pixmap)
        
        status = self.latest_status
        if self.moving_direction:
            direction_names = {'w': "上仰", 's': "下俯", 'a': "左转", 'd': "右转"}
            status += f" | 移动:{direction_names.get(self.moving_direction, '')}"
        if self.function_states['wiper']:
            status += " | 雨刷开"
        
        self.status_label.setText(status)
        self.speed_label.setText(f"云台速度: {self.ptz_speed}")
    
    def keyPressEvent(self, event):
        if self.is_closing:
            return
            
        current_time = time.time()
        if current_time - self.last_key_time < 0.15:
            return
        self.last_key_time = current_time
        
        key = event.text().lower()
        
        if key == 'q':
            self.close()
        
        elif key == 'f':
            self.capture_image()
        
        elif key == 'c':
            self.send_control('c')
            self.moving_direction = None
        
        elif key == '+':
            self.adjust_speed(1)
        elif key == '-':
            self.adjust_speed(-1)
        
        elif key in ['w', 's', 'a', 'd']:
            self.send_control(key)
            self.moving_direction = key
        
        elif key in map(str, range(10)):
            self.send_control(key)
            if key == '9':
                self.function_states['wiper'] = True
            elif key == '0':
                self.function_states['wiper'] = False
    
    def send_control(self, cmd):
        try:
            msg = String()
            msg.data = cmd
            self.control_pub.publish(msg)
        except Exception as e:
            rospy.logerr(f"发送命令失败: {e}")
    
    def adjust_speed(self, delta):
        new_speed = self.ptz_speed + delta
        if 1 <= new_speed <= 7:
            self.ptz_speed = new_speed
            try:
                msg = String()
                msg.data = str(self.ptz_speed)
                self.speed_pub.publish(msg)
            except Exception as e:
                rospy.logerr(f"发送速度失败: {e}")
    
    def capture_image(self):
        if self.latest_frame is not None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            save_dir = "/home/aihit/aihitplt_ws/src/aihitplt_pan_tilt_camera/img"
            os.makedirs(save_dir, exist_ok=True)
            filename = f"{save_dir}/image_{timestamp}.jpg"
            cv2.imwrite(filename, self.latest_frame)
            rospy.loginfo(f"截图已保存: {filename}")
    
    def closeEvent(self, event):
        """重写关闭事件，确保正确清理资源"""
        if self.is_closing:
            if event:
                event.accept()
            return
            
        self.is_closing = True
        rospy.loginfo("正在关闭程序...")
        
        # 停止显示定时器
        self.display_timer.stop()
        
        # 停止YOLO线程
        if hasattr(self, 'yolo_thread'):
            self.yolo_thread.stop()
        
        # 清理ROS资源
        try:
            # 取消所有订阅
            if hasattr(self, 'control_pub'):
                self.control_pub.unregister()
            if hasattr(self, 'speed_pub'):
                self.speed_pub.unregister()
            
            # 关闭ROS节点
            if rospy.core.is_initialized() and not rospy.is_shutdown():
                rospy.signal_shutdown("GUI关闭")
        except Exception as e:
            rospy.logerr(f"清理ROS资源时出错: {e}")
        
        # 清理OpenCV窗口
        cv2.destroyAllWindows()
        
        # 给线程一点时间退出
        time.sleep(0.5)
        
        # 接受关闭事件
        if event:
            event.accept()
        
        rospy.loginfo("程序已完全退出")


def main():
    app = QApplication(sys.argv)
    
    # 设置应用程序退出处理
    app.aboutToQuit.connect(lambda: rospy.loginfo("Qt应用即将退出"))
    
    gui = FlameDetectorGUI()
    
    # 在ROS1中，回调是在单独的线程中处理的，不需要手动spin
    # 我们已经在YOLO线程中处理了ROS回调
    
    exit_code = app.exec_()
    
    rospy.loginfo("应用退出，退出码: {}".format(exit_code))
    sys.exit(exit_code)


if __name__ == "__main__":
    main()