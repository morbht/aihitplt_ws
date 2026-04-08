#!/usr/bin/env python3
import os
import sys
import threading
import time
import ctypes
import signal
from datetime import datetime

# 在导入任何其他库之前彻底修复环境
os.environ['QT_QPA_PLATFORM'] = 'xcb'
os.environ['QT_LOGGING_RULES'] = 'qt.qpa.*=false'

import cv2
import rospy
import numpy as np
from std_msgs.msg import String, Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from PyQt5.QtWidgets import QApplication, QMainWindow, QLabel, QVBoxLayout, QWidget, QTextEdit
from PyQt5.QtCore import QTimer, Qt, pyqtSignal, QObject
from PyQt5.QtGui import QImage, QPixmap, QFont, QKeyEvent

class CameraSignals(QObject):
    """信号类，用于线程间通信"""
    update_frame = pyqtSignal(np.ndarray)
    update_status = pyqtSignal(str)
    key_pressed = pyqtSignal(str)

class HikvisionCameraNode:
    def __init__(self):
        print("启动云台相机ROS节点...")
        
        # 初始化ROS节点
        rospy.init_node('aihitplt_pan_camera', anonymous=True)
        
        # 初始化CV Bridge
        self.bridge = CvBridge()
        
        # 加载海康威视SDK
        self.load_sdk()
        
        # 摄像头参数
        self.camera_ip = "192.168.2.64"
        self.username = "admin"
        self.password = "abcd1234"
        self.port = 8000
        self.channel = 1
        
        # 控制参数
        self.ptz_speed = 3
        self.capture_count = 0
        self.capture_path = "/home/aihit/aihitplt_ws/src/aihitplt_pan_tilt_camera/img/"
        
        # 云台控制命令常量
        self.PTZ_COMMANDS = {
            '1': 11, '2': 12, '3': 2, '4': 13, '5': 14,
            '6': 15, '7': 16, '8': 3, '9': 3,
            'w': 21, 's': 22, 'a': 23, 'd': 24,
        }
        
        # 当前状态
        self.active_control = None
        self.frame_count = 0
        self.start_time = time.time()
        self.running = True  # 控制程序运行状态
        
        # 功能键状态跟踪
        self.function_states = {
            'light': False,    # 灯光状态
            'wiper': False,    # 雨刷状态
        }
        
        # 移动控制状态
        self.moving_direction = None  # 当前移动方向
        
        # 初始化ROS发布器和订阅器
        self.setup_ros_publishers()
        self.setup_ros_subscribers()
        
        # 登录摄像头
        self.user_id = self.login_camera()
        if self.user_id < 0:
            rospy.logerr("摄像头登录失败")
            return
            
        # 启动视频流 - 使用主码流
        if not self.start_video_stream():
            rospy.logerr("视频流启动失败")
            return
            
        rospy.loginfo("pan_camera node init completed")
        
        # 启动视频读取线程
        self.video_thread = threading.Thread(target=self.video_loop)
        self.video_thread.daemon = True
        self.video_thread.start()
    
    def setup_ros_publishers(self):
        """初始化ROS发布器"""
        # 图像话题
        self.image_pub = rospy.Publisher('/pan_tilt_camera/image', Image, queue_size=10)
        
        # 云台控制话题
        self.control_pub = rospy.Publisher('/pan_tilt_camera_control', String, queue_size=10)
        
        # 雨刷控制话题
        self.wiper_pub = rospy.Publisher('/pan_tilt_camera_wiper', Bool, queue_size=10)
        
        # 灯光控制话题
        self.light_pub = rospy.Publisher('/pan_tilt_camera_light', Bool, queue_size=10)
        
        print("ROS发布器初始化完成")
    
    def setup_ros_subscribers(self):
        """初始化ROS订阅器"""
        # 订阅云台控制命令
        rospy.Subscriber('/pan_tilt_camera_control', String, self.control_callback)
        
        # 订阅雨刷控制命令
        rospy.Subscriber('/pan_tilt_camera_wiper', Bool, self.wiper_callback)
        
        # 订阅灯光控制命令
        rospy.Subscriber('/pan_tilt_camera_light', Bool, self.light_callback)
        
        print("ROS订阅器初始化完成")
    
    def control_callback(self, msg):
        """云台控制命令回调"""
        command_char = msg.data.strip().lower()
        print(f"收到云台控制命令: {command_char}")
        
        if command_char == 'c':  # 停止命令
            self.stop_all_movement()
        elif command_char in ['w', 's', 'a', 'd']:  # 移动命令
            self.start_continuous_movement(command_char)
        elif command_char in self.PTZ_COMMANDS:  # 其他功能命令
            self.execute_single_command(command_char)
    
    def start_continuous_movement(self, direction):
        """开始持续移动"""
        # 如果已经在移动，先停止
        if self.moving_direction:
            self.stop_movement(self.moving_direction)
        
        # 开始新的移动
        if direction in self.PTZ_COMMANDS:
            command = self.PTZ_COMMANDS[direction]
            result = self.ptz_control(command, 0)  # 开始移动
            if result:
                self.moving_direction = direction
                self.active_control = command
    
    def stop_movement(self, direction):
        """停止指定方向的移动"""
        if direction in self.PTZ_COMMANDS:
            command = self.PTZ_COMMANDS[direction]
            self.ptz_control(command, 1)  # 停止移动
            print(f"停止{direction}方向移动")
    
    def stop_all_movement(self):
        """停止所有移动"""
        if self.moving_direction:
            self.stop_movement(self.moving_direction)
            self.moving_direction = None
            self.active_control = None
            print("停止所有移动")
    
    def execute_single_command(self, command_char):
        """执行单次命令（焦距、焦点、光圈等）"""
        if command_char in self.PTZ_COMMANDS:
            command = self.PTZ_COMMANDS[command_char]
            
            # 对于功能开关命令
            if command_char == '3':  # 灯光控制
                self.publish_light_command()
            elif command_char == '8':  # 打开雨刷
                self.control_wiper(True)
            elif command_char == '9':  # 关闭雨刷  
                self.control_wiper(False)
            else:  # 焦距、焦点、光圈控制
                self.ptz_control(command, 0)  # 开始控制
                # 0.5秒后自动停止
                threading.Timer(0.5, lambda: self.ptz_control(command, 1)).start()
    
    def wiper_callback(self, msg):
        """雨刷控制命令回调"""
        if msg.data:
            print("收到雨刷开启命令")
            self.control_wiper(True)
        else:
            print("收到雨刷关闭命令")
            self.control_wiper(False)
    
    def light_callback(self, msg):
        """灯光控制命令回调 - 使用pan_tilt_camera.py的正确逻辑"""
        if msg.data:  # 如果收到true消息
            print("收到灯光控制命令")
            self.control_light()
    
    def publish_image(self, frame):
        """发布图像话题"""
        try:
            # 转换OpenCV图像为ROS图像消息
            ros_image = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            ros_image.header.stamp = rospy.Time.now()
            ros_image.header.frame_id = "pan_tilt_camera"
            
            self.image_pub.publish(ros_image)
        except Exception as e:
            rospy.logerr(f"发布图像话题失败: {e}")
    
    def publish_control_command(self, command_char):
        """发布云台控制话题"""
        try:
            control_msg = String()
            control_msg.data = command_char
            self.control_pub.publish(control_msg)
        except Exception as e:
            rospy.logerr(f"发布云台控制话题失败: {e}")
    
    def publish_wiper_status(self, status):
        """发布雨刷状态话题"""
        try:
            wiper_msg = Bool()
            wiper_msg.data = status
            self.wiper_pub.publish(wiper_msg)
        except Exception as e:
            rospy.logerr(f"发布雨刷话题失败: {e}")
    
    def publish_light_command(self):
        """发布灯光控制命令 - 使用pan_tilt_camera.py的正确逻辑"""
        try:
            light_msg = Bool()
            light_msg.data = True  # 总是发布true
            self.light_pub.publish(light_msg)
            print("发布灯光控制命令")
        except Exception as e:
            rospy.logerr(f"发布灯光话题失败: {e}")
    
    def load_sdk(self):
        """加载SDK"""
        try:
            sdk_path = "/home/aihit/aihitplt_ws/src/aihitplt_pan_tilt_camera/lib/libhcnetsdk.so"
            self.sdk = ctypes.cdll.LoadLibrary(sdk_path)
        except Exception as e:
            print(f"SDK加载失败: {e}")
            sys.exit(1)
    
    def login_camera(self):
        """登录摄像头"""
        class NET_DVR_DEVICEINFO_V30(ctypes.Structure):
            _fields_ = [
                ("sSerialNumber", ctypes.c_byte * 48),
                ("byAlarmInPortNum", ctypes.c_byte), ("byAlarmOutPortNum", ctypes.c_byte),
                ("byDiskNum", ctypes.c_byte), ("byDVRType", ctypes.c_byte),
                ("byChanNum", ctypes.c_byte), ("byStartChan", ctypes.c_byte),
                ("byAudioChanNum", ctypes.c_byte), ("byIPChanNum", ctypes.c_byte),
                ("byZeroChanNum", ctypes.c_byte), ("byMainProto", ctypes.c_byte),
                ("bySubProto", ctypes.c_byte), ("bySupport", ctypes.c_byte),
                ("bySupport1", ctypes.c_byte), ("bySupport2", ctypes.c_byte),
                ("wDevType", ctypes.c_uint16), ("bySupport3", ctypes.c_byte),
                ("byMultiStreamProto", ctypes.c_byte), ("byStartDChan", ctypes.c_byte),
                ("byStartDTalkChan", ctypes.c_byte), ("byHighDChanNum", ctypes.c_byte),
                ("bySupport4", ctypes.c_byte), ("byLanguageType", ctypes.c_byte),
                ("byVoiceInChanNum", ctypes.c_byte), ("byStartVoiceInChanNo", ctypes.c_byte),
                ("byRes3", ctypes.c_byte * 2), ("byMirrorChanNum", ctypes.c_byte),
                ("wStartMirrorChanNo", ctypes.c_uint16), ("byRes2", ctypes.c_byte * 2)
            ]
        
        if not self.sdk.NET_DVR_Init():
            error = self.sdk.NET_DVR_GetLastError()
            print(f"SDK初始化失败，错误码: {error}")
            return -1
             
        self.sdk.NET_DVR_SetConnectTime(5000, 1)
        self.sdk.NET_DVR_SetReconnect(10000, 1)
        
        device_info = NET_DVR_DEVICEINFO_V30()
        user_id = self.sdk.NET_DVR_Login_V30(
            self.camera_ip.encode(), self.port, 
            self.username.encode(), self.password.encode(), 
            ctypes.byref(device_info)
        )
        
        if user_id < 0:
            error = self.sdk.NET_DVR_GetLastError()
            print(f"摄像头登录失败，错误码: {error}")
            return -1
            
        print(f"登录成功，UserID: {user_id}")
        return user_id
    
    def start_video_stream(self):
        """启动视频流 - 使用主码流（H.264）"""
        # 主码流RTSP地址
        main_stream_url = f"rtsp://{self.username}:{self.password}@{self.camera_ip}:554/h264/ch1/main/av_stream"
        
        print(f"连接主码流: {main_stream_url}")
        self.cap = cv2.VideoCapture(main_stream_url)
        
        # 设置缓冲区大小
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        
        # 测试连接
        if self.cap.isOpened():
            # 尝试读取一帧确认连接正常
            ret, frame = self.cap.read()
            if ret and frame is not None:
                print(f"主码流连接成功")
                print(f"视频分辨率: {frame.shape[1]}x{frame.shape[0]}")
                return True
            else:
                print(f"连接成功但无法读取帧")
                self.cap.release()
        else:
            print(f"连接失败")
        
        return False
    
    def ptz_control(self, command, stop=0):
        """云台控制"""
        result = self.sdk.NET_DVR_PTZControl_Other(
            self.user_id, self.channel, command, stop
        )
        
        if not result:
            error = self.sdk.NET_DVR_GetLastError()
            print(f"云台控制失败，错误码: {error}")
            return False
            
        return True
    
    def control_light(self):
        """控制灯光 - 使用pan_tilt_camera.py的正确逻辑"""
        result = self.ptz_control(2, 1)  # 执行灯光动作
        print("灯光控制执行")
        return result
    
    def wiper_worker(self):
        """雨刷工作线程"""
        while self.function_states['wiper'] and self.running:
            # 执行一次雨刷动作
            self.ptz_control(3, 0)  # 开始雨刷
            time.sleep(1)  # 雨刷动作持续时间
            self.ptz_control(3, 1)  # 停止雨刷
            time.sleep(2)  # 等待2秒后再次执行
    
    def control_wiper(self, turn_on=True):
        """控制雨刷"""
        if turn_on and not self.function_states['wiper']:
            self.function_states['wiper'] = True
            # 发布雨刷状态
            self.publish_wiper_status(True)
            # 启动雨刷工作线程
            wiper_thread = threading.Thread(target=self.wiper_worker)
            wiper_thread.daemon = True
            wiper_thread.start()
            return True
            
        elif not turn_on and self.function_states['wiper']:
            self.function_states['wiper'] = False
            # 发布雨刷状态
            self.publish_wiper_status(False)
            # 发送停止命令确保雨刷停止
            self.ptz_control(3, 1)
            return True
        
        return False
    
    def capture_image(self):
        """截图功能"""
        if not hasattr(self, 'current_frame') or self.current_frame is None:
 
            return False
            
        self.capture_count += 1
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"{self.capture_path}image_{timestamp}_{self.capture_count:04d}.jpg"
        
        cv2.imwrite(filename, self.current_frame)
        print(f"截图保存: {filename}")
        return True

    def video_loop(self):
        """视频读取循环"""
        while self.running and not rospy.is_shutdown():
            ret, frame = self.cap.read()
            if ret:
                self.frame_count += 1
                self.current_frame = frame.copy()
                
                # 发布图像话题
                self.publish_image(frame)
                
                # 简化帧率显示，每100帧显示一次
                if self.frame_count % 100 == 0:
                    fps = 100 / (time.time() - self.start_time)
                    print(f"实时帧率: {fps:.1f} FPS")
                    self.start_time = time.time()
                    self.frame_count = 0
                
            else:
                if self.running:  # 只在运行状态下打印错误
                    print("⚠️ 读取帧失败")
                time.sleep(0.1)
    
    def handle_key_press(self, key_char):
        """处理按键按下"""
        if key_char == 'q':
            print("\nkeyboard termination")
            self.shutdown()
            return
            
        elif key_char == 'f':
            self.capture_image()
            return
            
        elif key_char == 'c':  # 停止命令

            self.publish_control_command('c')  # 发布停止命令
            return
            
        elif key_char in ['w', 's', 'a', 'd']:  # 移动键

            self.publish_control_command(key_char)  # 发布移动命令
            
        elif key_char in ['1', '2', '3', '4', '5', '6', '7', '8', '9']:  # 功能键

            self.publish_control_command(key_char)  # 发布功能命令
    
    def shutdown(self):
        """关闭程序"""
        if not self.running:
            return
            
        self.running = False
        self.cleanup()
        os._exit(0)
    
    def cleanup(self):
        """清理资源"""
        print("cleanning resource...")
        # 停止所有移动
        self.stop_all_movement()
        
        # 关闭雨刷
        if self.function_states['wiper']:
            self.control_wiper(False)
            
        if hasattr(self, 'cap'):
            self.cap.release()
            
        if hasattr(self, 'user_id') and self.user_id >= 0:
            self.sdk.NET_DVR_Logout(self.user_id)
            
        self.sdk.NET_DVR_Cleanup()
        print("finish cleanning")
    
    def run(self):
        """ROS主循环"""
        try:
            rospy.spin()
        except KeyboardInterrupt:
            self.shutdown()

class CameraWindow(QMainWindow):
    def __init__(self, camera_node, app):
        super().__init__()
        self.camera_node = camera_node
        self.app = app
        self.setup_ui()
        self.setup_timer()
        
    def setup_ui(self):
        """设置用户界面"""
        self.setWindowTitle("pan_camera")
        self.setGeometry(100, 100, 1000, 800)
        
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        layout = QVBoxLayout(central_widget)
        
        # 视频显示标签
        self.video_label = QLabel()
        self.video_label.setAlignment(Qt.AlignCenter)
        self.video_label.setMinimumSize(800, 600)
        self.video_label.setStyleSheet("border: 2px solid gray; background-color: black;")
        layout.addWidget(self.video_label)
        
        # 状态显示
        self.status_label = QLabel("等待视频流...")
        self.status_label.setAlignment(Qt.AlignCenter)
        self.status_label.setFont(QFont("Arial", 12))
        layout.addWidget(self.status_label)
        
        # 控制说明
        self.help_text = QTextEdit()
        self.help_text.setReadOnly(True)
        self.help_text.setMaximumHeight(120)
        help_content = """
        控制说明:
        1:焦距变大  2:焦距变小  3:灯光开关  4:焦点前调  5:焦点后调
        6:光圈扩大  7:光圈缩小  8:打开雨刷  9:关闭雨刷
        w:云台上仰  s:云台下俯  a:云台左转  d:云台右转
        f:截图      c:停止移动  q:退出程序
        """
        self.help_text.setText(help_content)
        layout.addWidget(self.help_text)        
        
    def setup_timer(self):
        """设置定时器更新显示"""
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_display)
        self.timer.start(33)  # 30fps
        
    def update_display(self):
        """更新显示"""
        if (hasattr(self.camera_node, 'current_frame') and 
            self.camera_node.current_frame is not None and
            self.camera_node.running):
            
            frame = self.camera_node.current_frame
            
            # 直接显示原始画面，不添加任何控制信息
            display_frame = frame.copy()
            
            # 转换OpenCV图像为Qt图像
            rgb_image = cv2.cvtColor(display_frame, cv2.COLOR_BGR2RGB)
            h, w, ch = rgb_image.shape
            bytes_per_line = ch * w
            
            qt_image = QImage(rgb_image.data, w, h, bytes_per_line, QImage.Format_RGB888)
            pixmap = QPixmap.fromImage(qt_image)
            
            # 缩放并显示
            scaled_pixmap = pixmap.scaled(self.video_label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
            self.video_label.setPixmap(scaled_pixmap)
            
            # 更新状态栏文本
            status = "就绪"
            if self.camera_node.moving_direction:
                direction_names = {'w': "上仰", 's': "下俯", 'a': "左转", 'd': "右转"}
                status = f"移动中: {direction_names.get(self.camera_node.moving_direction, '未知')}"
            
            # 添加功能状态显示
            if self.camera_node.function_states['wiper']:
                status += " | 雨刷开"
                
            self.status_label.setText(status)
        elif not self.camera_node.running:
            # 程序正在退出，清空显示
            self.video_label.clear()
            self.status_label.setText("程序退出中...")
    
    def keyPressEvent(self, event):
        """键盘按下事件"""
        key_char = event.text().lower()
        
        if key_char in ['q', 'f', 'c', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'w', 's', 'a', 'd']:
            self.camera_node.handle_key_press(key_char)
        else:
            super().keyPressEvent(event)
    
    def closeEvent(self, event):
        """窗口关闭事件"""
        self.camera_node.shutdown()
        event.accept()

def signal_handler(sig, frame):
    """处理Ctrl+C信号"""
    # 强制退出
    os._exit(0)

def main():
    # 设置Ctrl+C信号处理
    signal.signal(signal.SIGINT, signal_handler)
    
    # 创建Qt应用
    app = QApplication(sys.argv)
    app.setStyle('Fusion')
    
    # 创建摄像头节点
    camera_node = HikvisionCameraNode()
    if camera_node.user_id < 0:
        sys.exit(1)
    
    # 创建显示窗口
    window = CameraWindow(camera_node, app)
    window.show()
    
    print("海康威视摄像头节点启动完成")
    print("使用键盘控制摄像头，按q退出")
    
    # 运行ROS和Qt
    try:
        # 在单独的线程中运行ROS
        ros_thread = threading.Thread(target=camera_node.run)
        ros_thread.daemon = True
        ros_thread.start()
        
        # 运行Qt主循环
        app_exec = app.exec_()
        
        sys.exit(app_exec)
        
    except Exception as e:
        print(f"\n程序异常: {e}")
        camera_node.shutdown()
        sys.exit(1)

if __name__ == "__main__":
    main()