#!/usr/bin/env python3
import os
import sys
import threading
import time
import ctypes
import signal
from datetime import datetime

# 先初始化ROS节点
import rospy
rospy.init_node('aihitplt_pan_camera', anonymous=True)

# 获取参数后再设置环境变量
gui_enabled = rospy.get_param('~gui', True)
if gui_enabled:
    os.environ['QT_QPA_PLATFORM'] = 'xcb'
    os.environ['QT_LOGGING_RULES'] = 'qt.qpa.*=false'

import cv2
import numpy as np
from std_msgs.msg import String, Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

if gui_enabled:
    from PyQt5.QtWidgets import QApplication, QMainWindow, QLabel, QVBoxLayout, QWidget, QTextEdit
    from PyQt5.QtCore import QTimer, Qt, QObject
    from PyQt5.QtGui import QImage, QPixmap, QFont

class HikvisionCameraNode:
    def __init__(self):
        self.gui_enabled = gui_enabled
        
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
        
        # 云台控制命令常量
        self.PTZ_COMMANDS = {
            '1': 11, '2': 12, '3': 2, '4': 2, '5': 13,
            '6': 14, '7': 15, '8': 16, '9': 3, '0': 3,
            'w': 21, 's': 22, 'a': 23, 'd': 24,
        }
        
        # 当前状态
        self.current_frame = None
        self.frame_count = 0
        self.start_time = time.time()
        self.running = True
        self.function_states = {'wiper': False}
        self.moving_direction = None
        self.camera_ready = False
        self.last_state_pub_time = time.time()
        self.state_pub_interval = 0.5
        
        # 初始化ROS发布器和订阅器
        self.setup_ros_publishers()
        self.setup_ros_subscribers()
        
        # 登录摄像头
        self.user_id = self.login_camera()
        if self.user_id < 0:
            rospy.logerr("摄像头登录失败")
            return
            
        # 启动视频流
        if not self.start_video_stream():
            rospy.logerr("视频流启动失败")
            return
            
        # 启动视频读取线程
        self.video_thread = threading.Thread(target=self.video_loop)
        self.video_thread.daemon = True
        self.video_thread.start()
    
    # ==================== ROS通信设置 ====================
    def setup_ros_publishers(self):
        self.image_pub = rospy.Publisher('/pan_tilt_camera/image', Image, queue_size=10)
        self.control_pub = rospy.Publisher('/pan_tilt_camera_control', String, queue_size=10)
        self.wiper_pub = rospy.Publisher('/pan_tilt_camera_wiper', Bool, queue_size=10)
        self.light_pub = rospy.Publisher('/pan_tilt_camera_light', Bool, queue_size=10)
        self.state_pub = rospy.Publisher('/pan_tilt_camera/state', Bool, queue_size=10)
    
    def setup_ros_subscribers(self):
        rospy.Subscriber('/pan_tilt_camera_control', String, self.control_callback)
        rospy.Subscriber('/pan_tilt_camera_wiper', Bool, self.wiper_callback)
        rospy.Subscriber('/pan_tilt_camera_light', Bool, self.light_callback)
        rospy.Subscriber('/pan_tilt_camera_speed', String, self.speed_callback)
        rospy.Subscriber('/pan_tilt_camera/preset_control', String, self.preset_control_callback)
    
    # ==================== SDK相关函数 ====================
    def load_sdk(self):
        try:
            sdk_path = "/home/aihit/aihitplt_ws/src/aihitplt_security/lib/libhcnetsdk.so"
            self.sdk = ctypes.cdll.LoadLibrary(sdk_path)
        except Exception as e:
            print(f"SDK加载失败: {e}")
            sys.exit(1)
    
    def login_camera(self):
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
            print(f"SDK初始化失败: {error}")
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
            print(f"摄像头登录失败: {error}")
            return -1
            
        return user_id
    
    # ==================== 视频流相关 ====================
    def start_video_stream(self):
        main_stream_url = f"rtsp://{self.username}:{self.password}@{self.camera_ip}:554/h264/ch1/main/av_stream"
        
        self.cap = cv2.VideoCapture(main_stream_url)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        
        if self.cap.isOpened():
            ret, frame = self.cap.read()
            if ret and frame is not None:
                self.camera_ready = True
                return True
            else:
                self.cap.release()
                self.camera_ready = False
        
        self.camera_ready = False
        return False
    
    def video_loop(self):
        while self.running and not rospy.is_shutdown():
            ret, frame = self.cap.read()
            if ret:
                self.frame_count += 1
                self.current_frame = frame.copy()
                self.publish_image(frame)
                
                # 每0.5秒发布一次状态
                current_time = time.time()
                if current_time - self.last_state_pub_time >= self.state_pub_interval:
                    self.camera_ready = True
                    self.publish_state()
                    self.last_state_pub_time = current_time
                
                if self.frame_count % 100 == 0:
                    fps = 100 / (time.time() - self.start_time)
                    self.start_time = time.time()
                    self.frame_count = 0
            else:
                # 视频流读取失败时发布false状态
                self.camera_ready = False
                current_time = time.time()
                if current_time - self.last_state_pub_time >= self.state_pub_interval:
                    self.publish_state()
                    self.last_state_pub_time = current_time
                
                if self.running:
                    time.sleep(0.1)
    
    # ==================== 预置点控制功能 ====================
    def preset_control_callback(self, msg):
        """预置点控制回调函数"""
        try:
            command = msg.data.strip().lower()
            
            if ',' in command:
                action, preset_str = command.split(',', 1)
                action = action.strip()
                preset_str = preset_str.strip()
                
                if preset_str.isdigit():
                    preset_num = int(preset_str)
                    if 1 <= preset_num <= 255:
                        if action == "go":
                            self.goto_preset(preset_num)
                        elif action == "set":
                            self.set_preset(preset_num)
                        elif action == "clear" or action == "clean":
                            self.clear_preset(preset_num)
                        else:
                            print(f"未知的预置点命令: {action}")
                    else:
                        print(f"预置点编号超出范围(1-255): {preset_num}")
                else:
                    print(f"无效的预置点编号: {preset_str}")
            else:
                print(f"命令格式错误，应为: action,number (如: go,1)")
        
        except Exception as e:
            print(f"处理预置点命令时出错: {e}")
    
    def goto_preset(self, preset_num):
        """转到预置点"""
        if self.user_id < 0:
            print("请先登录摄像头")
            return False

        result = self.sdk.NET_DVR_PTZPreset_Other(
            self.user_id, self.channel, 39, preset_num
        )
        
        if result:
            print(f"成功转到预置点 {preset_num}")
            return True
        else:
            error = self.sdk.NET_DVR_GetLastError()
            print(f"转到预置点 {preset_num} 失败: {error}")
            return False
    
    def set_preset(self, preset_num):
        """设置预置点"""
        if self.user_id < 0:
            print("请先登录摄像头")
            return False
        
        result = self.sdk.NET_DVR_PTZPreset_Other(
            self.user_id, self.channel, 8, preset_num
        )
        
        if result:
            print(f"成功设置预置点 {preset_num}")
            return True
        else:
            error = self.sdk.NET_DVR_GetLastError()
            print(f"设置预置点 {preset_num} 失败: {error}")
            return False
    
    def clear_preset(self, preset_num):
        """清除预置点"""
        if self.user_id < 0:
            print("请先登录摄像头")
            return False
        
        result = self.sdk.NET_DVR_PTZPreset_Other(
            self.user_id, self.channel, 9, preset_num
        )
        
        if result:
            print(f"成功清除预置点 {preset_num}")
            return True
        else:
            error = self.sdk.NET_DVR_GetLastError()
            print(f"清除预置点 {preset_num} 失败: {error}")
            return False
    
    # ==================== 云台控制函数 ====================
    def ptz_control_with_speed(self, command, stop=0, speed=None):
        """带速度的云台控制"""
        if speed is None:
            speed = self.ptz_speed
        
        speed = max(1, min(7, speed))
        
        try:
            result = self.sdk.NET_DVR_PTZControlWithSpeed_Other(
                self.user_id, self.channel, command, stop, speed
            )
        except AttributeError:
            try:
                result = self.sdk.NET_DVR_PTZControlWithSpeed(
                    self.user_id, self.channel, command, stop, speed
                )
            except AttributeError:
                print("警告：未找到带速度的控制函数，使用普通控制函数")
                result = self.sdk.NET_DVR_PTZControl_Other(
                    self.user_id, self.channel, command, stop
                )
        
        if not result:
            error = self.sdk.NET_DVR_GetLastError()
            print(f"云台控制失败: {error}")
            return False
            
        return True
    
    # ==================== 回调函数 ====================
    def control_callback(self, msg):
        command_char = msg.data.strip().lower()
        
        if command_char == 'c':
            self.stop_all_movement()
        elif command_char in ['w', 's', 'a', 'd']:
            self.start_continuous_movement(command_char)
        elif command_char in self.PTZ_COMMANDS:
            self.execute_single_command(command_char)
    
    def speed_callback(self, msg):
        speed_cmd = msg.data.strip().lower()
        
        if speed_cmd == '+':
            self.adjust_speed(1)
        elif speed_cmd == '-':
            self.adjust_speed(-1)
        elif speed_cmd.isdigit():
            speed = int(speed_cmd)
            if 1 <= speed <= 7:
                self.set_speed(speed)
    
    def wiper_callback(self, msg):
        self.control_wiper(msg.data)
    
    def light_callback(self, msg):
        if not msg.data:  
            self.ptz_control_with_speed(2, 0, 1)  
        else:  
            self.ptz_control_with_speed(2, 1, 0)
    
    # ==================== 运动控制函数 ====================
    def start_continuous_movement(self, direction):
        if self.moving_direction:
            self.stop_movement(self.moving_direction)
        
        if direction in self.PTZ_COMMANDS:
            command = self.PTZ_COMMANDS[direction]
            result = self.ptz_control_with_speed(command, 0, self.ptz_speed)
            if result:
                self.moving_direction = direction
    
    def stop_movement(self, direction):
        if direction in self.PTZ_COMMANDS:
            command = self.PTZ_COMMANDS[direction]
            self.ptz_control_with_speed(command, 1, self.ptz_speed)
    
    def stop_all_movement(self):
        if self.moving_direction:
            self.stop_movement(self.moving_direction)
            self.moving_direction = None
    
    def execute_single_command(self, command_char):
        if command_char in self.PTZ_COMMANDS:
            command = self.PTZ_COMMANDS[command_char]
            
            if command_char == '3':  # 开启灯光
                self.publish_light_command(True)
            elif command_char == '4':  # 关闭灯光
                self.publish_light_command(False)
            elif command_char == '9':  # 打开雨刷
                self.control_wiper(True)
            elif command_char == '0':  # 关闭雨刷  
                self.control_wiper(False)
            else:  # 焦距、焦点、光圈控制
                self.ptz_control_with_speed(command, 0, 3)
                threading.Timer(0.5, lambda: self.ptz_control_with_speed(command, 1, 3)).start()
    
    # ==================== 辅助功能函数 ====================
    def adjust_speed(self, delta):
        new_speed = self.ptz_speed + delta
        if 1 <= new_speed <= 7:
            self.ptz_speed = new_speed
            print(f"云台速度调整为: {self.ptz_speed}")
    
    def set_speed(self, speed):
        if 1 <= speed <= 7:
            self.ptz_speed = speed
            print(f"云台速度设置为: {self.ptz_speed}")
    
    def control_wiper(self, turn_on=True):
        if turn_on and not self.function_states['wiper']:
            self.function_states['wiper'] = True
            self.publish_wiper_status(True)
            wiper_thread = threading.Thread(target=self.wiper_worker)
            wiper_thread.daemon = True
            wiper_thread.start()
            return True
            
        elif not turn_on and self.function_states['wiper']:
            self.function_states['wiper'] = False
            self.publish_wiper_status(False)
            self.ptz_control_with_speed(3, 1, 3)
            return True
        
        return False
    
    def wiper_worker(self):
        while self.function_states['wiper'] and self.running:
            self.ptz_control_with_speed(3, 0, 3)
            time.sleep(1)
            self.ptz_control_with_speed(3, 1, 3)
            time.sleep(2)
    
    # ==================== 发布函数 ====================
    def publish_image(self, frame):
        try:
            ros_image = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            ros_image.header.stamp = rospy.Time.now()
            ros_image.header.frame_id = "pan_tilt_camera"
            self.image_pub.publish(ros_image)
        except Exception as e:
            rospy.logerr(f"发布图像话题失败: {e}")
    
    def publish_wiper_status(self, status):
        try:
            wiper_msg = Bool()
            wiper_msg.data = status
            self.wiper_pub.publish(wiper_msg)
        except Exception as e:
            rospy.logerr(f"发布雨刷话题失败: {e}")
    
    def publish_light_command(self, turn_on=True):
        try:
            light_msg = Bool()
            light_msg.data = turn_on  
            self.light_pub.publish(light_msg)
        except Exception as e:
            rospy.logerr(f"发布灯光话题失败: {e}")
    
    def publish_control_command(self, command_char):
        try:
            control_msg = String()
            control_msg.data = command_char
            self.control_pub.publish(control_msg)
        except Exception as e:
            rospy.logerr(f"发布云台控制话题失败: {e}")
    
    def publish_state(self):
        """发布相机状态"""
        try:
            state_msg = Bool()
            # 检查相机是否就绪：已登录、视频流已打开且正在运行
            self.camera_ready = (self.user_id >= 0 and 
                                hasattr(self, 'cap') and 
                                self.cap is not None and 
                                self.cap.isOpened() and 
                                self.running)
            state_msg.data = self.camera_ready
            self.state_pub.publish(state_msg)
        except Exception as e:
            rospy.logerr(f"发布状态话题失败: {e}")
    
    # ==================== 其他功能 ====================
    def capture_image(self):
        if not hasattr(self, 'current_frame') or self.current_frame is None:
            return False
            
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"/home/aihit/aihitplt_ws/src/aihitplt_pan_tilt_camera/img/image_{timestamp}.jpg"
        
        cv2.imwrite(filename, self.current_frame)
        return True
    
    def handle_key_press(self, key_char):
        current_time = time.time()
        if hasattr(self, 'last_key_time'):
            if current_time - self.last_key_time < 0.2:
                return
        self.last_key_time = current_time
        
        if key_char == 'q':
            self.shutdown()
        elif key_char == 'f':
            self.capture_image()
        elif key_char == 'c':
            self.publish_control_command('c')
        elif key_char == '+':
            self.adjust_speed(1)
        elif key_char == '-':
            self.adjust_speed(-1)
        elif key_char in ['w', 's', 'a', 'd', '1', '2', '3', '4', '5', '6', '7', '8', '9', '0']:
            self.publish_control_command(key_char)
    
    def cleanup(self):
        self.stop_all_movement()
        
        if self.function_states['wiper']:
            self.control_wiper(False)
            
        if hasattr(self, 'cap'):
            self.cap.release()
            self.camera_ready = False
            self.publish_state()  # 发布最终状态
            
        if hasattr(self, 'user_id') and self.user_id >= 0:
            self.sdk.NET_DVR_Logout(self.user_id)
            
        self.sdk.NET_DVR_Cleanup()
    
    def shutdown(self):
        if not self.running:
            return
            
        self.running = False
        self.cleanup()
        os._exit(0)
    
    def run(self):
        try:
            rospy.spin()
        except KeyboardInterrupt:
            self.shutdown()

# ==================== GUI部分（保持原样） ====================
if gui_enabled:
    class CameraWindow(QMainWindow):
        def __init__(self, camera_node, app):
            super().__init__()
            self.camera_node = camera_node
            self.app = app
            self.setup_ui()
            self.setup_timer()
            
        def setup_ui(self):
            self.setWindowTitle("pan_camera")
            self.setGeometry(100, 100, 1000, 800)
            
            central_widget = QWidget()
            self.setCentralWidget(central_widget)
            layout = QVBoxLayout(central_widget)
            
            self.video_label = QLabel()
            self.video_label.setAlignment(Qt.AlignCenter)
            self.video_label.setMinimumSize(800, 600)
            self.video_label.setStyleSheet("border: 2px solid gray; background-color: black;")
            layout.addWidget(self.video_label)
            
            self.status_label = QLabel("等待视频流...")
            self.status_label.setAlignment(Qt.AlignCenter)
            layout.addWidget(self.status_label)
            
            self.speed_label = QLabel(f"云台速度: {self.camera_node.ptz_speed}")
            self.speed_label.setAlignment(Qt.AlignCenter)
            layout.addWidget(self.speed_label)
            
            self.help_text = QTextEdit()
            self.help_text.setReadOnly(True)
            self.help_text.setMaximumHeight(140)
            help_content = """
            控制说明:
            1:焦距变大  2:焦距变小  3:灯光开启  4:灯光关闭  5:焦点前调
            6:焦点后调  7:光圈扩大  8:光圈缩小  9:打开雨刷  0:关闭雨刷
            w:云台上仰  s:云台下俯  a:云台左转  d:云台右转
            +:增加速度  -:降低速度  f:截图      c:停止移动  q:退出程序
            速度范围: 1-7 
            """
            self.help_text.setText(help_content)
            layout.addWidget(self.help_text)
        
        def setup_timer(self):
            self.timer = QTimer()
            self.timer.timeout.connect(self.update_display)
            self.timer.start(33)
            
        def update_display(self):
            if (hasattr(self.camera_node, 'current_frame') and 
                self.camera_node.current_frame is not None and
                self.camera_node.running):
                
                frame = self.camera_node.current_frame
                rgb_image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                h, w, ch = rgb_image.shape
                
                qt_image = QImage(rgb_image.data, w, h, ch * w, QImage.Format_RGB888)
                pixmap = QPixmap.fromImage(qt_image)
                scaled_pixmap = pixmap.scaled(self.video_label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
                self.video_label.setPixmap(scaled_pixmap)
                
                status = "就绪"
                if self.camera_node.moving_direction:
                    direction_names = {'w': "上仰", 's': "下俯", 'a': "左转", 'd': "右转"}
                    status = f"移动中: {direction_names.get(self.camera_node.moving_direction, '未知')}"
                
                if self.camera_node.function_states['wiper']:
                    status += " | 雨刷开"
                    
                self.status_label.setText(status)
                self.speed_label.setText(f"云台速度: {self.camera_node.ptz_speed}")
        
        def keyPressEvent(self, event):
            key_char = event.text().lower()
            
            if key_char in ['q', 'f', 'c', '+', '-', '1', '2', '3', '4', '5', '6', '7', '8', '9', '0', 'w', 's', 'a', 'd']:
                self.camera_node.handle_key_press(key_char)
            else:
                super().keyPressEvent(event)
        
        def closeEvent(self, event):
            self.camera_node.shutdown()
            event.accept()

def signal_handler(sig, frame):
    os._exit(0)

def main():
    signal.signal(signal.SIGINT, signal_handler)
    
    camera_node = HikvisionCameraNode()
    if camera_node.user_id < 0:
        sys.exit(1)
    
    if gui_enabled:
        app = QApplication(sys.argv)
        window = CameraWindow(camera_node, app)
        window.show()
        
        ros_thread = threading.Thread(target=camera_node.run)
        ros_thread.daemon = True
        ros_thread.start()
        
        app_exec = app.exec_()
        sys.exit(app_exec)
    else:
        camera_node.run()

if __name__ == "__main__":
    main()