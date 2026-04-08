#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import tkinter as tk
from tkinter import ttk, messagebox
import threading
import subprocess
import os
import time
import yaml
import cv2
from PIL import Image, ImageTk
import socket
import struct
import rospy
import rospkg
from std_msgs.msg import String, Bool
from sensor_msgs.msg import Image as RosImage
from cv_bridge import CvBridge
import numpy as np
import glob
from datetime import datetime
import psutil

class PanTiltCameraController:
    def __init__(self, root):
        self.root = root
        self.root.title("云台相机控制系统")
        self.root.geometry("680x780")
        
        # 相机相关变量
        self.camera_ip = None
        self.camera_connected = False
        self.capture = None
        self.camera_thread = None
        self.stop_camera_thread = False
        
        # ROS相关变量
        self.ros_process = None
        self.ros_running = False
        self.ros_pid = None
        self.ros_node_initialized = False
        
        # ROS话题相关
        self.control_pub = None
        self.wiper_pub = None
        self.light_pub = None
        self.image_sub = None
        self.CONTROL_TOPIC = "/pan_tilt_camera_control"
        self.WIPER_TOPIC = "/pan_tilt_camera_wiper"
        self.LIGHT_TOPIC = "/pan_tilt_camera_light"
        self.IMAGE_TOPIC = "/pan_tilt_camera/image"
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # ROS包路径
        self.pkg_path = None
        self.config_file = None
        self.screenshot_dir = None
        self._init_ros_path()
        
        # 当前帧
        self.current_frame = None
        self.frame_lock = threading.Lock()
        
        # 视频更新定时器
        self.video_timer = None
        
        # 创建界面
        self.create_widgets()
        
        # 加载保存的IP地址
        self.load_saved_ip()
        
        # 绑定关闭事件
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        
        # 截图计数器
        self.capture_count = 0
        
    def _init_ros_path(self):
        """初始化ROS包路径"""
        try:
            rospack = rospkg.RosPack()
            self.pkg_path = rospack.get_path('aihitplt_hardware_test')
            self.config_file = os.path.join(self.pkg_path, 'config', 'camera_ip.yaml')
            self.screenshot_dir = os.path.join(self.pkg_path, 'img')
            os.makedirs(self.screenshot_dir, exist_ok=True)
            print(f"找到ROS包路径: {self.pkg_path}")
        except Exception as e:
            print(f"ROS包加载警告: {e}")
            # 使用默认路径
            self.pkg_path = "/home/aihit/aihitplt_ws/src/aihitplt_pan_tilt_camera"
            self.config_file = os.path.join(self.pkg_path, 'config', 'camera_ip.yaml')
            self.screenshot_dir = "/home/aihit/aihitplt_ws/src/aihitplt_pan_tilt_camera/img"
            os.makedirs(self.screenshot_dir, exist_ok=True)
    
    def init_ros_node(self):
        """初始化ROS节点"""
        try:
            if not rospy.is_shutdown() and not self.ros_node_initialized:
                rospy.init_node('pan_tilt_camera_controller_gui', anonymous=True, disable_signals=True)
                
                # 创建发布器
                self.control_pub = rospy.Publisher(self.CONTROL_TOPIC, String, queue_size=10)
                self.wiper_pub = rospy.Publisher(self.WIPER_TOPIC, Bool, queue_size=10)
                self.light_pub = rospy.Publisher(self.LIGHT_TOPIC, Bool, queue_size=10)
                
                # 订阅图像话题
                self.image_sub = rospy.Subscriber(
                    self.IMAGE_TOPIC,
                    RosImage,
                    self.image_callback
                )
                
                self.ros_node_initialized = True
                print("ROS节点初始化成功")
                
                return True
                
        except Exception as e:
            print(f"ROS节点初始化失败: {e}")
            return False
    
    def image_callback(self, msg):
        """ROS图像话题回调"""
        try:
            # 转换ROS图像为OpenCV格式
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            with self.frame_lock:
                self.current_frame = cv_image
            
                
        except Exception as e:
            print(f"图像回调错误: {e}")
    
    def create_widgets(self):
        """创建界面组件"""
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.pack(fill=tk.BOTH, expand=True)
        
        # ========== 第一部分：相机连接 ==========
        frame1 = ttk.LabelFrame(main_frame, text="相机连接", padding=10)
        frame1.pack(fill=tk.X, pady=(0, 10))
        
        # IP地址输入部分
        ip_frame = ttk.Frame(frame1)
        ip_frame.pack(fill=tk.X, pady=5)
        
        ttk.Label(ip_frame, text="IP地址:").pack(side=tk.LEFT, padx=(0, 5))
        
        self.ip_entry = ttk.Entry(ip_frame, width=20)
        self.ip_entry.pack(side=tk.LEFT, padx=(0, 15))
        self.ip_entry.insert(0, "192.168.2.64")
        
        # 连接按钮
        self.connect_btn = ttk.Button(
            ip_frame, 
            text="连接相机",
            command=self.toggle_camera_connection,
            width=12
        )
        self.connect_btn.pack(side=tk.LEFT)
        
        # ========== 第二部分：相机画面显示 ==========
        frame2 = ttk.LabelFrame(main_frame, text="相机画面", padding=10)
        frame2.pack(fill=tk.BOTH, expand=True, pady=(0, 10))
        
        # 创建视频显示区域
        self.video_label = tk.Label(frame2, bg="black", relief=tk.SUNKEN)
        self.video_label.pack(fill=tk.BOTH, expand=True)
        
        # ========== 第三部分：相机控制（修复按钮标签） ==========
        frame3 = ttk.LabelFrame(main_frame, text="相机控制", padding=10)
        frame3.pack(fill=tk.X, pady=(0, 10))
        
        # 第一行：移动控制
        move_frame = ttk.Frame(frame3)
        move_frame.pack(fill=tk.X, pady=(0, 10))
        
        # 移动控制按钮（修改为新的控制方法）
        self.move_up_btn = ttk.Button(move_frame, text="上", width=12, 
                                     command=self.control_move_up)
        self.move_up_btn.pack(side=tk.LEFT, padx=16)
        
        self.move_down_btn = ttk.Button(move_frame, text="下", width=12,
                                       command=self.control_move_down)
        self.move_down_btn.pack(side=tk.LEFT, padx=16)
        
        self.move_left_btn = ttk.Button(move_frame, text="左", width=12,
                                       command=self.control_move_left)
        self.move_left_btn.pack(side=tk.LEFT, padx=16)
        
        self.move_right_btn = ttk.Button(move_frame, text="右", width=12,
                                        command=self.control_move_right)
        self.move_right_btn.pack(side=tk.LEFT, padx=16)
        
        self.stop_btn = ttk.Button(move_frame, text="停止", width=12,
                                  command=self.control_stop)
        self.stop_btn.pack(side=tk.LEFT, padx=16)
        
        # 第二行：功能控制
        func_frame = ttk.Frame(frame3)
        func_frame.pack(fill=tk.X)
        
        # 功能控制按钮（修改为新的控制方法）
        self.wiper_on_btn = ttk.Button(func_frame, text="雨刷开", width=12,
                                      command=lambda: self.control_wiper(True))
        self.wiper_on_btn.pack(side=tk.LEFT, padx=31)
        
        self.wiper_off_btn = ttk.Button(func_frame, text="雨刷关", width=12,
                                       command=lambda: self.control_wiper(False))
        self.wiper_off_btn.pack(side=tk.LEFT, padx=31)
        
        self.light_btn = ttk.Button(func_frame, text="灯光开启", width=12,
                                   command=self.control_light)
        self.light_btn.pack(side=tk.LEFT, padx=31)
        
        self.capture_btn = ttk.Button(func_frame, text="截图", width=12,
                                     command=self.capture_image)
        self.capture_btn.pack(side=tk.LEFT, padx=31)
        
        # ========== 第四和第五部分：配置和启动 ==========
        config_launch_frame = ttk.Frame(main_frame)
        config_launch_frame.pack(fill=tk.X, pady=(0, 10))
        
        # 第四部分：IP配置（左半部分）
        frame4 = ttk.LabelFrame(config_launch_frame, text="IP配置", padding=10)
        frame4.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 5))
        
        self.save_ip_btn = ttk.Button(
            frame4,
            text="保存IP地址",
            command=self.save_ip_address,
            width=20
        )
        self.save_ip_btn.pack()
        
        # 第五部分：ROS启动（右半部分）
        frame5 = ttk.LabelFrame(config_launch_frame, text="ROS启动", padding=10)
        frame5.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=(5, 0))
        
        # 创建启动按钮
        self.launch_btn = tk.Button(
            frame5,
            text="启动launch文件",
            command=self.toggle_ros_launch,
            width=20,
            bg="lightgray",
            fg="black"
        )
        self.launch_btn.pack()
        
        # ========== 状态栏 ==========
        self.status_var = tk.StringVar(value="就绪")
        status_bar = ttk.Label(main_frame, textvariable=self.status_var, 
                              relief=tk.SUNKEN, anchor=tk.W)
        status_bar.pack(side=tk.BOTTOM, fill=tk.X, pady=(10, 0))
        
        # 初始状态
        self.update_button_states()
        
        # 启动视频更新定时器
        self.start_video_update()
    
    def start_video_update(self):
        """启动视频更新定时器"""
        # 取消现有的定时器
        if self.video_timer:
            try:
                self.video_timer.cancel()
            except:
                pass
        
        # 使用after方法在主线程中调度视频更新
        self.update_video_display()
    
    def update_video_display(self):
        """更新视频显示"""
        try:
            display_frame = None
            
            with self.frame_lock:
                if self.current_frame is not None:
                    display_frame = self.current_frame.copy()
            
            if display_frame is not None:
                # 调整图像大小以适应显示区域
                display_frame = self.resize_frame(display_frame, 640, 360)
                
                # 转换为PhotoImage
                rgb_frame = cv2.cvtColor(display_frame, cv2.COLOR_BGR2RGB)
                img = Image.fromarray(rgb_frame)
                imgtk = ImageTk.PhotoImage(image=img)
                
                # 更新显示
                self.video_label.imgtk = imgtk
                self.video_label.config(image=imgtk)
            
        except Exception as e:
            print(f"更新显示错误: {e}")
        
        # 每33毫秒（约30fps）更新一次
        self.video_timer = self.root.after(33, self.update_video_display)
    
    def toggle_camera_connection(self):
        """切换相机连接状态"""
        if not self.camera_connected:
            self.connect_camera()
        else:
            self.disconnect_camera()
    
    def connect_camera(self):
        """连接相机"""
        ip_address = self.ip_entry.get().strip()
        if not ip_address:
            messagebox.showwarning("警告", "请输入IP地址")
            return
        
        # 验证IP地址格式
        try:
            socket.inet_aton(ip_address)
        except socket.error:
            messagebox.showwarning("警告", "IP地址格式不正确")
            return
        
        self.camera_ip = ip_address
        
        try:
            # 构造RTSP URL
            rtsp_url = f"rtsp://admin:abcd1234@{ip_address}:554/h264/ch1/main/av_stream"
            print(f"尝试连接RTSP: {rtsp_url}")
            
            # 尝试连接相机
            self.capture = cv2.VideoCapture(rtsp_url)
            
            # 设置缓冲区大小
            self.capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            # 设置读取超时
            self.capture.set(cv2.CAP_PROP_OPEN_TIMEOUT_MSEC, 5000)
            
            # 测试连接
            if not self.capture.isOpened():
                raise Exception("无法打开视频流")
            
            # 尝试读取一帧确认连接正常
            ret, frame = self.capture.read()
            if not ret or frame is None:
                self.capture.release()
                raise Exception("连接成功但无法读取帧")
            
            self.camera_connected = True
            self.stop_camera_thread = False
            
            # 更新按钮状态
            self.connect_btn.config(text="断开相机")
            self.launch_btn.config(state="disabled")
            self.save_ip_btn.config(state="disabled")
            
            # 启动视频读取线程
            self.camera_thread = threading.Thread(
                target=self.read_camera_feed,
                daemon=True
            )
            self.camera_thread.start()
            
            self.update_status(f"已连接到相机: {ip_address}")

            
            # 尝试初始化ROS节点（如果还没有）
            if not self.ros_node_initialized:
                try:
                    # 尝试初始化ROS节点
                    rospy.init_node('camera_controller_gui', anonymous=True, disable_signals=True)
                    self.control_pub = rospy.Publisher(self.CONTROL_TOPIC, String, queue_size=10)
                    self.wiper_pub = rospy.Publisher(self.WIPER_TOPIC, Bool, queue_size=10)
                    self.light_pub = rospy.Publisher(self.LIGHT_TOPIC, Bool, queue_size=10)
                    self.ros_node_initialized = True
                    self.update_status("ROS节点初始化成功")
                except Exception as e:
                    self.update_status(f"ROS节点初始化失败: {e}")
                    # 继续连接相机，但使用直接控制
            
        except Exception as e:
            messagebox.showerror("连接失败", f"无法连接相机:\n{e}")
            self.update_status("连接失败")
            
            if self.capture:
                self.capture.release()
                self.capture = None
    
    def disconnect_camera(self):
        """断开相机连接"""
        if self.camera_connected:
            self.camera_connected = False
            self.stop_camera_thread = True
            
            if self.capture:
                try:
                    self.capture.release()
                except:
                    pass
                self.capture = None
            
            # 等待线程结束
            if self.camera_thread and self.camera_thread.is_alive():
                self.camera_thread.join(timeout=1.0)
            
            # 更新按钮状态
            self.connect_btn.config(text="连接相机")
            self.launch_btn.config(state="normal")
            self.save_ip_btn.config(state="normal")
            
            # 清空视频显示
            with self.frame_lock:
                self.current_frame = None
            self.video_label.config(image='')
            
            self.update_status("已断开相机连接")
    
    def read_camera_feed(self):
        """读取相机视频流"""
        while self.camera_connected and not self.stop_camera_thread:
            try:
                if self.capture and self.capture.isOpened():
                    ret, frame = self.capture.read()
                    
                    if ret and frame is not None:
                        with self.frame_lock:
                            self.current_frame = frame
                    else:
                        print("读取帧失败，尝试重新连接...")
                        # 尝试重新读取
                        time.sleep(0.1)
                
                time.sleep(0.03)  # 约30fps
                
            except Exception as e:
                if not self.stop_camera_thread:
                    print(f"视频读取错误: {e}")
                    self.root.after(0, self.disconnect_camera)
                break
    
    def resize_frame(self, frame, max_width, max_height):
        """调整帧大小"""
        if frame is None:
            return None
            
        height, width = frame.shape[:2]
        
        # 计算缩放比例
        scale = min(max_width / width, max_height / height)
        
        if scale < 1:
            new_width = int(width * scale)
            new_height = int(height * scale)
            return cv2.resize(frame, (new_width, new_height))
        
        return frame
    
    def control_move_up(self):
        """控制向上移动"""
        if self.ros_node_initialized:
            self.send_control_command('w')
        else:
            self.send_direct_control_command('w')
    
    def control_move_down(self):
        """控制向下移动"""
        if self.ros_node_initialized:
            self.send_control_command('s')
        else:
            self.send_direct_control_command('s')
    
    def control_move_left(self):
        """控制向左移动"""
        if self.ros_node_initialized:
            self.send_control_command('a')
        else:
            self.send_direct_control_command('a')
    
    def control_move_right(self):
        """控制向右移动"""
        if self.ros_node_initialized:
            self.send_control_command('d')
        else:
            self.send_direct_control_command('d')
    
    def control_stop(self):
        """控制停止"""
        if self.ros_node_initialized:
            self.send_control_command('c')
        else:
            self.send_direct_control_command('c')
    
    def control_wiper(self, turn_on):
        """控制雨刷"""
        if self.ros_node_initialized:
            self.send_wiper_command(turn_on)
        else:
            self.send_direct_wiper_command(turn_on)
    
    def control_light(self):
        """控制灯光"""
        if self.ros_node_initialized:
            self.send_light_command()
        else:
            self.send_direct_light_command()
    
    def send_control_command(self, command):
        """发送ROS控制命令"""
        if not self.ros_node_initialized:
            messagebox.showwarning("警告", "ROS节点未初始化")
            return
        
        try:
            control_msg = String()
            control_msg.data = command
            self.control_pub.publish(control_msg)
            self.update_status(f"发送控制命令: {command}")
        except Exception as e:
            print(f"发送控制命令失败: {e}")
    
    def send_direct_control_command(self, command):
        """直接发送控制命令到相机（不通过ROS）"""
        try:
            if not self.camera_ip:
                messagebox.showwarning("警告", "请先连接相机")
                return
            
            # 根据命令构造不同的URL
            commands_map = {
                'w': 'ptzmove?pos=up',      # 上
                's': 'ptzmove?pos=down',    # 下
                'a': 'ptzmove?pos=left',    # 左
                'd': 'ptzmove?pos=right',   # 右
                'c': 'ptzstop',             # 停止
            }
            
            if command in commands_map:
                # 构建控制URL（根据相机API调整）
                control_url = f"http://{self.camera_ip}/cgi-bin/ptz.cgi?action={commands_map[command]}"
                
                # 发送HTTP请求
                import requests
                response = requests.get(control_url, timeout=2, auth=('admin', 'abcd1234'))
                
                if response.status_code == 200:
                    self.update_status(f"直接控制: {command} 成功")
                else:
                    self.update_status(f"直接控制: {command} 失败")
            else:
                print(f"未知命令: {command}")
                
        except ImportError:
            messagebox.showerror("错误", "需要requests库，请安装: pip install requests")
        except Exception as e:
            self.update_status(f"直接控制失败: {e}")
    
    def send_wiper_command(self, turn_on):
        """发送ROS雨刷控制命令"""
        if not self.ros_node_initialized:
            messagebox.showwarning("警告", "ROS节点未初始化")
            return
        
        try:
            wiper_msg = Bool()
            wiper_msg.data = turn_on
            self.wiper_pub.publish(wiper_msg)
            status = "开启" if turn_on else "关闭"
            self.update_status(f"发送雨刷{status}命令")
        except Exception as e:
            print(f"发送雨刷命令失败: {e}")
    
    def send_direct_wiper_command(self, turn_on):
        """直接控制雨刷"""
        try:
            if not self.camera_ip:
                messagebox.showwarning("警告", "请先连接相机")
                return
            
            # 根据相机API构造雨刷控制URL
            action = "start" if turn_on else "stop"
            control_url = f"http://{self.camera_ip}/cgi-bin/wiper.cgi?action={action}"
            
            import requests
            response = requests.get(control_url, timeout=2, auth=('admin', 'abcd1234'))
            
            if response.status_code == 200:
                status = "开启" if turn_on else "关闭"
                self.update_status(f"雨刷{status}成功")
            else:
                self.update_status(f"雨刷控制失败")
                
        except ImportError:
            messagebox.showerror("错误", "需要requests库，请安装: pip install requests")
        except Exception as e:
            self.update_status(f"雨刷控制失败: {e}")
    
    def send_light_command(self):
        """发送ROS灯光控制命令"""
        if not self.ros_node_initialized:
            messagebox.showwarning("警告", "ROS节点未初始化")
            return
        
        try:
            light_msg = Bool()
            light_msg.data = True  # 总是发送True
            self.light_pub.publish(light_msg)
            self.update_status("发送灯光开启命令")
        except Exception as e:
            print(f"发送灯光命令失败: {e}")
    
    def send_direct_light_command(self):
        """直接控制灯光"""
        try:
            if not self.camera_ip:
                messagebox.showwarning("警告", "请先连接相机")
                return
            
            # 灯光控制URL
            control_url = f"http://{self.camera_ip}/cgi-bin/light.cgi?action=toggle"
            
            import requests
            response = requests.get(control_url, timeout=2, auth=('admin', 'abcd1234'))
            
            if response.status_code == 200:
                self.update_status("灯光控制成功")
            else:
                self.update_status("灯光控制失败")
                
        except ImportError:
            messagebox.showerror("错误", "需要requests库，请安装: pip install requests")
        except Exception as e:
            self.update_status(f"灯光控制失败: {e}")
    
    def capture_image(self):
        """截图功能"""
        try:
            display_frame = None
            with self.frame_lock:
                if self.current_frame is not None:
                    display_frame = self.current_frame.copy()
            
            if display_frame is not None:
                self.capture_count += 1
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                filename = f"camera_{timestamp}_{self.capture_count:04d}.jpg"
                filepath = os.path.join(self.screenshot_dir, filename)
                
                cv2.imwrite(filepath, display_frame)
                self.update_status(f"截图保存: {filename}")
                
                # 显示保存成功的消息
                messagebox.showinfo("截图成功", f"截图已保存到:\n{filepath}")
            else:
                messagebox.showwarning("警告", "没有可用的图像帧")
                
        except Exception as e:
            messagebox.showerror("截图失败", f"截图失败:\n{e}")
    
    def update_button_states(self):
        """更新按钮状态"""
        if self.camera_connected:
            # 相机连接时，禁用launch启动按钮
            self.launch_btn.config(state="disabled")
            self.save_ip_btn.config(state="disabled")
        elif self.ros_running:
            # ROS运行时，禁用相机连接按钮
            self.connect_btn.config(state="disabled")
            self.save_ip_btn.config(state="disabled")
        else:
            # 正常状态
            self.launch_btn.config(state="normal")
            self.save_ip_btn.config(state="normal")
            self.connect_btn.config(state="normal")
    
    def save_ip_address(self):
        """保存IP地址到配置文件"""
        if not self.pkg_path:
            messagebox.showerror("错误", "未找到ROS包路径")
            return
        
        ip_address = self.ip_entry.get().strip()
        if not ip_address:
            messagebox.showwarning("警告", "请输入IP地址")
            return
        
        try:
            # 验证IP地址格式
            socket.inet_aton(ip_address)
            
            # 创建config目录
            config_dir = os.path.join(self.pkg_path, 'config')
            os.makedirs(config_dir, exist_ok=True)
            
            # 保存配置到YAML文件
            config = {
                'camera_ip': ip_address,
                'last_updated': datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            }
            
            with open(self.config_file, 'w') as f:
                yaml.dump(config, f, default_flow_style=False)
            
            messagebox.showinfo("保存成功",
                              f"IP地址 {ip_address} 已保存成功\n"
                              f"配置文件: {self.config_file}")
            
            self.update_status(f"已保存IP地址: {ip_address}")
            
        except socket.error:
            messagebox.showwarning("警告", "IP地址格式不正确")
        except Exception as e:
            messagebox.showerror("保存失败", f"保存IP地址失败:\n{str(e)}")
    
    def load_saved_ip(self):
        """加载保存的IP地址"""
        if not os.path.exists(self.config_file):
            return
        
        try:
            with open(self.config_file, 'r') as f:
                config = yaml.safe_load(f)
            
            if config and 'camera_ip' in config:
                saved_ip = config['camera_ip']
                self.ip_entry.delete(0, tk.END)
                self.ip_entry.insert(0, saved_ip)
                self.update_status(f"已加载保存的IP地址: {saved_ip}")
                
        except Exception as e:
            print(f"加载保存的IP地址失败: {e}")
    
    def toggle_ros_launch(self):
        """切换ROS launch文件"""
        if not self.ros_running:
            self.start_ros_launch()
        else:
            self.stop_ros_launch()
    
    def start_ros_launch(self):
        """启动ROS launch文件"""
        try:
            # 构建roslaunch命令
            roslaunch_cmd = f'roslaunch aihitplt_hardware_test aihitplt_pan_tilt_camera.launch'
            
            print(f"启动命令: {roslaunch_cmd}")
            
            # 使用gnome-terminal打开新窗口
            cmd = [
                'gnome-terminal',
                '--title=云台相机系统 - ROS Launch',
                '--geometry=80x24+100+100',
                '--',
                'bash', '-c',
                f'{roslaunch_cmd}'
            ]
            
            # 启动终端
            self.ros_process = subprocess.Popen(
                cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                start_new_session=True
            )
            
            # 保存进程ID
            self.ros_pid = self.ros_process.pid
            self.ros_running = True
            
            # 更新按钮状态 - 按钮变绿
            self.launch_btn.config(
                text="关闭launch文件",
                bg="green",
                fg="white"
            )
            self.connect_btn.config(state="disabled")
            self.save_ip_btn.config(state="disabled")
            
            self.update_status("已启动ROS launch文件")
            
            # 等待ROS启动
            time.sleep(2)
            
            # 初始化ROS节点
            if not self.ros_node_initialized:
                threading.Thread(target=self.init_ros_node, daemon=True).start()
            
        except FileNotFoundError:
            messagebox.showerror("启动失败", "未找到gnome-terminal。")
            self.update_status("启动失败: 未找到gnome-terminal")
        except Exception as e:
            messagebox.showerror("启动失败", f"无法启动ROS launch文件:\n{e}")
            self.update_status(f"启动失败: {e}")
    
    def kill_process_tree(self, pid):
        """终止进程树"""
        try:
            process = psutil.Process(pid)
            children = process.children(recursive=True)
            
            # 先终止子进程
            for child in children:
                try:
                    child.terminate()
                except:
                    pass
            
            # 等待子进程结束aihitplt_pan_tilt_camera.launch
            try:
                process.terminate()
                process.wait(timeout=3)
            except:
                try:
                    process.kill()
                except:
                    pass
                    
        except psutil.NoSuchProcess:
            pass
        except Exception as e:
            print(f"终止进程树时出错: {e}")
    
    def stop_ros_launch(self):
        """停止ROS launch文件"""
        if self.ros_running:
            try:
                # 终止进程
                if self.ros_pid:
                    self.kill_process_tree(self.ros_pid)
                
                # 终止roscore和相关进程
                self._kill_ros_processes()
                
            except Exception as e:
                print(f"停止ROS进程时出错: {e}")
                
            finally:
                # 更新状态
                self.ros_running = False
                self.ros_process = None
                self.ros_pid = None
                
                # 清理ROS资源
                if self.ros_node_initialized:
                    try:
                        if self.image_sub:
                            self.image_sub.unregister()
                        rospy.signal_shutdown("GUI关闭")
                    except:
                        pass
                    self.ros_node_initialized = False
                
                # 恢复按钮状态
                self.launch_btn.config(
                    text="启动launch文件",
                    bg="lightgray",
                    fg="black"
                )
                self.connect_btn.config(state="normal")
                self.save_ip_btn.config(state="normal")
                
                # 清空视频显示
                with self.frame_lock:
                    self.current_frame = None
                self.video_label.config(image='')
                
                self.update_status("ROS进程已停止")
    
    def _kill_ros_processes(self):
        """终止所有与ROS相关的进程"""
        try:
            # 更精确的终止方式：只终止特定的launch文件
            launch_files_to_kill = [
                'aihitplt_pan_tilt_camera.launch'
            ]
            
            for launch_file in launch_files_to_kill:
                subprocess.run(['pkill', '-f', launch_file], 
                            stdout=subprocess.DEVNULL, 
                            stderr=subprocess.DEVNULL)
            
            time.sleep(1)
            
        except Exception as e:
            print(f"终止ROS进程时出错: {e}")
    
    def update_status(self, message):
        """更新状态栏"""
        timestamp = datetime.now().strftime("%H:%M:%S")
        self.status_var.set(f"[{timestamp}] {message}")
        print(f"[状态] {message}")
    
    def on_closing(self):
        """窗口关闭时的处理"""
        # 停止视频更新定时器
        if self.video_timer:
            try:
                self.root.after_cancel(self.video_timer)
            except:
                pass
        
        # 断开相机连接
        if self.camera_connected:
            self.disconnect_camera()
        
        # 停止ROS进程
        if self.ros_running:
            self.stop_ros_launch()
        
        # 关闭窗口
        self.root.destroy()

def main():
    """主函数"""
    # 检查DISPLAY环境变量
    if 'DISPLAY' not in os.environ:
        os.environ['DISPLAY'] = ':0'
    
    # 设置信号处理，当终端关闭时退出程序
    import signal
    
    def signal_handler(signum, frame):
        print(f"\n收到信号 {signum}，正在退出程序...")
        # 尝试安全退出
        try:
            if 'app' in locals() and hasattr(app, 'on_closing'):
                app.on_closing()
        except:
            pass
        os._exit(0)
    
    # 注册信号处理器
    signal.signal(signal.SIGTERM, signal_handler)
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGHUP, signal_handler)  # 终端挂断信号
    
    root = tk.Tk()
    
    # 设置窗口大小和位置
    window_width = 680
    window_height = 690
    screen_width = root.winfo_screenwidth()
    screen_height = root.winfo_screenheight()
    x = (screen_width - window_width) // 2
    y = (screen_height - window_height) // 2
    root.geometry(f"{window_width}x{window_height}+{x}+{y}")
    
    # 设置窗口最小大小
    root.minsize(600, 600)
    
    # 创建应用
    app = PanTiltCameraController(root)
    
    # 设置关闭协议
    root.protocol("WM_DELETE_WINDOW", app.on_closing)
    
    try:
        # 运行主循环
        root.mainloop()
    except KeyboardInterrupt:
        print("\n收到中断信号，正在退出...")
        app.on_closing()
    finally:
        # 确保程序完全退出
        os._exit(0)

if __name__ == "__main__":
    main()

if __name__ == "__main__":
    main()