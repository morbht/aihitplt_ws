#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# AI套件传感器系统测试程序

import tkinter as tk
from tkinter import ttk, messagebox, scrolledtext
import serial
import serial.tools.list_ports
import threading
import subprocess
import os
import time
import yaml
import rospkg
import struct
from datetime import datetime
import glob
import queue
import psutil
import rospy
from std_msgs.msg import String
import re

class AISensorSystemTester:
    def __init__(self, root):
        self.root = root
        self.root.title("AI套件传感器系统测试")
        self.root.geometry("600x600")  # 调整窗口大小为800x750
        
        # 串口相关
        self.serial_port = None
        self.serial_connected = False
        self.serial_thread = None
        self.stop_serial_thread = False
        
        # ROS相关
        self.ros_process = None
        self.ros_running = False
        self.ros_node_initialized = False
        self.ros_pid = None
        
        # ROS话题
        self.sensor_pub = None
        self.sensor_sub = None
        self.control_pub = None
        
        # 话题名称
        self.SENSOR_DATA_TOPIC = "/multi_ai/sensor_data"  
        self.CONTROL_TOPIC = "/multi_ai/pan_control"  
        
        # 数据解析相关
        self.FRAME_HEADER = b'\xAA\x55'
        self.FRAME_SIZE = 55
        self.FRAME_FORMAT = '<2sI 8H 2f 6f B'
        
        # 当前舵机角度
        self.current_h = 90
        self.current_v = 90
        
        # 获取ROS包路径
        self.pkg_path = None
        self.config_file = None
        self._init_paths()
        
        # 数据队列用于线程间通信
        self.sensor_data_queue = queue.Queue()
        
        # 当前模式：serial(串口模式) 或 ros(话题模式)
        self.current_mode = "serial"
        
        # 创建界面
        self.create_widgets()
        
        # 加载保存的串口
        self.load_saved_port()
        
        # 绑定关闭事件
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        
        # 启动数据更新线程
        self.update_sensor_data_thread = threading.Thread(
            target=self.update_sensor_data_from_queue, 
            daemon=True
        )
        self.update_sensor_data_thread.start()
    
    def _init_paths(self):
        """初始化路径配置"""
        try:
            rospack = rospkg.RosPack()
            self.pkg_path = rospack.get_path('aihitplt_hardware_test')
            self.config_file = os.path.join(self.pkg_path, 'config', 'multi_ai_port.yaml')
        except Exception as e:
            print(f"ROS包加载警告: {e}")
            default_path = "/home/aihit/aihitplt_ws/src/aihitplt_hardware_test"
            if os.path.exists(default_path):
                self.pkg_path = default_path
                self.config_file = os.path.join(self.pkg_path, 'config', 'multi_ai_port.yaml')
    
    def _normalize_port_path(self, port):
        """规范化串口路径格式"""
        if not port:
            return port
        
        if port.startswith('//dev/'):
            port = port.replace('//dev/', '/dev/')
        elif not port.startswith('/dev/'):
            port = f'/dev/{port}'
        
        return port
    
    def _update_button_states(self, mode="normal"):
        """统一更新按钮状态
        """
        if mode == "serial_connected":
            # 串口连接时的状态
            self.connect_btn.config(text="关闭AI套件传感器系统")
            self.refresh_btn.config(state="disabled")
            self.save_btn.config(state="disabled")
            self.launch_btn.config(state="disabled")
            self.send_angle_btn.config(state="normal")
            self.reset_angle_btn.config(state="normal")
            self.horizontal_scale.config(state="normal")
            self.vertical_scale.config(state="normal")
            self.current_mode = "serial"
            
        elif mode == "ros_running":
            # ROS运行时的状态
            self.connect_btn.config(state="disabled")
            self.refresh_btn.config(state="disabled")
            self.save_btn.config(state="disabled")
            self.send_angle_btn.config(state="normal")
            self.reset_angle_btn.config(state="normal")
            self.horizontal_scale.config(state="normal")
            self.vertical_scale.config(state="normal")
            self.current_mode = "ros"
            
            # 更新ROS按钮状态
            self.root.after(0, self._update_ros_button_running)
            
        elif mode == "normal":
            # 正常状态（无连接）
            self.connect_btn.config(text="连接AI套件传感器系统", state="normal")
            self.refresh_btn.config(state="normal")
            self.save_btn.config(state="normal" if self.pkg_path else "disabled")
            self.launch_btn.config(state="normal")
            self.send_angle_btn.config(state="disabled")
            self.reset_angle_btn.config(state="disabled")
            self.horizontal_scale.config(state="disabled")
            self.vertical_scale.config(state="disabled")
            self.current_mode = "serial"
            
            # 恢复ROS按钮颜色
            self.root.after(0, self._update_ros_button_normal)
    
    def _update_ros_button_running(self):
        """更新ROS按钮为运行状态"""
        self.launch_btn.config(
            text="关闭AI套件传感器系统launch文件",
            bg="green",
            fg="white"
        )
    
    def _update_ros_button_normal(self):
        """更新ROS按钮为正常状态"""
        if hasattr(self, 'default_button_bg'):
            self.launch_btn.config(
                text="启动AI套件传感器系统launch文件",
                bg=self.default_button_bg,
                fg=self.default_button_fg
            )
    
    def create_widgets(self):
        """创建界面"""
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.pack(fill=tk.BOTH, expand=True)
        
        # 第一部分：串口连接
        self._create_serial_connection_frame(main_frame)
        
        # 第二部分：传感器数据显示和舵机控制
        self._create_sensor_control_frame(main_frame)
        
        # 第三部分和第四部分
        self._create_bottom_frames(main_frame)
        
        # 状态栏
        self.status_var = tk.StringVar(value="就绪")
        status_bar = ttk.Label(main_frame, textvariable=self.status_var, 
                              relief=tk.SUNKEN, anchor=tk.W)
        status_bar.pack(side=tk.BOTTOM, fill=tk.X)
        
        # 初始化串口列表
        self.refresh_ports()
    
    def _create_serial_connection_frame(self, parent):
        """创建串口连接部分"""
        frame = ttk.LabelFrame(parent, text="串口连接", padding=8)
        frame.pack(fill=tk.X, pady=(0, 8))
        
        # 使用更紧凑的布局
        ttk.Label(frame, text="选择串口:").grid(row=0, column=0, padx=5, pady=3, sticky="w")
        
        self.port_combo = ttk.Combobox(frame, width=20, state="readonly")
        self.port_combo.grid(row=0, column=1, padx=5, pady=3, sticky="ew")
        
        self.refresh_btn = ttk.Button(frame, text="刷新", command=self.refresh_ports)
        self.refresh_btn.grid(row=0, column=2, padx=5, pady=3)
        
        self.connect_btn = ttk.Button(frame, text="连接系统", command=self.toggle_serial_connection)
        self.connect_btn.grid(row=0, column=3, padx=5, pady=3)
        
        # 使列自适应宽度
        frame.columnconfigure(1, weight=1)
    
    def _create_sensor_control_frame(self, parent):
        """创建传感器数据显示和舵机控制部分"""
        frame = ttk.LabelFrame(parent, text="传感器数据和控制", padding=8)
        frame.pack(fill=tk.BOTH, expand=True, pady=(0, 8))
        
        # 创建两个子框架，调整宽度比例
        left_frame = ttk.Frame(frame)
        left_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 5))
        
        right_frame = ttk.Frame(frame)
        right_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=(5, 0))
        
        # 设置比例：舵机控制部分占30%，传感器数据显示占70%
        left_frame.pack_propagate(False)
        left_frame.config(width=300)  # 压缩舵机控制部分宽度
        
        # 左边：舵机控制（压缩版）
        self._create_servo_control_frame(left_frame)
        
        # 右边：传感器数据显示（拓宽版）
        self._create_sensor_display_frame(right_frame)
    
    def _create_servo_control_frame(self, parent):
        """创建舵机控制框架 - 压缩版本"""
        frame = ttk.LabelFrame(parent, text="舵机控制", padding=8)
        frame.pack(fill=tk.BOTH, expand=True)
        
        # 使用紧凑的网格布局
        # 水平舵机控制
        ttk.Label(frame, text="水平 (0-180°):").grid(row=0, column=0, padx=3, pady=2, sticky="w")
        self.horizontal_scale = tk.Scale(frame, from_=0, to=180, orient=tk.HORIZONTAL, 
                                        length=120, command=self.on_scale_change)  # 缩短滑块
        self.horizontal_scale.set(90)
        self.horizontal_scale.grid(row=0, column=1, padx=3, pady=2, sticky="ew")
        self.horizontal_scale.config(state="disabled")
        
        # 垂直舵机控制
        ttk.Label(frame, text="垂直 (0-180°):").grid(row=1, column=0, padx=3, pady=2, sticky="w")
        self.vertical_scale = tk.Scale(frame, from_=0, to=180, orient=tk.HORIZONTAL, 
                                      length=120, command=self.on_scale_change)  # 缩短滑块
        self.vertical_scale.set(90)
        self.vertical_scale.grid(row=1, column=1, padx=3, pady=2, sticky="ew")
        self.vertical_scale.config(state="disabled")
        
        # 当前角度显示 - 使用更紧凑的字体
        self.angle_var = tk.StringVar(value="角度: 水平=90°, 垂直=90°")
        ttk.Label(frame, textvariable=self.angle_var, font=("Arial", 9)).grid(row=2, column=0, columnspan=2, pady=(6, 3))
        
        # 按钮框架 - 使用更紧凑的布局
        button_frame = ttk.Frame(frame)
        button_frame.grid(row=3, column=0, columnspan=2, pady=3, sticky="ew")
        button_frame.columnconfigure(0, weight=1)
        button_frame.columnconfigure(1, weight=1)
        
        # 发送角度按钮 - 使用更小的字体
        self.send_angle_btn = ttk.Button(
            button_frame,
            text="发送",
            command=self.send_servo_angles,
            state="disabled"
        )
        self.send_angle_btn.grid(row=0, column=0, padx=1, pady=1, sticky="ew")
        
        # 角度复位按钮 - 使用更小的字体
        self.reset_angle_btn = ttk.Button(
            button_frame,
            text="复位",
            command=self.reset_servo_angles,
            state="disabled"
        )
        self.reset_angle_btn.grid(row=0, column=1, padx=1, pady=1, sticky="ew")
        
        # 使列自适应宽度
        frame.columnconfigure(1, weight=1)
    
    def _create_sensor_display_frame(self, parent):
        """创建传感器数据显示框架 - 拓宽版本"""
        frame = ttk.LabelFrame(parent, text="传感器数据", padding=8)
        frame.pack(fill=tk.BOTH, expand=True)
        
        # 创建滚动文本框显示传感器数据
        self.sensor_text = scrolledtext.ScrolledText(frame, height=15, width=55)
        current_font = self.sensor_text.cget("font")
        self.sensor_text.config(font=(current_font, 8))
        self.sensor_text.pack(fill=tk.BOTH, expand=True, padx=2, pady=2)
        
        # 初始化传感器数据显示
        self.update_sensor_display(self.get_default_sensor_data())
    
    def _create_bottom_frames(self, parent):
        """创建底部框架（串口配置和ROS启动）"""
        bottom_frame = ttk.Frame(parent)
        bottom_frame.pack(fill=tk.X, pady=(0, 5))
        
        # 串口配置框架 - 压缩高度
        frame3 = ttk.LabelFrame(bottom_frame, text="串口配置", padding=6)
        frame3.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 5))
        
        # ROS启动框架 - 压缩高度
        frame4 = ttk.LabelFrame(bottom_frame, text="ROS启动", padding=6)
        frame4.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=(5, 0))
        
        self._create_bottom_buttons(frame3, frame4)
    
    def _create_bottom_buttons(self, config_frame, ros_frame):
        """创建底部按钮 - 调整大小使两个按钮一致"""
        # 保存串口号按钮 - 使用与ROS按钮相同的高度和字体
        if self.pkg_path:
            self.save_btn = tk.Button(
                config_frame,
                text="保存AI套件传感器系统串口号",
                command=self.save_serial_port,
                font=("Arial", 9),
                height=1,  # 与launch_btn保持一致
                width=30   # 设置相似的宽度
            )
        else:
            self.save_btn = tk.Button(
                config_frame,
                text="保存AI套件传感器系统串口号 (ROS包未找到)",
                state="disabled",
                font=("Arial", 9),
                height=1,
                width=30
            )
        self.save_btn.pack(pady=8, padx=8, fill=tk.X)
        
        # ROS启动按钮 - 调整高度与保存按钮一致
        self.launch_btn = tk.Button(
            ros_frame,
            text="启动AI套件传感器系统launch文件",
            command=self.toggle_ros_launch,
            font=("Arial", 9),
            height=1,  # 与save_btn保持一致
            width=30   # 设置相似的宽度
        )
        self.launch_btn.pack(pady=8, padx=8, fill=tk.X)
        
        # 保存默认按钮颜色
        test_button = tk.Button(self.root)
        self.default_button_bg = test_button.cget("background")
        self.default_button_fg = test_button.cget("foreground")
        test_button.destroy()
    
    def get_default_sensor_data(self):
        """获取默认的传感器数据（用于初始化显示）"""
        return {
            'timestamp': 0,
            'alcohol': 0, 'smoke': 0, 'light': 0,
            'eCO2': 0, 'eCH2O': 0, 'TVOC': 0,
            'PM25': 0, 'PM10': 0,
            'temperature': 0.0, 'humidity': 0.0,
            'accel_x': 0.0, 'accel_y': 0.0, 'accel_z': 0.0,
            'gyro_x': 0.0, 'gyro_y': 0.0, 'gyro_z': 0.0
        }
    
    def refresh_ports(self):
        """刷新串口列表"""
        ports = []
        
        # 获取标准串口
        for port in serial.tools.list_ports.comports():
            port_path = port.device
            if not any(f'ttyS{i}' in port_path for i in range(0, 32)):
                ports.append(port_path)
        
        # 获取其他可能的设备
        other_ports = glob.glob('/dev/ttyUSB*') + glob.glob('/dev/ttyACM*') + glob.glob('/dev/aihitplt*')
        ports.extend([p for p in other_ports if p not in ports])
        
        # 排序端口列表
        ports.sort()
        
        current = self.port_combo.get()
        self.port_combo['values'] = ports
            
        if ports:
            if current and current in ports:
                self.port_combo.set(current)
            else:
                self.port_combo.current(0)
            
        self.update_status(f"找到 {len(ports)} 个串口")
    
    def toggle_serial_connection(self):
        """切换串口连接"""
        if not self.serial_connected:
            self.connect_serial()
        else:
            self.disconnect_serial()
    
    def connect_serial(self):
        """连接串口"""
        port = self.port_combo.get()
        if not port:
            messagebox.showwarning("警告", "请选择串口")
            return
        
        port = self._normalize_port_path(port)
        
        if not os.path.exists(port):
            messagebox.showerror("错误", f"串口 {port} 不存在")
            return
        
        try:
            # 先关闭可能存在的连接
            if self.serial_port:
                try:
                    self.serial_port.close()
                except:
                    pass
            
            # 波特率使用115200
            self.serial_port = serial.Serial(
                port=port,
                baudrate=115200,
                timeout=0.1,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE
            )
            
            self.serial_connected = True
            self.stop_serial_thread = False
            
            # 更新按钮状态
            self._update_button_states("serial_connected")
            
            # 启动读取线程
            self.serial_thread = threading.Thread(
                target=self.read_serial_data,
                daemon=True
            )
            self.serial_thread.start()
            
            self.update_status(f"已连接到串口: {port}")
            
        except Exception as e:
            error_msg = str(e)
            if "Permission denied" in error_msg:
                error_msg = f"权限不足，请运行: sudo chmod 666 {port}"
            
            messagebox.showerror("连接失败", f"无法连接串口:\n{error_msg}")
            print(f"连接失败: {e}")
            self.update_status("连接失败")
    
    def disconnect_serial(self):
        """断开串口连接"""
        if self.serial_connected:
            self.serial_connected = False
            self.stop_serial_thread = True
            
            if self.serial_port:
                try:
                    self.serial_port.close()
                except:
                    pass
                self.serial_port = None
            
            # 等待线程结束
            if self.serial_thread and self.serial_thread.is_alive():
                self.serial_thread.join(timeout=1.0)
            
            # 更新按钮状态
            self._update_button_states("normal")
            
            self.update_status("已断开串口连接")
    
    def read_serial_data(self):
        """读取串口数据并解析传感器帧"""
        while self.serial_connected and not self.stop_serial_thread:
            try:
                if self.serial_port and self.serial_port.in_waiting > 0:
                    header = self.serial_port.read(2)
                    if header == self.FRAME_HEADER:
                        frame_data = self.serial_port.read(self.FRAME_SIZE - 2)
                        if len(frame_data) == self.FRAME_SIZE - 2:
                            full_frame = header + frame_data
                            
                            try:
                                unpacked = struct.unpack(self.FRAME_FORMAT, full_frame)
                                
                                sensor_data = {
                                    'header': unpacked[0],
                                    'timestamp': unpacked[1],
                                    'alcohol': unpacked[2],
                                    'smoke': unpacked[3],
                                    'light': unpacked[4],
                                    'eCO2': unpacked[5],
                                    'eCH2O': unpacked[6],
                                    'TVOC': unpacked[7],
                                    'PM25': unpacked[8],
                                    'PM10': unpacked[9],
                                    'temperature': unpacked[10],
                                    'humidity': unpacked[11],
                                    'accel_x': unpacked[12],
                                    'accel_y': unpacked[13],
                                    'accel_z': unpacked[14],
                                    'gyro_x': unpacked[15],
                                    'gyro_y': unpacked[16],
                                    'gyro_z': unpacked[17],
                                    'checksum': unpacked[18]
                                }
                                
                                # 验证校验和
                                calculated = sum(full_frame[:-1]) & 0xFF
                                if calculated == sensor_data['checksum']:
                                    self.sensor_data_queue.put(sensor_data)
                                    
                            except struct.error as e:
                                print(f"帧解析错误: {e}")
            
            except Exception as e:
                if not self.stop_serial_thread:
                    print(f"串口读取错误: {e}")
                break
            
            time.sleep(0.01)
    
    def init_ros_node(self):
        """初始化ROS节点 - 在主线程中调用"""
        try:
            # 检查ROS Master是否已启动
            import rosgraph
            try:
                rosgraph.Master('/rostopic').getPid()
            except:
                print("等待ROS Master启动...")
                time.sleep(0.5)
                # 再次检查
                try:
                    rosgraph.Master('/rostopic').getPid()
                except:
                    print("ROS Master未启动，等待5秒...")
                    time.sleep(5)
            
            # 初始化ROS节点 - 必须在主线程中调用
            print("开始初始化ROS节点...")
            rospy.init_node('ai_sensor_gui_node', anonymous=True, disable_signals=True)
            
            # 创建话题发布器（用于控制舵机）
            self.control_pub = rospy.Publisher(self.CONTROL_TOPIC, String, queue_size=10)
            
            # 创建话题订阅器（用于接收传感器数据）
            self.sensor_sub = rospy.Subscriber(self.SENSOR_DATA_TOPIC, String, self.sensor_data_callback)
            
            # 等待发布器和订阅器建立连接
            time.sleep(0.5)
            
            self.ros_node_initialized = True
            print("ROS节点初始化成功")
            self.update_status("ROS节点初始化成功，已连接话题")
            
            return True
            
        except Exception as e:
            print(f"ROS节点初始化失败: {e}")
            self.update_status(f"ROS节点初始化失败: {e}")
            return False
    
    def sensor_data_callback(self, msg):
        """ROS传感器数据话题回调函数"""
        try:
            # 解析pan_tilt_sensor.py发布的单行字符串数据
            # 示例: timestamp:"123456" temperature:"25.50" humidity:"60.00" ...
            data_str = msg.data
            
            # 解析传感器数据
            sensor_data = self.parse_sensor_string(data_str)
            
            # 放入队列供显示
            self.sensor_data_queue.put(sensor_data)
            
        except Exception as e:
            print(f"解析ROS传感器数据失败: {e}")
    
    def parse_sensor_string(self, data_str):
        """解析pan_tilt_sensor.py发布的单行字符串"""
        # 初始化默认数据
        sensor_data = self.get_default_sensor_data()
        
        try:
            # 使用正则表达式提取键值对
            # 匹配模式: key:"value"
            pattern = r'(\w+):"([^"]+)"'
            matches = re.findall(pattern, data_str)
            
            for key, value in matches:
                if key == 'timestamp':
                    sensor_data['timestamp'] = int(value) if value.isdigit() else 0
                elif key == 'alcohol':
                    sensor_data['alcohol'] = int(value) if value.isdigit() else 0
                elif key == 'smoke':
                    sensor_data['smoke'] = int(value) if value.isdigit() else 0
                elif key == 'light':
                    sensor_data['light'] = int(value) if value.isdigit() else 0
                elif key == 'eCO2':
                    sensor_data['eCO2'] = int(value) if value.isdigit() else 0
                elif key == 'eCH2O':
                    sensor_data['eCH2O'] = int(value) if value.isdigit() else 0
                elif key == 'TVOC':
                    sensor_data['TVOC'] = int(value) if value.isdigit() else 0
                elif key == 'PM25':
                    sensor_data['PM25'] = int(value) if value.isdigit() else 0
                elif key == 'PM10':
                    sensor_data['PM10'] = int(value) if value.isdigit() else 0
                elif key == 'temperature':
                    sensor_data['temperature'] = float(value) if value.replace('.', '', 1).isdigit() else 0.0
                elif key == 'humidity':
                    sensor_data['humidity'] = float(value) if value.replace('.', '', 1).isdigit() else 0.0
                elif key == 'accel':
                    # 格式: "x,y,z"
                    accel_parts = value.split(',')
                    if len(accel_parts) == 3:
                        sensor_data['accel_x'] = float(accel_parts[0]) if accel_parts[0].replace('.', '', 1).isdigit() else 0.0
                        sensor_data['accel_y'] = float(accel_parts[1]) if accel_parts[1].replace('.', '', 1).isdigit() else 0.0
                        sensor_data['accel_z'] = float(accel_parts[2]) if accel_parts[2].replace('.', '', 1).isdigit() else 0.0
                elif key == 'gyro':
                    # 格式: "x,y,z"
                    gyro_parts = value.split(',')
                    if len(gyro_parts) == 3:
                        sensor_data['gyro_x'] = float(gyro_parts[0]) if gyro_parts[0].replace('.', '', 1).isdigit() else 0.0
                        sensor_data['gyro_y'] = float(gyro_parts[1]) if gyro_parts[1].replace('.', '', 1).isdigit() else 0.0
                        sensor_data['gyro_z'] = float(gyro_parts[2]) if gyro_parts[2].replace('.', '', 1).isdigit() else 0.0
                        
        except Exception as e:
            print(f"解析传感器字符串失败: {e}, 数据: {data_str}")
        
        return sensor_data
    
    def update_sensor_data_from_queue(self):
        """从队列中获取传感器数据并更新显示"""
        while True:
            try:
                sensor_data = self.sensor_data_queue.get(timeout=0.1)
                self.root.after(0, lambda data=sensor_data: self.update_sensor_display(data))
            except queue.Empty:
                continue
            except Exception as e:
                print(f"更新传感器数据错误: {e}")
    
    def _format_sensor_display_text(self, data):
        """格式化传感器数据显示文本"""
        if not isinstance(data, dict):
            return ""
            
        # 使用字典get方法设置默认值
        text_parts = []
        
        # 传感器数据
        text_parts.append("=== 传感器数据 ===")
        text_parts.append(f"时间戳: {data.get('timestamp', 0)}ms  ")
        text_parts.append(f"酒精浓度: {data.get('alcohol', 0)}             烟雾浓度: {data.get('smoke', 0)}")
        text_parts.append(f"光照强度: {data.get('light', 0)}           eCO2: {data.get('eCO2', 0)} ppm")
        text_parts.append(f"eCH2O: {data.get('eCH2O', 0)} μg/m³       TVOC: {data.get('TVOC', 0)} μg/m³")
        text_parts.append(f"PM2.5: {data.get('PM25', 0)} μg/m³       PM10: {data.get('PM10', 0)} μg/m³")
        text_parts.append(f"温度: {data.get('temperature', 0.0):.1f} °C                 湿度: {data.get('humidity', 0.0):.1f}%\n")
    
        # 加速度数据
        text_parts.append("=== 加速度数据 ===")
        text_parts.append(f"加速度X: {data.get('accel_x', 0.0):.2f} m/s²")
        text_parts.append(f"加速度Y: {data.get('accel_y', 0.0):.2f} m/s²")
        text_parts.append(f"加速度Z: {data.get('accel_z', 0.0):.2f} m/s²\n")
        
        # 角速度数据
        text_parts.append("=== 角速度数据 ===")
        text_parts.append(f"角速度X: {data.get('gyro_x', 0.0):.4f} rad/s")
        text_parts.append(f"角速度Y: {data.get('gyro_y', 0.0):.4f} rad/s")
        text_parts.append(f"角速度Z: {data.get('gyro_z', 0.0):.4f} rad/s")
        
        return "\n".join(text_parts)
    
    def update_sensor_display(self, data):
        """更新传感器数据显示"""
        if not hasattr(self, 'sensor_text'):
            return
        
        # 清空文本框
        self.sensor_text.delete(1.0, tk.END)
        
        # 添加格式化后的传感器数据
        display_text = self._format_sensor_display_text(data)
        self.sensor_text.insert(tk.END, display_text)
        
        # 确保文本只读
        self.sensor_text.config(state=tk.NORMAL)
        self.sensor_text.see(tk.END)
    
    def on_scale_change(self, value):
        """滑杆值变化时的回调"""
        self.current_h = self.horizontal_scale.get()
        self.current_v = self.vertical_scale.get()
        self.angle_var.set(f"当前角度: 水平={self.current_h}°, 垂直={self.current_v}°")
    
    def send_servo_angles(self):
        """发送舵机角度命令"""
        horizontal = self.horizontal_scale.get()
        vertical = self.vertical_scale.get()
        
        # 验证角度范围
        if not (0 <= horizontal <= 180 and 0 <= vertical <= 180):
            messagebox.showwarning("警告", "舵机角度必须在0-180之间")
            return
        
        if self.current_mode == "serial":
            # 串口模式
            cmd = f"P:{horizontal},{vertical}\n"
            
            if self.serial_connected and self.serial_port:
                try:
                    self.serial_port.write(cmd.encode())
                    self.update_status(f"发送舵机命令: 水平={horizontal}°, 垂直={vertical}°")
                except Exception as e:
                    messagebox.showerror("发送失败", f"发送舵机命令失败:\n{e}")
                    self.update_status(f"发送失败: {e}")
        
        elif self.current_mode == "ros":
            # ROS话题模式
            if self.ros_node_initialized and self.control_pub:
                try:
                    # 根据pan_control.py的格式发送消息
                    # 支持格式: "90,45" 或 "h:90 v:45"
                    cmd_msg = String()
                    cmd_msg.data = f"{horizontal},{vertical}"
                    
                    self.control_pub.publish(cmd_msg)
                    
                    self.update_status(f"通过话题发送舵机命令: 水平={horizontal}°, 垂直={vertical}°")
                    print(f"发送舵机角度话题: {horizontal},{vertical}")
                    
                except Exception as e:
                    messagebox.showerror("发送失败", f"发送舵机命令失败:\n{e}")
                    self.update_status(f"发送失败: {e}")
            else:
                messagebox.showwarning("警告", "ROS节点未初始化或话题未连接")
    
    def reset_servo_angles(self):
        """复位舵机角度到初始位置 (90,90)"""
        # 设置滑杆为90度
        self.horizontal_scale.set(90)
        self.vertical_scale.set(90)
        
        # 更新显示
        self.current_h = 90
        self.current_v = 90
        self.angle_var.set("当前角度: 水平=90°, 垂直=90°")
        
        if self.current_mode == "serial":
            # 串口模式
            if self.serial_connected and self.serial_port:
                cmd = "P:90,90\n"
                try:
                    self.serial_port.write(cmd.encode())
                    self.update_status("发送舵机复位命令: 水平=90°, 垂直=90°")
                    messagebox.showinfo("复位成功", "舵机已复位到初始位置:\n水平: 90°\n垂直: 90°")
                except Exception as e:
                    messagebox.showerror("复位失败", f"发送复位命令失败:\n{e}")
                    self.update_status(f"复位失败: {e}")
            else:
                messagebox.showinfo("复位", "舵机角度已复位到90°, 但未连接到串口")
        
        elif self.current_mode == "ros":
            # ROS话题模式
            if self.ros_node_initialized and self.control_pub:
                try:
                    cmd_msg = String()
                    cmd_msg.data = "90,90"
                    
                    self.control_pub.publish(cmd_msg)
                    
                    self.update_status("通过话题发送舵机复位命令: 水平=90°, 垂直=90°")
                    messagebox.showinfo("复位成功", "舵机已复位到初始位置:\n水平: 90°\n垂直: 90°")
                    print("发送舵机复位话题: 90,90")
                    
                except Exception as e:
                    messagebox.showerror("复位失败", f"发送复位命令失败:\n{e}")
                    self.update_status(f"复位失败: {e}")
            else:
                messagebox.showinfo("复位", "舵机角度已复位到90°, 但未连接到ROS")
    
    def save_serial_port(self):
        """保存串口号到配置文件"""
        if not self.pkg_path:
            messagebox.showerror("错误", "未找到ROS包路径")
            return
        
        port = self.port_combo.get()
        if not port:
            messagebox.showwarning("警告", "请先选择串口")
            return
        
        port = self._normalize_port_path(port)
        
        try:
            # 创建config目录
            config_dir = os.path.join(self.pkg_path, 'config')
            os.makedirs(config_dir, exist_ok=True)
            
            # 保存配置到YAML文件
            config = {
                'port': port,
                'baudrate': 115200,
                'last_updated': datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            }
            
            with open(self.config_file, 'w') as f:
                yaml.dump(config, f, default_flow_style=False)
            
            # 更新串口选择框
            self.port_combo.set(port)
            
            messagebox.showinfo("保存成功", f"已保存串口: {port}\n配置已保存到: {self.config_file}")
            
            self.update_status(f"已保存串口: {port}")
            
        except Exception as e:
            messagebox.showerror("保存失败", f"保存串口失败:\n{str(e)}")
            print(f"保存失败: {e}")
    
    def load_saved_port(self):
        """加载保存的串口"""
        if not self.pkg_path or not os.path.exists(self.config_file):
            return
        
        try:
            with open(self.config_file, 'r') as f:
                config = yaml.safe_load(f)
            
            if config and 'port' in config:
                saved_port = config['port']
                saved_port = self._normalize_port_path(saved_port)
                
                # 设置串口选择框
                self.port_combo.set(saved_port)
                
                # 刷新串口列表以确保显示正确
                self.refresh_ports()
                
                self.update_status(f"已加载保存的串口: {saved_port}")
                print(f"从配置文件加载串口: {saved_port}")
                
        except Exception as e:
            print(f"加载保存的串口失败: {e}")
    
    def toggle_ros_launch(self):
        """切换ROS launch文件"""
        if not self.ros_running:
            self.start_ros_launch()
        else:
            self.stop_ros_launch()
    
    def start_ros_launch(self):
        """启动ROS launch文件 - 在新终端窗口中"""
        try:
            # 首先断开串口连接（如果已连接）
            if self.serial_connected:
                self.disconnect_serial()
            
            # 获取当前串口
            port = self.port_combo.get()
            if not port:
                messagebox.showwarning("警告", "请先选择串口")
                return
            
            port = self._normalize_port_path(port)
            
            # 构建roslaunch命令
            roslaunch_cmd = f'roslaunch aihitplt_hardware_test aihitplt_multi_ai_sensor.launch'
            
            print(f"ROS启动命令: {roslaunch_cmd}")
            print(f"使用的串口: {port}")
            
            # 使用gnome-terminal打开新窗口
            cmd = [
                'gnome-terminal',
                '--title=AI套件传感器系统 - ROS Launch',
                '--geometry=80x24+100+100',
                '--',
                'bash', '-c',
                f'{roslaunch_cmd}'
            ]
            
            print(f"终端命令: {' '.join(cmd)}")
            
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
            
            # 立即更新按钮状态（在主线程中）
            self.root.after(0, lambda: self._update_button_states("ros_running"))
            
            
            # 等待一会儿让ROS启动
            time.sleep(0.5)
            
            # 在主线程中初始化ROS节点
            self.root.after(0, self.init_ros_node)
            
        except FileNotFoundError:
            messagebox.showerror("启动失败", "未找到gnome-terminal。")
            self.update_status("启动失败: 未找到gnome-terminal")
            
        except Exception as e:
            messagebox.showerror("启动失败", f"无法启动ROS launch文件:\n{e}")
            print(f"启动失败: {e}")
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
            
            # 给子进程一点时间结束
            gone, alive = psutil.wait_procs(children, timeout=3)
            
            # 强制终止仍然存活的子进程
            for child in alive:
                try:
                    child.kill()
                except:
                    pass
            
            # 终止父进程
            try:
                process.terminate()
            except:
                pass
            
            # 等待父进程结束
            try:
                process.wait(timeout=3)
            except:
                try:
                    process.kill()
                except:
                    pass
                    
        except psutil.NoSuchProcess:
            # 进程已经不存在了
            pass
        except Exception as e:
            print(f"终止进程树时出错: {e}")
    
    def stop_ros_launch(self):
        """停止ROS launch文件"""
        if self.ros_running:
            try:
                # 关闭ROS节点
                self.ros_node_initialized = False
                
                # 清理ROS订阅器和发布器
                if self.sensor_sub:
                    self.sensor_sub.unregister()
                if self.control_pub:
                    self.control_pub.unregister()
                
                # 首先尝试终止gnome-terminal进程
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
                
                # 恢复按钮状态（在主线程中）
                self.root.after(0, lambda: self._update_button_states("normal"))
                
                self.update_status("ROS进程已停止")
                
                # 检查并清理残留的ROS进程
                self._cleanup_ros_processes()
    
    def _kill_ros_processes(self):
        """终止所有与ROS相关的进程"""
        try:
            processes_to_kill = ['aihitplt_multi_ai_sensor.launch']
            for proc_name in processes_to_kill:
                subprocess.run(['pkill', '-f', proc_name], 
                             stdout=subprocess.DEVNULL, 
                             stderr=subprocess.DEVNULL)
            
            time.sleep(1)
            
        except Exception as e:
            print(f"终止ROS进程时出错: {e}")
    
    def _cleanup_ros_processes(self):
        """清理残留的ROS进程"""
        try:
            processes_to_clean = [
                'multi_ai_sensor_node.py',
                'multi_ai_control_node.py',
            ]
            
            for proc_name in processes_to_clean:
                subprocess.run(['pkill', '-9', '-f', proc_name], 
                             stdout=subprocess.DEVNULL, 
                             stderr=subprocess.DEVNULL)
            
        except Exception as e:
            print(f"清理ROS进程时出错: {e}")
    
    def update_status(self, message):
        """更新状态栏"""
        timestamp = datetime.now().strftime("%H:%M:%S")
        self.status_var.set(f"[{timestamp}] {message}")
        print(f"[状态] {message}")
    
    def on_closing(self):
        """窗口关闭时的处理"""
        # 停止串口连接
        if self.serial_connected:
            self.disconnect_serial()
        
        # 停止ROS进程
        if self.ros_running:
            self.stop_ros_launch()
        
        # 等待一小段时间确保资源清理完成
        time.sleep(0.5)
        
        # 关闭窗口
        self.root.destroy()

def main():
    """主函数"""
    # 设置环境
    if 'DISPLAY' not in os.environ:
        os.environ['DISPLAY'] = ':0'
    os.environ['QT_X11_NO_MITSHM'] = '1'

    root = tk.Tk()
    
    # 设置窗口图标和标题
    root.title("AI套件传感器系统测试")
    
    # 窗口居中
    window_width = 800
    window_height = 750
    screen_width = root.winfo_screenwidth()
    screen_height = root.winfo_screenheight()
    x = (screen_width - window_width) // 2
    y = (screen_height - window_height) // 2
    root.geometry(f"{window_width}x{window_height}+{x}+{y}")
    
    # 创建应用
    app = AISensorSystemTester(root)
    
    # 运行主循环
    root.mainloop()

if __name__ == "__main__":
    main()