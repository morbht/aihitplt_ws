#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
磁导航传感器测试程序
支持磁导航传感器设备连接和ROS launch文件管理
"""

import tkinter as tk
from tkinter import ttk, messagebox
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
import signal
import re
import math
import psutil
import rospy
from std_msgs.msg import Int16MultiArray

class MagNavSensorTester:
    def __init__(self, root):
        self.root = root
        self.root.title("磁导航传感器测试")
        self.root.geometry("450x500")
        
        # 串口相关
        self.serial_port = None
        self.serial_connected = False
        self.serial_thread = None
        self.stop_serial_thread = False
        
        # ROS相关
        self.ros_process = None
        self.ros_running = False
        self.ros_pid = None
        self.ros_node_initialized = False
        
        # ROS话题相关
        self.mag_sub = None
        self.MAG_TOPIC = "/mag_sensor"
        
        # 磁导航传感器数据
        self.sensor_data = {
            'io': 0,  # 8位传感器状态
            'offset_left': 0,
            'offset_straight': 0,
            'offset_right': 0,
            'sensor_bits': [0, 0, 0, 0, 0, 0, 0, 0],  # 8个传感器位
            'last_update': 0
        }
        
        # 显示控制
        self.last_display_time = 0
        self.DISPLAY_INTERVAL = 0.05  # 20Hz显示频率
        
        # 获取ROS包路径
        self.pkg_path = None
        self.config_file = None
        self._init_ros_path()
        
        # 创建界面
        self.create_widgets()
        
        # 加载保存的串口
        self.load_saved_port()
        
        # 绑定关闭事件
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        
        # 初始更新显示
        self.update_sensor_display()
    
    def _init_ros_path(self):
        """初始化ROS包路径"""
        try:
            rospack = rospkg.RosPack()
            self.pkg_path = rospack.get_path('aihitplt_hardware_test')
            self.config_file = os.path.join(self.pkg_path, 'config', 'magnav_sensor_port.yaml')
            print(f"找到ROS包路径: {self.pkg_path}")
        except Exception as e:
            print(f"ROS包加载警告: {e}")
    
    def create_widgets(self):
        """创建界面 - 分成4个部分"""
        
        # 第一部分：设备连接
        frame1 = ttk.LabelFrame(self.root, text="设备连接", padding=10)
        frame1.pack(fill="x", padx=10, pady=(10, 5))
        
        # 串口选择部分 - 单行布局
        port_select_frame = ttk.Frame(frame1)
        port_select_frame.pack(fill=tk.X, pady=5)
        
        ttk.Label(port_select_frame, text="设备:").pack(side=tk.LEFT, padx=(0, 5))
        
        # 缩短设备选择栏
        self.port_combo = ttk.Combobox(port_select_frame, width=25, state="readonly")
        self.port_combo.pack(side=tk.LEFT, padx=(0, 15))
        
        # 刷新按钮 - 与设备选择在同一行
        self.refresh_btn = ttk.Button(
            port_select_frame, 
            text="刷新", 
            command=self.refresh_ports,
            width=8
        )
        self.refresh_btn.pack(side=tk.LEFT, padx=(0, 15))
        
        # 连接按钮 - 与设备选择在同一行
        self.connect_btn = ttk.Button(
            port_select_frame, 
            text="连接",
            command=self.toggle_serial_connection,
            width=8
        )
        self.connect_btn.pack(side=tk.LEFT)
        
        # 第二部分：磁导航传感器状态显示
        frame2 = ttk.LabelFrame(self.root, text="磁导航传感器状态", padding=10)
        frame2.pack(fill="both", expand=True, padx=10, pady=5)
        
        # 8个圆形指示灯（从左到右排列）
        self.sensor_lights_frame = tk.Frame(frame2)
        self.sensor_lights_frame.pack(pady=10)
        
        self.sensor_lights = []
        for i in range(8):  # 8个灯，对应S1到S8
            # 创建传感器编号标签
            label = tk.Label(self.sensor_lights_frame, text=f"S{i+1}")
            label.grid(row=0, column=i, padx=5, pady=(0, 5))
            
            # 创建传感器状态灯
            light = tk.Canvas(self.sensor_lights_frame, width=40, height=40, 
                             bg="white", highlightthickness=0)
            light.grid(row=1, column=i, padx=5)
            # 绘制圆形（初始为灰色）
            light.create_oval(5, 5, 35, 35, fill="gray", outline="black", width=2)
            self.sensor_lights.append(light)
        
        # 偏移信息显示
        self.offset_label = tk.Label(
            frame2, 
            text="偏移信息:左:0(0mm) 中:0(0mm) 右:0(0mm)",
            font=("Arial", 12),
            bg="white",
            padx=10,
            pady=5,
            relief=tk.SUNKEN,
            width=45
        )
        self.offset_label.pack(pady=10)
        
        # 连接状态显示
        self.connection_status = tk.Label(
            frame2,
            text="状态: 未连接",
            font=("Arial", 10),
            fg="gray"
        )
        self.connection_status.pack(pady=5)
        
        # 第三部分：保存配置
        frame3 = ttk.LabelFrame(self.root, text="设备配置", padding=10)
        frame3.pack(fill="x", padx=10, pady=5)
        
        self.save_btn = ttk.Button(
            frame3,
            text="保存设备号",
            command=self.save_serial_port,
            width=28  # 与安防传感器程序保持一致
        )
        self.save_btn.pack()
        
        # 第四部分：ROS启动
        frame4 = ttk.LabelFrame(self.root, text="ROS启动", padding=10)
        frame4.pack(fill="x", padx=10, pady=(5, 10))
        
        # 使用普通Button而不是ttk.Button以便设置背景色
        self.launch_btn = tk.Button(
            frame4,
            text="启动launch文件",
            command=self.toggle_ros_launch,
            width=26,  # 与安防传感器程序保持一致
            bg="lightgray",
            fg="black"
        )
        self.launch_btn.pack()
        
        # 状态栏
        self.status_var = tk.StringVar(value="就绪")
        status_bar = ttk.Label(self.root, textvariable=self.status_var, 
                              relief=tk.SUNKEN, anchor=tk.W)
        status_bar.pack(side=tk.BOTTOM, fill=tk.X, pady=(10, 0))
        
        # 初始化设备列表
        self.refresh_ports()
        
        # 初始状态
        self.update_button_states()
    
    def update_button_states(self):
        """更新按钮状态"""
        if self.serial_connected:
            # 串口连接时，禁用保存和启动按钮
            self.save_btn.config(state="disabled")
            self.launch_btn.config(state="disabled")
        elif self.ros_running:
            # ROS运行时，禁用连接按钮
            self.connect_btn.config(state="disabled")
            self.refresh_btn.config(state="disabled")
            self.save_btn.config(state="disabled")
        else:
            # 正常状态
            self.save_btn.config(state="normal")
            self.launch_btn.config(state="normal")
            self.connect_btn.config(state="normal")
            self.refresh_btn.config(state="normal")
    
    def refresh_ports(self):
        """刷新设备列表"""
        # 保存当前选择的值
        current_selection = self.port_combo.get()
        
        devices = []
        
        # 1. 获取aihitplt_mag设备（优先显示）
        aihitplt_devices = glob.glob('/dev/aihitplt_*')
        for device in aihitplt_devices:
            if 'mag' in device.lower():
                devices.insert(0, device)
            else:
                devices.append(device)
        
        # 2. 获取标准串口（过滤ttyS0-ttyS31）
        for p in serial.tools.list_ports.comports():
            port_path = p.device
            # 过滤掉ttyS0-ttyS31这些没用的串口
            if not any(f'ttyS{i}' in port_path for i in range(0, 32)):
                if port_path not in devices:
                    devices.append(port_path)
        
        # 3. 获取其他可能的设备
        other_devices = glob.glob('/dev/ttyUSB*') + glob.glob('/dev/ttyACM*')
        for device in other_devices:
            if device not in devices:
                devices.append(device)
        
        # 排序：aihitplt_mag优先，然后其他
        devices.sort(key=lambda x: (0 if 'aihitplt_mag' in x else 1, x))
        
        # 更新下拉框的值
        self.port_combo['values'] = devices
        
        # 如果之前有选择的值且该值仍然存在，保持选择
        if current_selection and current_selection in devices:
            self.port_combo.set(current_selection)
        elif devices:
            # 否则选择第一个
            self.port_combo.current(0)
        
        self.update_status(f"找到 {len(devices)} 个设备")
    
    def toggle_serial_connection(self):
        """切换设备连接"""
        if not self.serial_connected:
            self.connect_serial()
        else:
            self.disconnect_serial()
    
    def connect_serial(self):
        """连接磁导航传感器"""
        device = self.port_combo.get()
        if not device:
            messagebox.showwarning("警告", "请选择设备")
            return
        
        # 确保设备路径格式正确
        if not device.startswith('/dev/'):
            device = f'/dev/{device}'
        
        # 检查设备是否存在
        if not os.path.exists(device):
            messagebox.showerror("错误", f"设备 {device} 不存在")
            return
        
        try:
            # 先关闭可能存在的连接
            if self.serial_port:
                try:
                    self.serial_port.close()
                except:
                    pass
            
            print(f"正在连接设备: {device}")
            
            # 对于aihitplt_mag，可能需要额外的初始化
            # 先尝试标准连接方式
            self.serial_port = serial.Serial(
                port=device,
                baudrate=115200,
                timeout=0.1,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE
            )
            
            # 尝试发送初始化命令（如果需要）
            time.sleep(0.1)
            self.serial_port.flushInput()
            self.serial_port.flushOutput()
            
            self.serial_connected = True
            self.stop_serial_thread = False
            
            # 更新按钮状态
            self.connect_btn.config(text="关闭")
            self.refresh_btn.config(state="disabled")
            self.save_btn.config(state="disabled")
            self.launch_btn.config(state="disabled")
            
            # 更新连接状态
            self.connection_status.config(text=f"状态: 已连接 {device}", fg="green")
            
            # 重置传感器数据
            self.reset_sensor_data()
            
            # 启动读取线程
            self.serial_thread = threading.Thread(
                target=self.read_serial_data,
                daemon=True
            )
            self.serial_thread.start()
            
            self.update_status(f"已连接到磁导航传感器 {device}")
            
        except Exception as e:
            error_msg = str(e)
            if "Permission denied" in error_msg:
                error_msg = f"权限不足，请运行: sudo chmod 666 {device}"
            
            messagebox.showerror("连接失败", f"无法连接磁导航传感器:\n{error_msg}")
            print(f"连接失败: {e}")
            self.update_status("连接失败")
    
    def disconnect_serial(self):
        """断开设备连接"""
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
            self.connect_btn.config(text="连接")
            self.refresh_btn.config(state="normal")
            self.save_btn.config(state="normal")
            self.launch_btn.config(state="normal")
            
            # 更新连接状态
            self.connection_status.config(text="状态: 未连接", fg="gray")
            
            # 重置传感器数据
            self.reset_sensor_data()
            
            self.update_status("已断开磁导航传感器连接")
    
    def calculate_checksum(self, data):
        """计算校验和（与C++程序一致）"""
        sum_val = 0
        for byte in data:
            sum_val += byte
        return sum_val & 0xFF
    
    def parse_igk_g408_data(self, buffer):
        """解析IGK-G408数据帧"""
        try:
            buffer_size = len(buffer)
            
            # 查找帧头 0xDD
            for i in range(buffer_size - 22):  # 至少需要23字节
                if buffer[i] == 0xDD:
                    # 检查是否有完整的23字节帧
                    if i + 23 <= buffer_size:
                        frame = buffer[i:i+23]
                        
                        # 校验和检查
                        calculated_checksum = self.calculate_checksum(frame[:22])
                        received_checksum = frame[22]
                        
                        if calculated_checksum != received_checksum:
                            continue
                        
                        # 解析开关量 (2字节) - 大端模式
                        switch_value = (frame[1] << 8) | frame[2]
                        
                        # 解析偏移量 (有符号字节)
                        left_offset = struct.unpack('b', bytes([frame[3]]))[0]
                        straight_offset = struct.unpack('b', bytes([frame[4]]))[0]
                        right_offset = struct.unpack('b', bytes([frame[5]]))[0]
                        
                        # 更新传感器数据
                        self.sensor_data['io'] = switch_value
                        self.sensor_data['offset_left'] = left_offset
                        self.sensor_data['offset_straight'] = straight_offset
                        self.sensor_data['offset_right'] = right_offset
                        
                        # 解析8个传感器位（S1对应bit0）
                        for i in range(8):
                            self.sensor_data['sensor_bits'][i] = (switch_value >> i) & 0x01
                        
                        self.sensor_data['last_update'] = time.time()
                        
                        return True, i + 23  # 返回成功和已处理的位置
            
            return False, 0
            
        except Exception as e:
            print(f"解析数据帧错误: {e}")
            return False, 0
    
    def read_serial_data(self):
        """读取磁导航传感器数据"""
        cumulative_buffer = bytearray()
        last_data_time = time.time()
        data_received = False
        
        print(f"开始读取磁导航传感器数据...")
        
        while self.serial_connected and not self.stop_serial_thread:
            try:
                if self.serial_port and self.serial_port.in_waiting > 0:
                    # 读取所有可用数据
                    available = self.serial_port.in_waiting
                    data = self.serial_port.read(available)
                    
                    if len(data) > 0:
                        last_data_time = time.time()
                        data_received = True
                    
                    cumulative_buffer.extend(data)
                    
                    # 限制缓冲区大小
                    if len(cumulative_buffer) > 500:
                        cumulative_buffer = cumulative_buffer[-200:]
                    
                    # 尝试解析数据
                    while len(cumulative_buffer) >= 23:
                        parsed, processed_len = self.parse_igk_g408_data(cumulative_buffer)
                        if parsed and processed_len > 0:
                            # 成功解析后更新显示
                            self.root.after(0, self.update_sensor_display)
                            
                            # 移除已处理的数据
                            cumulative_buffer = cumulative_buffer[processed_len:]
                        else:
                            # 没有找到有效帧，尝试移除一些数据
                            if len(cumulative_buffer) > 100:
                                # 查找下一个0xDD
                                next_dd = -1
                                for i in range(1, len(cumulative_buffer)):
                                    if cumulative_buffer[i] == 0xDD:
                                        next_dd = i
                                        break
                                
                                if next_dd > 0:
                                    cumulative_buffer = cumulative_buffer[next_dd:]
                                else:
                                    cumulative_buffer = cumulative_buffer[-46:]  # 保留最后46字节
                            break
                
                # 检查是否长时间没有收到数据
                current_time = time.time()
                if data_received and (current_time - last_data_time) > 2.0:
                    print("长时间未收到数据，重置传感器状态")
                    self.reset_sensor_data()
                    data_received = False
                
                time.sleep(0.01)
                
            except Exception as e:
                if not self.stop_serial_thread:
                    print(f"磁导航传感器读取错误: {e}")
                break
        
        print("磁导航传感器读取线程结束")
    
    def reset_sensor_data(self):
        """重置传感器数据为默认值"""
        self.sensor_data = {
            'io': 0,
            'offset_left': 0,
            'offset_straight': 0,
            'offset_right': 0,
            'sensor_bits': [0, 0, 0, 0, 0, 0, 0, 0],
            'last_update': 0
        }
        self.root.after(0, self.update_sensor_display)
    
    def update_sensor_display(self):
        """更新传感器状态显示 - 添加频率控制"""
        current_time = time.time()
        
        # 控制显示频率
        if current_time - self.last_display_time < self.DISPLAY_INTERVAL:
            return
        
        self.last_display_time = current_time
        
        # 更新8个传感器指示灯
        for i in range(8):
            # 确保有传感器位数据
            if i < len(self.sensor_data['sensor_bits']):
                detected = self.sensor_data['sensor_bits'][i]
            else:
                detected = 0
                
            color = "green" if detected else "gray"
            
            canvas = self.sensor_lights[i]
            # 清除之前的圆形
            canvas.delete("all")
            # 绘制新圆形
            canvas.create_oval(5, 5, 35, 35, fill=color, outline="black", width=2)
        
        # 更新偏移信息
        offset_text = f"偏移信息:左:{self.sensor_data['offset_left']}({self.sensor_data['offset_left'] * 5}mm) "
        offset_text += f"中:{self.sensor_data['offset_straight']}({self.sensor_data['offset_straight'] * 5}mm) "
        offset_text += f"右:{self.sensor_data['offset_right']}({self.sensor_data['offset_right'] * 5}mm)"
        self.offset_label.config(text=offset_text)
    
    def save_serial_port(self):
        """保存磁导航传感器串口号"""
        if not self.pkg_path:
            messagebox.showerror("错误", "未找到ROS包路径")
            return
        
        device = self.port_combo.get()
        if not device:
            messagebox.showwarning("警告", "请先选择设备")
            return
        
        # 确保设备路径格式正确
        if not device.startswith('/dev/'):
            device = f'/dev/{device}'
        
        try:
            # 创建config目录
            config_dir = os.path.join(self.pkg_path, 'config')
            os.makedirs(config_dir, exist_ok=True)
            
            # 保存配置到YAML文件
            config = {
                'port': device,
                'baudrate': 115200,
                'sensor_type': 'IGK-G408',
                'last_updated': datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            }
            
            with open(self.config_file, 'w') as f:
                yaml.dump(config, f, default_flow_style=False)
            
            # 更新串口选择框
            self.port_combo.set(device)
            
            # 弹出保存成功弹窗
            messagebox.showinfo("保存成功", 
                                f"设备 {device} 已保存成功\n" 
                                f"配置文件: {self.config_file}")
            
            self.update_status(f"已保存设备: {device}")
            
        except Exception as e:
            messagebox.showerror("保存失败", f"保存设备失败:\n{str(e)}")
    
    def load_saved_port(self):
        """加载保存的串口"""
        if not self.pkg_path or not os.path.exists(self.config_file):
            return
        
        try:
            with open(self.config_file, 'r') as f:
                config = yaml.safe_load(f)
            
            if config and 'port' in config:
                saved_port = config['port']
                
                # 修复可能存在的路径问题
                if saved_port.startswith('//dev/'):
                    saved_port = saved_port.replace('//dev/', '/dev/')
                elif not saved_port.startswith('/dev/'):
                    saved_port = f'/dev/{saved_port}'
                
                # 设置设备选择框
                self.port_combo.set(saved_port)
                
                self.update_status(f"已加载保存的设备: {saved_port}")
                print(f"从配置文件加载设备: {saved_port}")
                
        except Exception as e:
            print(f"加载保存的设备失败: {e}")
    
    def init_ros_node(self):
        """初始化ROS节点"""
        try:
            if not rospy.is_shutdown():
                # 初始化节点
                rospy.init_node('magnav_sensor_tester_gui', anonymous=True, disable_signals=True)
                
                # 订阅磁导航传感器话题
                self.mag_sub = rospy.Subscriber(
                    self.MAG_TOPIC,
                    Int16MultiArray,
                    self.ros_mag_callback
                )
                
                self.ros_node_initialized = True
                print("ROS节点初始化成功")
                self.update_status("ROS节点初始化成功，已订阅磁导航传感器话题")
                
                return True
                
        except Exception as e:
            print(f"ROS节点初始化失败: {e}")
            self.update_status(f"ROS节点初始化失败: {e}")
            return False
    
    def ros_mag_callback(self, msg):
        """ROS磁导航传感器话题回调函数"""
        try:
            if len(msg.data) >= 4:
                # 解析ROS话题数据
                # 假设数据格式为：[io, offset_left, offset_straight, offset_right]
                self.sensor_data['io'] = msg.data[0]
                self.sensor_data['offset_left'] = msg.data[1]
                self.sensor_data['offset_straight'] = msg.data[2]
                self.sensor_data['offset_right'] = msg.data[3]
                
                # 解析8个传感器位
                for i in range(8):
                    self.sensor_data['sensor_bits'][i] = (self.sensor_data['io'] >> i) & 0x01
                
                self.sensor_data['last_update'] = time.time()
                
                # 更新显示
                self.root.after(0, self.update_sensor_display)
                    
        except Exception as e:
            print(f"ROS话题数据处理失败: {e}")
    
    def toggle_ros_launch(self):
        """切换ROS launch文件"""
        if not self.ros_running:
            self.start_ros_launch()
        else:
            self.stop_ros_launch()
    
    def start_ros_launch(self):
        """启动ROS launch文件 - 使用gnome-terminal打开新终端"""
        try:
            # 获取当前设备
            port = self.port_combo.get()
            if not port:
                messagebox.showwarning("警告", "请先选择设备")
                return
            
            # 构建roslaunch命令
            roslaunch_cmd = f'roslaunch aihitplt_hardware_test aihitplt_mag_follower.launch'
            
            print(f"启动命令: {roslaunch_cmd}")
            print(f"使用设备: {port}")
            
            # 使用gnome-terminal打开新窗口
            cmd = [
                'gnome-terminal',
                '--title=磁导航传感器 - ROS Launch',
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
            self.refresh_btn.config(state="disabled")
            self.save_btn.config(state="disabled")
            
            self.update_status(f"已启动ROS launch文件")
            
            # 重置传感器数据
            self.reset_sensor_data()
            self.connection_status.config(text="ROS已启动，等待磁导航传感器话题...")
            
            # 在新线程中初始化ROS节点
            ros_init_thread = threading.Thread(
                target=self.init_ros_node,
                daemon=True
            )
            ros_init_thread.start()
            
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
            
            # 等待子进程结束
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
                # 清理ROS节点
                if self.ros_node_initialized:
                    # 取消订阅
                    if self.mag_sub:
                        self.mag_sub.unregister()
                    
                    # 关闭节点
                    try:
                        rospy.signal_shutdown("GUI关闭")
                    except:
                        pass
                    
                    self.ros_node_initialized = False
                
                # 终止进程
                if self.ros_pid:
                    self.kill_process_tree(self.ros_pid)
                
                # 终止特定的launch文件进程
                self._kill_ros_processes()
                
            except Exception as e:
                print(f"停止ROS进程时出错: {e}")
                
            finally:
                # 更新状态
                self.ros_running = False
                self.ros_process = None
                self.ros_pid = None
                
                # 恢复按钮状态
                self.launch_btn.config(
                    text="启动launch文件",
                    bg="lightgray",
                    fg="black"
                )
                self.connect_btn.config(state="normal")
                self.refresh_btn.config(state="normal")
                self.save_btn.config(state="normal")
                
                # 重置传感器数据
                self.reset_sensor_data()
                self.connection_status.config(text="ROS已停止")
                
                self.update_status("ROS进程已停止")
    
    def _kill_ros_processes(self):
        """终止所有与ROS相关的进程"""
        try:
            # 终止特定的launch文件
            launch_files_to_kill = [
                'aihitplt_mag_follower.launch'
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
    
    def cleanup_resources(self):
        """清理所有资源"""
        print("正在清理资源...")
        
        # 停止串口连接
        if self.serial_connected:
            self.disconnect_serial()
        
        # 停止ROS进程
        if self.ros_running:
            self.stop_ros_launch()
        
        print("资源清理完成")
    
    def on_closing(self):
        """窗口关闭时的处理"""
        self.cleanup_resources()
        
        # 等待一小段时间确保资源清理完成
        time.sleep(0.5)
        
        # 关闭窗口
        self.root.destroy()

def signal_handler(signum, frame):
    """处理Ctrl+C信号"""
    import sys
    sys.exit(0)

def main():
    """主函数"""
    # 设置信号处理器，处理Ctrl+C
    signal.signal(signal.SIGINT, signal_handler)
    
    # 检查DISPLAY环境变量
    if 'DISPLAY' not in os.environ:
        os.environ['DISPLAY'] = ':0'
    
    # 设置环境变量，避免X Window错误
    os.environ['QT_X11_NO_MITSHM'] = '1'

    root = tk.Tk()
    
    # 设置窗口居中
    window_width = 650
    window_height = 500
    screen_width = root.winfo_screenwidth()
    screen_height = root.winfo_screenheight()
    x = (screen_width - window_width) // 2
    y = (screen_height - window_height) // 2
    root.geometry(f"{window_width}x{window_height}+{x}+{y}")
    
    # 创建应用
    app = MagNavSensorTester(root)
    
    try:
        root.mainloop()
    except KeyboardInterrupt:
        print("\n收到键盘中断，正在清理资源...")
        app.cleanup_resources()
        root.destroy()
    except Exception as e:
        print(f"程序运行错误: {e}")
        app.cleanup_resources()
        root.destroy()

if __name__ == "__main__":
    main()