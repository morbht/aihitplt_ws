#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
送物模块硬件系统测试程序 - Ubuntu适配版（集成ROS Launch功能）
"""

import tkinter as tk
from tkinter import ttk, scrolledtext, messagebox
import serial
import serial.tools.list_ports
import threading
import time
import os
import yaml
import subprocess
import glob
from datetime import datetime

try:
    import rospy
    from std_msgs.msg import String, Bool, Float32, Int32
    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False
    print("警告: 未安装ROS Python库，ROS话题功能将不可用")

class DeliveryModuleTestUI_Ubuntu:
    def __init__(self, root):
        self.root = root
        self.root.title("送物模块硬件系统测试")
        self.root.geometry("870x450")
        
        # 串口相关变量
        self.ser = None
        self.is_connected = False
        self.serial_thread = None
        self.running = False
        
        # ROS相关变量
        self.ros_process = None
        self.ros_running = False
        self.ros_node_initialized = False
        
        # ROS话题订阅器和发布器
        self.state_sub = None
        self.upper_motor_sub = None
        self.upper_up_limit_sub = None
        self.upper_down_limit_sub = None
        self.lower_motor_sub = None
        self.lower_up_limit_sub = None
        self.lower_down_limit_sub = None
        self.sys_state_sub = None
        self.emergency_sub = None
        
        self.upper_motor_pub = None
        self.upper_reset_pub = None
        self.upper_control_pub = None
        self.lower_motor_pub = None
        self.lower_reset_pub = None
        self.lower_control_pub = None
        self.init_pub = None
        self.reset_pub = None
        
        # 配置路径
        self.config_dir = os.path.expanduser("/home/aihit/aihitplt_ws/src/aihitplt_hardware_test/config")
        self.config_file = os.path.join(self.config_dir, "delivery_module_config.yaml")
        
        # 确保config目录存在
        if not os.path.exists(self.config_dir):
            try:
                os.makedirs(self.config_dir, exist_ok=True)
                print(f"已创建config目录: {self.config_dir}")
            except Exception as e:
                print(f"创建config目录失败: {e}")
        
        # 状态变量（与下位机对应）
        self.sys_state = 0
        self.upper_motor_enabled = False
        self.lower_motor_enabled = False
        self.c1u_complete = False
        self.c1d_complete = False
        self.c2u_complete = False
        self.c2d_complete = False
        self.emergency_stop = False
        self.c1u_limit = False
        self.c1d_limit = False
        self.c2u_limit = False
        self.c2d_limit = False
        
        # 默认配置
        self.default_config = {
            'serial_port': '/dev/ttyUSB0',
            'baudrate': 115200,
            'upper_init_distance': 220.0,
            'lower_init_distance': 220.0,
            'upper_cal_distance': 220.0,
            'lower_cal_distance': 220.0,
            'upper_move_distance': 10.0,
            'lower_move_distance': 10.0
        }
        
        # 保存按钮颜色
        temp_btn = tk.Button(self.root)
        self.default_bg = temp_btn.cget("bg")
        self.default_fg = temp_btn.cget("fg")
        temp_btn.destroy()
        
        # 初始化UI布局
        self.create_widgets()
        
        # 启动串口扫描线程
        self.scan_serial_ports()
        self.root.after(2000, self.scan_serial_ports)  # 每2秒刷新串口列表
        
        # 加载配置
        self.load_config()
        
        # 绑定关闭事件
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)

    def create_widgets(self):
        # 整体布局：左（系统连接/初始化/状态/日志） + 右（上下舱门控制/配置/ROS）
        main_frame = ttk.Frame(self.root, padding=2)
        main_frame.pack(fill=tk.BOTH, expand=True)
        
        # 左侧区域（系统控制）
        left_frame = ttk.LabelFrame(main_frame, text="系统控制", padding=10)
        left_frame.grid(row=0, column=0, sticky=tk.NSEW, padx=2, pady=5)
        
        # 系统连接子区域
        conn_frame = ttk.LabelFrame(left_frame, text="串口连接", padding=5)
        conn_frame.pack(fill=tk.X, pady=0)
        
        ttk.Label(conn_frame, text="串口:").grid(row=0, column=0, sticky=tk.W)
        self.port_combo = ttk.Combobox(conn_frame, state="readonly", width=20)
        self.port_combo.grid(row=0, column=1, padx=2)
        
        self.refresh_btn = ttk.Button(conn_frame, text="刷新", command=self.scan_serial_ports)
        self.refresh_btn.grid(row=0, column=2, padx=5)
        
        self.connect_btn = ttk.Button(conn_frame, text="连接", command=self.toggle_serial)
        self.connect_btn.grid(row=0, column=3, padx=5)
        
        # 保存串口按钮
        self.save_btn = ttk.Button(conn_frame, text="保存串口配置", command=self.save_serial_config)
        self.save_btn.grid(row=0, column=4, padx=5)
        
        # 初始化区域
        init_frame = ttk.LabelFrame(left_frame, text="系统初始化", padding=5)
        init_frame.pack(fill=tk.X, pady=5)
        
        # 创建Frame来包裹初始化距离标签和输入框
        init_input_frame = ttk.Frame(init_frame)
        init_input_frame.pack(fill=tk.X, pady=2)
        
        ttk.Label(init_input_frame, text="上门初始化距离(mm):").grid(row=0, column=0, sticky=tk.W, padx=2)
        self.init_upper_entry = ttk.Entry(init_input_frame, width=10)
        self.init_upper_entry.insert(0, "220.0")
        self.init_upper_entry.grid(row=0, column=1, padx=5)
        
        ttk.Label(init_input_frame, text="下门初始化距离(mm):").grid(row=0, column=2, sticky=tk.W, padx=2)
        self.init_lower_entry = ttk.Entry(init_input_frame, width=10)
        self.init_lower_entry.insert(0, "220.0")
        self.init_lower_entry.grid(row=0, column=3, padx=5)
        
        # 系统初始化按钮 - 使用pack并fill=X使其占满宽度
        self.init_btn = ttk.Button(init_frame, text="执行系统初始化", command=self.send_init, state="disabled")
        self.init_btn.pack(fill=tk.X, pady=5)
        
        # 状态显示区域
        status_frame = ttk.LabelFrame(left_frame, text="设备状态", padding=5)
        status_frame.pack(fill=tk.X, pady=5)

        status_frame.grid_columnconfigure(0, weight=1)
        status_frame.grid_columnconfigure(1, weight=0)

        ttk.Label(status_frame, text="当前状态:").grid(row=0, column=0, sticky=tk.W, padx=(0, 10))
        self.status_label = ttk.Label(status_frame, text="未连接", foreground="red")
        self.status_label.grid(row=0, column=0, sticky=tk.W, padx=(80, 0))

        self.reset_btn = ttk.Button(status_frame, text="电机重置", command=self.send_reset, state="disabled")
        self.reset_btn.grid(row=0, column=1, padx=(0, 70), sticky=tk.E)
        
        # 系统日志区域
        log_frame = ttk.LabelFrame(left_frame, text="系统日志", padding=5)
        log_frame.pack(fill=tk.BOTH, expand=True, pady=2)
        
        self.log_text = scrolledtext.ScrolledText(log_frame, height=12, width=50, wrap=tk.WORD)
        self.log_text.pack(fill=tk.BOTH, expand=True, padx=2, pady=2)
        
        # 右侧区域（舱门控制和ROS）
        right_frame = ttk.Frame(main_frame)
        right_frame.grid(row=0, column=1, sticky=tk.NSEW, padx=1, pady=0)
        right_frame.grid_rowconfigure(0, weight=1)
        right_frame.grid_rowconfigure(1, weight=0)
        
        # 上舱门控制
        upper_door_frame = ttk.LabelFrame(right_frame, text="上舱门控制", padding=2)
        upper_door_frame.grid(row=0, column=0, sticky=tk.NSEW, pady=5)
        
        # 上舱门电机状态
        ttk.Label(upper_door_frame, text="电机状态:").grid(row=0, column=0, sticky=tk.W, pady=5)
        self.upper_motor_status_label = ttk.Label(upper_door_frame, text="已禁用", foreground="blue")
        self.upper_motor_status_label.grid(row=0, column=1, padx=5, pady=0, sticky=tk.W)
        self.upper_enable_btn = ttk.Button(upper_door_frame, text="使能电机", command=self.upper_enable_motor, state="disabled")
        self.upper_enable_btn.grid(row=0, column=2, padx=5, pady=5)
        self.upper_disable_btn = ttk.Button(upper_door_frame, text="禁用电机", command=self.upper_disable_motor, state="disabled")
        self.upper_disable_btn.grid(row=0, column=3, padx=5, pady=5)
        
        # 上舱门限位状态
        ttk.Label(upper_door_frame, text="限位状态:").grid(row=1, column=0, sticky=tk.W, pady=0)
        self.upper_limit_label = ttk.Label(upper_door_frame, text="上限位: 断开 | 下限位: 断开")
        self.upper_limit_label.grid(row=1, column=1, columnspan=3, sticky=tk.W, pady=0)
        
        # 上舱门复位控制
        ttk.Label(upper_door_frame, text="复位距离(mm):").grid(row=2, column=0, sticky=tk.W, pady=0)
        self.upper_cal_input = ttk.Entry(upper_door_frame, width=10)
        self.upper_cal_input.insert(0, "220.0")
        self.upper_cal_input.grid(row=2, column=1, padx=5, pady=0)
        self.upper_reset_up_btn = ttk.Button(upper_door_frame, text="上复位", command=self.upper_reset_up, state="disabled")
        self.upper_reset_up_btn.grid(row=2, column=2, padx=5, pady=5)
        self.upper_reset_down_btn = ttk.Button(upper_door_frame, text="下复位", command=self.upper_reset_down, state="disabled")
        self.upper_reset_down_btn.grid(row=2, column=3, padx=5, pady=5)
        
        # 上舱门移动控制
        ttk.Label(upper_door_frame, text="移动距离(mm):").grid(row=3, column=0, sticky=tk.W, pady=0)
        self.upper_move_input = ttk.Entry(upper_door_frame, width=10)
        self.upper_move_input.insert(0, "10.0")
        self.upper_move_input.grid(row=3, column=1, padx=5, pady=5)
        self.upper_move_up_btn = ttk.Button(upper_door_frame, text="上移", command=self.upper_move_up, state="disabled")
        self.upper_move_up_btn.grid(row=3, column=2, padx=5, pady=5)
        self.upper_move_down_btn = ttk.Button(upper_door_frame, text="下移", command=self.upper_move_down, state="disabled")
        self.upper_move_down_btn.grid(row=3, column=3, padx=5, pady=5)
        
        # 下舱门控制
        lower_door_frame = ttk.LabelFrame(right_frame, text="下舱门控制", padding=2)
        lower_door_frame.grid(row=1, column=0, sticky=tk.NSEW, pady=0)
        
        # 下舱门电机状态
        ttk.Label(lower_door_frame, text="电机状态:").grid(row=0, column=0, sticky=tk.W, pady=0)
        self.lower_motor_status_label = ttk.Label(lower_door_frame, text="已禁用", foreground="blue")
        self.lower_motor_status_label.grid(row=0, column=1, padx=5, pady=0, sticky=tk.W)
        self.lower_enable_btn = ttk.Button(lower_door_frame, text="使能电机", command=self.lower_enable_motor, state="disabled")
        self.lower_enable_btn.grid(row=0, column=2, padx=5, pady=5)
        self.lower_disable_btn = ttk.Button(lower_door_frame, text="禁用电机", command=self.lower_disable_motor, state="disabled")
        self.lower_disable_btn.grid(row=0, column=3, padx=5, pady=5)
        
        # 下舱门限位状态
        ttk.Label(lower_door_frame, text="限位状态:").grid(row=1, column=0, sticky=tk.W, pady=5)
        self.lower_limit_label = ttk.Label(lower_door_frame, text="上限位: 断开 | 下限位: 断开")
        self.lower_limit_label.grid(row=1, column=1, columnspan=3, sticky=tk.W, pady=5)
        
        # 下舱门复位控制
        ttk.Label(lower_door_frame, text="复位距离(mm):").grid(row=2, column=0, sticky=tk.W, pady=5)
        self.lower_cal_input = ttk.Entry(lower_door_frame, width=10)
        self.lower_cal_input.insert(0, "220.0")
        self.lower_cal_input.grid(row=2, column=1, padx=5, pady=5)
        self.lower_reset_up_btn = ttk.Button(lower_door_frame, text="上复位", command=self.lower_reset_up, state="disabled")
        self.lower_reset_up_btn.grid(row=2, column=2, padx=5, pady=5)
        self.lower_reset_down_btn = ttk.Button(lower_door_frame, text="下复位", command=self.lower_reset_down, state="disabled")
        self.lower_reset_down_btn.grid(row=2, column=3, padx=5, pady=5)
        
        # 下舱门移动控制
        ttk.Label(lower_door_frame, text="移动距离(mm):").grid(row=3, column=0, sticky=tk.W, pady=0)
        self.lower_move_input = ttk.Entry(lower_door_frame, width=10)
        self.lower_move_input.insert(0, "10.0")
        self.lower_move_input.grid(row=3, column=1, padx=5, pady=0)
        self.lower_move_up_btn = ttk.Button(lower_door_frame, text="上移", command=self.lower_move_up, state="disabled")
        self.lower_move_up_btn.grid(row=3, column=2, padx=5, pady=5)
        self.lower_move_down_btn = ttk.Button(lower_door_frame, text="下移", command=self.lower_move_down, state="disabled")
        self.lower_move_down_btn.grid(row=3, column=3, padx=5, pady=5)
        
        # ROS启动区域 - 保持原来的LabelFrame结构，但只有按钮
        ros_frame = ttk.LabelFrame(right_frame, text="ROS控制", padding=10)
        ros_frame.grid(row=2, column=0, sticky=tk.EW, pady=(0, 5))
        
        # 只有一个按钮，使用pack使其占满宽度
        self.ros_btn = ttk.Button(ros_frame, text="启动launch文件", 
                                 command=self.toggle_ros_launch)
        self.ros_btn.pack(fill=tk.X, pady=5)
        
        # 布局权重设置
        main_frame.grid_columnconfigure(0, weight=1)
        main_frame.grid_columnconfigure(1, weight=2)
        main_frame.grid_rowconfigure(0, weight=1)
        
        # 配置右侧区域行权重
        right_frame.grid_rowconfigure(0, weight=1)
        right_frame.grid_rowconfigure(1, weight=1)
        right_frame.grid_rowconfigure(2, weight=0)
        
        # 初始化按钮状态
        self.update_button_states()

    def update_button_states(self):
        """更新按钮状态"""
        if self.ros_running:
            # ROS运行状态
            self.connect_btn.config(state="disabled")
            self.refresh_btn.config(state="disabled")
            self.save_btn.config(state="disabled")
            self.ros_btn.config(text="关闭launch文件")
            
            # 启用控制按钮
            self.init_btn.config(state="normal")
            self.reset_btn.config(state="normal")
            self.upper_enable_btn.config(state="normal")
            self.upper_disable_btn.config(state="normal")
            self.upper_reset_up_btn.config(state="normal")
            self.upper_reset_down_btn.config(state="normal")
            self.upper_move_up_btn.config(state="normal")
            self.upper_move_down_btn.config(state="normal")
            self.lower_enable_btn.config(state="normal")
            self.lower_disable_btn.config(state="normal")
            self.lower_reset_up_btn.config(state="normal")
            self.lower_reset_down_btn.config(state="normal")
            self.lower_move_up_btn.config(state="normal")
            self.lower_move_down_btn.config(state="normal")
            
        elif self.is_connected:
            # 串口连接状态
            self.ros_btn.config(state="disabled")
            self.ros_btn.config(text="启动launch文件")
            
            # 启用控制按钮
            self.init_btn.config(state="normal")
            self.reset_btn.config(state="normal")
            self.upper_enable_btn.config(state="normal")
            self.upper_disable_btn.config(state="normal")
            self.upper_reset_up_btn.config(state="normal")
            self.upper_reset_down_btn.config(state="normal")
            self.upper_move_up_btn.config(state="normal")
            self.upper_move_down_btn.config(state="normal")
            self.lower_enable_btn.config(state="normal")
            self.lower_disable_btn.config(state="normal")
            self.lower_reset_up_btn.config(state="normal")
            self.lower_reset_down_btn.config(state="normal")
            self.lower_move_up_btn.config(state="normal")
            self.lower_move_down_btn.config(state="normal")
            
        else:
            # 未连接状态
            self.ros_btn.config(state="normal")
            self.ros_btn.config(text="启动launch文件")
            
            # 启用串口相关按钮
            self.connect_btn.config(state="normal")
            self.refresh_btn.config(state="normal")
            self.save_btn.config(state="normal")
            
            # 禁用控制按钮
            self.init_btn.config(state="disabled")
            self.reset_btn.config(state="disabled")
            self.upper_enable_btn.config(state="disabled")
            self.upper_disable_btn.config(state="disabled")
            self.upper_reset_up_btn.config(state="disabled")
            self.upper_reset_down_btn.config(state="disabled")
            self.upper_move_up_btn.config(state="disabled")
            self.upper_move_down_btn.config(state="disabled")
            self.lower_enable_btn.config(state="disabled")
            self.lower_disable_btn.config(state="disabled")
            self.lower_reset_up_btn.config(state="disabled")
            self.lower_reset_down_btn.config(state="disabled")
            self.lower_move_up_btn.config(state="disabled")
            self.lower_move_down_btn.config(state="disabled")


    def scan_serial_ports(self):
        """扫描可用串口并更新下拉列表 - Ubuntu适配版"""
        ports = []
        
        # 获取标准串口
        for port in serial.tools.list_ports.comports():
            port_path = port.device
            # 过滤掉系统串口（ttyS*）
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
            
        self.log(f"扫描到 {len(ports)} 个串口")
        return ports

    def toggle_serial(self):
        """连接/断开串口"""
        if not self.is_connected:
            # 连接串口
            port = self.port_combo.get()
            if not port:
                self.log("操作失败：未选择串口")
                messagebox.showwarning("警告", "请选择串口")
                return
            
            # 检查串口是否存在
            if not os.path.exists(port):
                self.log(f"串口不存在: {port}")
                messagebox.showerror("错误", f"串口不存在: {port}\n请检查设备连接")
                return
            
            try:
                self.ser = serial.Serial(
                    port=port,
                    baudrate=115200,
                    timeout=0.1
                )
                self.is_connected = True
                self.connect_btn.config(text="断开")
                self.status_label.config(text="已连接", foreground="green")
                self.log(f"成功连接串口：{port}（波特率115200）")
                
                # 启动串口接收线程
                self.running = True
                self.serial_thread = threading.Thread(target=self.read_serial, daemon=True)
                self.serial_thread.start()
                
                # 更新按钮状态
                self.update_button_states()
                
            except Exception as e:
                error_msg = str(e)
                if "Permission denied" in error_msg:
                    error_msg = f"权限不足，请运行: sudo chmod 666 {port}"
                self.log(f"连接串口失败：{error_msg}")
                messagebox.showerror("连接失败", f"无法连接串口:\n{error_msg}")
        else:
            # 断开串口
            self.running = False  # 先设置标志，再关闭串口
            time.sleep(0.1)  # 给线程一点时间响应
            
            if self.ser:
                try:
                    self.ser.close()
                except Exception as e:
                    self.log(f"关闭串口时出错: {e}")
            
            self.is_connected = False
            self.connect_btn.config(text="连接")
            self.status_label.config(text="未连接", foreground="red")
            self.log("断开串口连接")
            
            # 更新按钮状态
            self.update_button_states()

    def read_serial(self):
        """串口接收线程：持续读取下位机数据并解析"""
        while self.running and self.ser and self.ser.is_open:
            try:
                if self.ser.in_waiting > 0:
                    data = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    if data:
                        if data.startswith("S:"):
                            self.parse_status(data)
            except serial.SerialException as e:
                if self.running:  # 如果不是主动断开
                    self.log(f"串口读取异常：{str(e)}")
                    # 在GUI线程中执行断开操作
                    self.root.after(0, self.toggle_serial)
                break
            except Exception as e:
                if self.running:  # 如果不是主动断开
                    self.log(f"串口读取异常：{str(e)}")
                time.sleep(0.01)
            
            time.sleep(0.01)

    def parse_status(self, status_str):
        """解析下位机状态数据并更新UI"""
        try:
            parts = status_str.split(':')[1].split(',')
            if len(parts) >= 12:
                self.sys_state = int(parts[0])
                self.upper_motor_enabled = bool(int(parts[1]))
                self.lower_motor_enabled = bool(int(parts[2]))
                self.c1u_complete = bool(int(parts[3]))
                self.c1d_complete = bool(int(parts[4]))
                self.c2u_complete = bool(int(parts[5]))
                self.c2d_complete = bool(int(parts[6]))
                self.emergency_stop = bool(int(parts[7]))
                self.c1u_limit = bool(int(parts[8]))
                self.c1d_limit = bool(int(parts[9]))
                self.c2u_limit = bool(int(parts[10]))
                self.c2d_limit = bool(int(parts[11]))
                
                # 在GUI线程中更新UI
                self.root.after(0, self.update_ui_from_state)
                
        except Exception as e:
            self.log(f"状态解析失败：{str(e)}")

    def update_ui_from_state(self):
        """从状态更新UI显示"""
        # 更新电机状态显示
        if self.upper_motor_enabled:
            self.upper_motor_status_label.config(text="已使能", foreground="green")
        else:
            self.upper_motor_status_label.config(text="已禁用", foreground="red")
            
        if self.lower_motor_enabled:
            self.lower_motor_status_label.config(text="已使能", foreground="green")
        else:
            self.lower_motor_status_label.config(text="已禁用", foreground="red")
        
        # 更新限位状态显示
        upper_limit_text = f"上限位: {'闭合' if self.c1u_limit else '断开'} | 下限位: {'闭合' if self.c1d_limit else '断开'}"
        self.upper_limit_label.config(text=upper_limit_text)
        
        lower_limit_text = f"上限位: {'闭合' if self.c2u_limit else '断开'} | 下限位: {'闭合' if self.c2d_limit else '断开'}"
        self.lower_limit_label.config(text=lower_limit_text)
        
        # 更新设备状态显示（根据数字状态码）
        state_text = {
            0: "正常",
            1: "初始化中",
            2: "急停中",
            3: "电机重置中",
            4: "上门上复位",
            5: "上门下复位",
            6: "下门上复位",
            7: "下门下复位",
            8: "上门向上移动",
            9: "上门向下移动",
            10: "下门向上移动",
            11: "下门向下移动"
        }.get(self.sys_state, f"未知({self.sys_state})")
        
        if self.emergency_stop:
            state_text = "⚠ 急停已触发 ⚠"
            self.status_label.config(foreground="red")
            self.log("设备触发急停状态！")
        else:
            self.status_label.config(foreground="green")
        self.status_label.config(text=state_text)

    # ====== 串口控制方法 ======
    def send_cmd(self, cmd, operation_desc):
        """发送基础指令（串口）"""
        if not self.is_connected or not self.ser or not self.ser.is_open:
            self.log(f"操作失败（{operation_desc}）：未连接串口")
            messagebox.showwarning("警告", "请先连接串口")
            return
        try:
            self.ser.write(f"{cmd}\n".encode())
            self.log(f"执行操作：{operation_desc}")
            
            # 发送指令后立即更新本地状态
            if cmd == "E1":
                self.upper_motor_enabled = True
                self.upper_motor_status_label.config(text="已使能", foreground="green")
            elif cmd == "D1":
                self.upper_motor_enabled = False
                self.upper_motor_status_label.config(text="已禁用", foreground="red")
            elif cmd == "E2":
                self.lower_motor_enabled = True
                self.lower_motor_status_label.config(text="已使能", foreground="green")
            elif cmd == "D2":
                self.lower_motor_enabled = False
                self.lower_motor_status_label.config(text="已禁用", foreground="red")
                
        except Exception as e:
            self.log(f"操作失败（{operation_desc}）：{str(e)}")
            messagebox.showerror("发送失败", f"无法发送命令:\n{str(e)}")

    def send_cal_cmd(self, cmd_prefix, distance_str, operation_desc):
        """发送复位指令（串口）"""
        if not self.is_connected:
            self.log(f"操作失败（{operation_desc}）：未连接串口")
            return
            
        try:
            distance = float(distance_str.strip())
            if distance <= 0 or distance > 1000:
                self.log(f"{operation_desc}：复位距离无效（{distance}mm），使用默认值220mm")
                cmd = cmd_prefix
            else:
                cmd = f"{cmd_prefix},{distance}"
                self.log(f"{operation_desc}：复位距离设置为{distance}mm")
        except (ValueError, TypeError):
            self.log(f"{operation_desc}：复位距离输入错误，使用默认值220mm")
            cmd = cmd_prefix
        
        self.send_cmd(cmd, operation_desc)

    def send_move_cmd(self, cmd_prefix, distance_str, operation_desc):
        """发送移动指令（串口）"""
        if not self.is_connected:
            self.log(f"操作失败（{operation_desc}）：未连接串口")
            return
            
        try:
            distance = float(distance_str.strip())
            if distance <= 0:
                self.log(f"{operation_desc}：移动距离无效（{distance}mm），使用默认值10mm")
                cmd = cmd_prefix
            else:
                cmd = f"{cmd_prefix},{distance}"
                self.log(f"{operation_desc}：移动距离设置为{distance}mm")
        except (ValueError, TypeError):
            self.log(f"{operation_desc}：移动距离输入错误，使用默认值10mm")
            cmd = cmd_prefix
        
        self.send_cmd(cmd, operation_desc)

    def send_init(self):
        """发送系统初始化指令"""
        if self.ros_running:
            self.send_init_ros()
            return
            
        if not self.is_connected:
            self.log("执行系统初始化失败：未连接串口")
            messagebox.showwarning("警告", "请先连接串口")
            return
            
        try:
            upper_dist = float(self.init_upper_entry.get().strip())
            lower_dist = float(self.init_lower_entry.get().strip())
            
            if upper_dist <= 0 or upper_dist > 1000:
                self.log(f"系统初始化：上门初始化距离无效（{upper_dist}mm），使用默认值220mm")
                upper_dist = 220.0
            else:
                self.log(f"系统初始化：上门初始化距离设置为{upper_dist}mm")
            
            if lower_dist <= 0 or lower_dist > 1000:
                self.log(f"系统初始化：下门初始化距离无效（{lower_dist}mm），使用默认值220mm")
                lower_dist = 220.0
            else:
                self.log(f"系统初始化：下门初始化距离设置为{lower_dist}mm")
                
            cmd = f"INIT,{upper_dist},{lower_dist}"
            self.send_cmd(cmd, "执行系统初始化")
        except ValueError:
            self.log("系统初始化：距离输入错误，使用默认值220mm（上下门）")
            cmd = "INIT"
            self.send_cmd(cmd, "执行系统初始化")

    def send_reset(self):
        """发送电机重置指令"""
        if self.ros_running:
            self.send_reset_ros()
            return
            
        self.send_cmd("R", "执行电机重置")
        # 重置电机状态显示
        self.upper_motor_status_label.config(text="已禁用", foreground="red")
        self.lower_motor_status_label.config(text="已禁用", foreground="red")

    # ====== 按钮控制方法 ======
    def upper_enable_motor(self):
        """使能上舱门电机"""
        if self.ros_running:
            self.send_upper_motor_cmd(True)
        else:
            self.send_cmd("E1", "上舱门使能电机")

    def upper_disable_motor(self):
        """禁用上舱门电机"""
        if self.ros_running:
            self.send_upper_motor_cmd(False)
        else:
            self.send_cmd("D1", "上舱门禁用电机")

    def upper_reset_up(self):
        """上舱门上复位"""
        distance = self.upper_cal_input.get()
        if self.ros_running:
            self.send_upper_reset_cmd("up", distance)
        else:
            self.send_cal_cmd("C1U", distance, "上舱门执行上复位")

    def upper_reset_down(self):
        """上舱门下复位"""
        distance = self.upper_cal_input.get()
        if self.ros_running:
            self.send_upper_reset_cmd("down", distance)
        else:
            self.send_cal_cmd("C1D", distance, "上舱门执行下复位")

    def upper_move_up(self):
        """上舱门向上移动"""
        distance = self.upper_move_input.get()
        if self.ros_running:
            self.send_upper_control_cmd("up", distance)
        else:
            self.send_move_cmd("M1U", distance, "上舱门向上移动")

    def upper_move_down(self):
        """上舱门向下移动"""
        distance = self.upper_move_input.get()
        if self.ros_running:
            self.send_upper_control_cmd("down", distance)
        else:
            self.send_move_cmd("M1D", distance, "上舱门向下移动")

    def lower_enable_motor(self):
        """使能下舱门电机"""
        if self.ros_running:
            self.send_lower_motor_cmd(True)
        else:
            self.send_cmd("E2", "下舱门使能电机")

    def lower_disable_motor(self):
        """禁能下舱门电机"""
        if self.ros_running:
            self.send_lower_motor_cmd(False)
        else:
            self.send_cmd("D2", "下舱门禁用电机")

    def lower_reset_up(self):
        """下舱门上复位"""
        distance = self.lower_cal_input.get()
        if self.ros_running:
            self.send_lower_reset_cmd("up", distance)
        else:
            self.send_cal_cmd("C2U", distance, "下舱门执行上复位")

    def lower_reset_down(self):
        """下舱门下复位"""
        distance = self.lower_cal_input.get()
        if self.ros_running:
            self.send_lower_reset_cmd("down", distance)
        else:
            self.send_cal_cmd("C2D", distance, "下舱门执行下复位")

    def lower_move_up(self):
        """下舱门向上移动"""
        distance = self.lower_move_input.get()
        if self.ros_running:
            self.send_lower_control_cmd("up", distance)
        else:
            self.send_move_cmd("M2U", distance, "下舱门向上移动")

    def lower_move_down(self):
        """下舱门向下移动"""
        distance = self.lower_move_input.get()
        if self.ros_running:
            self.send_lower_control_cmd("down", distance)
        else:
            self.send_move_cmd("M2D", distance, "下舱门向下移动")

    # ====== ROS话题控制方法 ======
    def send_upper_motor_cmd(self, enable):
        """通过ROS话题控制上舱门电机"""
        if not ROS_AVAILABLE or not self.upper_motor_pub:
            self.log("ROS功能不可用或未初始化")
            return
            
        try:
            msg = Bool()
            msg.data = enable
            self.upper_motor_pub.publish(msg)
            operation = "上舱门使能电机" if enable else "上舱门禁用电机"
            self.log(f"通过ROS话题发送: {operation}")
        except Exception as e:
            self.log(f"ROS话题发送失败: {e}")

    def send_upper_reset_cmd(self, direction, distance_str):
        """通过ROS话题控制上舱门复位"""
        if not ROS_AVAILABLE or not self.upper_reset_pub:
            self.log("ROS功能不可用或未初始化")
            return
            
        try:
            distance = float(distance_str.strip())
            if distance <= 0 or distance > 1000:
                msg = String()
                msg.data = direction
                self.log(f"上舱门复位: 使用默认距离")
            else:
                msg = String()
                msg.data = f"{direction},{distance}"
                self.log(f"上舱门复位: {direction}, 距离{distance}mm")
                
            self.upper_reset_pub.publish(msg)
        except Exception as e:
            self.log(f"ROS话题发送失败: {e}")

    def send_upper_control_cmd(self, direction, distance_str):
        """通过ROS话题控制上舱门移动"""
        if not ROS_AVAILABLE or not self.upper_control_pub:
            self.log("ROS功能不可用或未初始化")
            return
            
        try:
            distance = float(distance_str.strip())
            if distance <= 0:
                msg = String()
                msg.data = direction
                self.log(f"上舱门移动: 使用默认距离")
            else:
                msg = String()
                msg.data = f"{direction},{distance}"
                self.log(f"上舱门移动: {direction}, 距离{distance}mm")
                
            self.upper_control_pub.publish(msg)
        except Exception as e:
            self.log(f"ROS话题发送失败: {e}")

    def send_lower_motor_cmd(self, enable):
        """通过ROS话题控制下舱门电机"""
        if not ROS_AVAILABLE or not self.lower_motor_pub:
            self.log("ROS功能不可用或未初始化")
            return
            
        try:
            msg = Bool()
            msg.data = enable
            self.lower_motor_pub.publish(msg)
            operation = "下舱门使能电机" if enable else "下舱门禁用电机"
            self.log(f"通过ROS话题发送: {operation}")
        except Exception as e:
            self.log(f"ROS话题发送失败: {e}")

    def send_lower_reset_cmd(self, direction, distance_str):
        """通过ROS话题控制下舱门复位"""
        if not ROS_AVAILABLE or not self.lower_reset_pub:
            self.log("ROS功能不可用或未初始化")
            return
            
        try:
            distance = float(distance_str.strip())
            if distance <= 0 or distance > 1000:
                msg = String()
                msg.data = direction
                self.log(f"下舱门复位: 使用默认距离")
            else:
                msg = String()
                msg.data = f"{direction},{distance}"
                self.log(f"下舱门复位: {direction}, 距离{distance}mm")
                
            self.lower_reset_pub.publish(msg)
        except Exception as e:
            self.log(f"ROS话题发送失败: {e}")

    def send_lower_control_cmd(self, direction, distance_str):
        """通过ROS话题控制下舱门移动"""
        if not ROS_AVAILABLE or not self.lower_control_pub:
            self.log("ROS功能不可用或未初始化")
            return
            
        try:
            distance = float(distance_str.strip())
            if distance <= 0:
                msg = String()
                msg.data = direction
                self.log(f"下舱门移动: 使用默认距离")
            else:
                msg = String()
                msg.data = f"{direction},{distance}"
                self.log(f"下舱门移动: {direction}, 距离{distance}mm")
                
            self.lower_control_pub.publish(msg)
        except Exception as e:
            self.log(f"ROS话题发送失败: {e}")

    def send_init_ros(self):
        """通过ROS话题发送系统初始化"""
        if not ROS_AVAILABLE or not self.init_pub:
            self.log("ROS功能不可用或未初始化")
            return
            
        try:
            upper_dist = float(self.init_upper_entry.get().strip())
            lower_dist = float(self.init_lower_entry.get().strip())
            
            if upper_dist <= 0 or upper_dist > 1000 or lower_dist <= 0 or lower_dist > 1000:
                msg = String()
                msg.data = "INIT"
                self.log("系统初始化: 使用默认距离")
            else:
                msg = String()
                msg.data = f"{upper_dist},{lower_dist}"
                self.log(f"系统初始化: 上门{upper_dist}mm, 下门{lower_dist}mm")
                
            self.init_pub.publish(msg)
        except ValueError:
            msg = String()
            msg.data = "INIT"
            self.init_pub.publish(msg)
            self.log("系统初始化: 使用默认距离")
        except Exception as e:
            self.log(f"ROS话题发送失败: {e}")

    def send_reset_ros(self):
        """通过ROS话题发送电机重置"""
        if not ROS_AVAILABLE or not self.reset_pub:
            self.log("ROS功能不可用或未初始化")
            return
            
        try:
            msg = Bool()
            msg.data = True
            self.reset_pub.publish(msg)
            self.log("通过ROS话题发送: 电机重置")
        except Exception as e:
            self.log(f"ROS话题发送失败: {e}")

    # ====== ROS状态回调函数 ======
    def device_state_callback(self, msg):
        """设备状态话题回调（Int32类型）"""
        self.sys_state = msg.data
        self.root.after(0, self.update_status_from_sys_state)

    def update_status_from_sys_state(self):
        """根据系统状态码更新状态显示"""
        state_text = {
            0: "正常",
            1: "初始化中",
            2: "急停中",
            3: "电机重置中",
            4: "上门上复位",
            5: "上门下复位",
            6: "下门上复位",
            7: "下门下复位",
            8: "上门向上移动",
            9: "上门向下移动",
            10: "下门向上移动",
            11: "下门向下移动"
        }.get(self.sys_state, f"未知({self.sys_state})")
        
        # 状态码>=100可能是复合状态码，但按照你的ROS节点设计应该不会超过99
        if self.sys_state >= 100:
            state_text = f"复合状态({self.sys_state})"
        
        # 注意：ROS模式下我们不知道emergency_stop状态，所以不显示急停
        self.status_label.config(text=state_text, foreground="green")

    def upper_motor_callback(self, msg):
        """上舱门电机状态回调"""
        self.upper_motor_enabled = msg.data
        self.root.after(0, lambda m=msg.data: self.upper_motor_status_label.config(
            text="已使能" if m else "已禁用",
            foreground="green" if m else "red"
        ))

    def upper_up_limit_callback(self, msg):
        """上舱门上限位状态回调"""
        self.c1u_limit = msg.data
        self.update_upper_limit_display()

    def upper_down_limit_callback(self, msg):
        """上舱门下限位状态回调"""
        self.c1d_limit = msg.data
        self.update_upper_limit_display()

    def lower_motor_callback(self, msg):
        """下舱门电机状态回调"""
        self.lower_motor_enabled = msg.data
        self.root.after(0, lambda m=msg.data: self.lower_motor_status_label.config(
            text="已使能" if m else "已禁用",
            foreground="green" if m else "red"
        ))

    def lower_up_limit_callback(self, msg):
        """下舱门上限位状态回调"""
        self.c2u_limit = msg.data
        self.update_lower_limit_display()

    def lower_down_limit_callback(self, msg):
        """下舱门下限位状态回调"""
        self.c2d_limit = msg.data
        self.update_lower_limit_display()

    def sys_state_callback(self, msg):
        """系统状态回调"""
        self.sys_state = msg.data
        self.root.after(0, self.update_status_from_sys_state)

    def emergency_callback(self, msg):
        """急停状态回调"""
        self.emergency_stop = msg.data
        if self.emergency_stop:
            self.root.after(0, lambda: self.status_label.config(text="⚠ 急停已触发 ⚠", foreground="red"))
            self.log("设备触发急停状态！")

    def update_upper_limit_display(self):
        """更新上舱门限位显示"""
        text = f"上限位: {'闭合' if self.c1u_limit else '断开'} | 下限位: {'闭合' if self.c1d_limit else '断开'}"
        self.root.after(0, lambda t=text: self.upper_limit_label.config(text=t))

    def update_lower_limit_display(self):
        """更新下舱门限位显示"""
        text = f"上限位: {'闭合' if self.c2u_limit else '断开'} | 下限位: {'闭合' if self.c2d_limit else '断开'}"
        self.root.after(0, lambda t=text: self.lower_limit_label.config(text=t))

    # ====== ROS Launch控制 ======
    def toggle_ros_launch(self):
        """切换ROS Launch"""
        if not self.ros_running:
            self.start_ros_launch()
        else:
            self.stop_ros_launch()

    def start_ros_launch(self):
        """启动ROS Launch"""
        # 如果串口已连接，先断开
        if self.is_connected:
            self.toggle_serial()
        
        try:
            # 构建roslaunch命令
            roslaunch_cmd = 'roslaunch aihitplt_hardware_test aihitplt_delivery_node.launch'
            
            # 在新终端中启动
            cmd = [
                'gnome-terminal',
                '--title=送物模块ROS节点',
                '--geometry=80x24+100+100',
                '--',
                'bash', '-c',
                f'source ~/.bashrc && '
                f'source ~/aihitplt_ws/devel/setup.bash && '
                f'{roslaunch_cmd}; '
                f'echo "送物模块ROS节点运行中...按Enter键关闭终端"; '
                f'read'
            ]
            
            # 启动进程
            self.ros_process = subprocess.Popen(
                cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                start_new_session=True
            )
            
            self.ros_running = True
            self.status_label.config(text="正常", foreground="green")  # 简化状态显示
            
            # 更新按钮状态
            self.update_button_states()
            
            self.log("正在启动ROS节点...")
            self.log("ROS节点启动成功，状态: 正常")
            
            # 等待ROS启动后初始化ROS节点连接
            self.root.after(3000, self.init_ros_node_connection)
            
        except Exception as e:
            messagebox.showerror("启动失败", f"无法启动ROS Launch:\n{e}")
            self.log(f"ROS启动失败: {e}")
            # 如果启动失败，重置状态
            self.ros_running = False
            self.update_button_states()

    def init_ros_node_connection(self):
        """初始化ROS节点连接（订阅话题）"""
        if not ROS_AVAILABLE:
            self.log("未安装ROS Python库，无法连接ROS话题")
            return
        
        try:
            # 初始化ROS节点（如果尚未初始化）
            try:
                rospy.init_node('delivery_gui_node', anonymous=True, disable_signals=True)
            except rospy.exceptions.ROSException:
                # 节点已经初始化
                pass
            
            # 创建话题订阅器
            self.state_sub = rospy.Subscriber(
                'delivery_device_state',
                Int32,
                self.device_state_callback
            )
            
            self.upper_motor_sub = rospy.Subscriber(
                'upper_motor_state',
                Bool,
                self.upper_motor_callback
            )
            
            self.upper_up_limit_sub = rospy.Subscriber(
                'upper_up_limit_state',
                Bool,
                self.upper_up_limit_callback
            )
            
            self.upper_down_limit_sub = rospy.Subscriber(
                'upper_down_limit_state',
                Bool,
                self.upper_down_limit_callback
            )
            
            self.lower_motor_sub = rospy.Subscriber(
                'lower_motor_state',
                Bool,
                self.lower_motor_callback
            )
            
            self.lower_up_limit_sub = rospy.Subscriber(
                'lower_up_limit_state',
                Bool,
                self.lower_up_limit_callback
            )
            
            self.lower_down_limit_sub = rospy.Subscriber(
                'lower_down_limit_state',
                Bool,
                self.lower_down_limit_callback
            )
            
            self.sys_state_sub = rospy.Subscriber(
                'delivery_system_state',
                Int32,
                self.sys_state_callback
            )
            
            self.emergency_sub = rospy.Subscriber(
                'emergency_stop',
                Bool,
                self.emergency_callback
            )
            
            # 创建命令发布器
            self.upper_motor_pub = rospy.Publisher(
                'upper_motor_state_cmd',
                Bool,
                queue_size=10
            )
            
            self.upper_reset_pub = rospy.Publisher(
                'upper_reset_cmd',
                String,
                queue_size=10
            )
            
            self.upper_control_pub = rospy.Publisher(
                'upper_control_cmd',
                String,
                queue_size=10
            )
            
            self.lower_motor_pub = rospy.Publisher(
                'lower_motor_state_cmd',
                Bool,
                queue_size=10
            )
            
            self.lower_reset_pub = rospy.Publisher(
                'lower_reset_cmd',
                String,
                queue_size=10
            )
            
            self.lower_control_pub = rospy.Publisher(
                'lower_control_cmd',
                String,
                queue_size=10
            )
            
            self.init_pub = rospy.Publisher(
                'delivery_init_cmd',
                String,
                queue_size=10
            )
            
            self.reset_pub = rospy.Publisher(
                'motor_reset_cmd',
                Bool,
                queue_size=10
            )
            
            # 等待发布器建立连接
            time.sleep(0.5)
            
            self.ros_node_initialized = True
            self.log("ROS话题连接初始化成功")
            self.log("现在可以通过ROS话题控制设备")
            
        except Exception as e:
            error_msg = f"ROS话题连接初始化失败: {e}"
            self.log(error_msg)

    def stop_ros_launch(self):
        """停止ROS Launch"""
        if self.ros_running:
            try:
                # 清理ROS话题
                self.ros_node_initialized = False
                
                if self.state_sub:
                    self.state_sub.unregister()
                if self.upper_motor_sub:
                    self.upper_motor_sub.unregister()
                if self.upper_up_limit_sub:
                    self.upper_up_limit_sub.unregister()
                if self.upper_down_limit_sub:
                    self.upper_down_limit_sub.unregister()
                if self.lower_motor_sub:
                    self.lower_motor_sub.unregister()
                if self.lower_up_limit_sub:
                    self.lower_up_limit_sub.unregister()
                if self.lower_down_limit_sub:
                    self.lower_down_limit_sub.unregister()
                if self.sys_state_sub:
                    self.sys_state_sub.unregister()
                if self.emergency_sub:
                    self.emergency_sub.unregister()
                
                # 清理发布器
                if self.upper_motor_pub:
                    self.upper_motor_pub.unregister()
                if self.upper_reset_pub:
                    self.upper_reset_pub.unregister()
                if self.upper_control_pub:
                    self.upper_control_pub.unregister()
                if self.lower_motor_pub:
                    self.lower_motor_pub.unregister()
                if self.lower_reset_pub:
                    self.lower_reset_pub.unregister()
                if self.lower_control_pub:
                    self.lower_control_pub.unregister()
                if self.init_pub:
                    self.init_pub.unregister()
                if self.reset_pub:
                    self.reset_pub.unregister()
                
                # 终止进程
                import psutil
                try:
                    if self.ros_process:
                        process = psutil.Process(self.ros_process.pid)
                        for child in process.children(recursive=True):
                            try:
                                child.terminate()
                            except:
                                pass
                        process.terminate()
                except:
                    pass
                
                # 终止ROS相关进程
                subprocess.run(['pkill', '-f', 'aihitplt_delivery_node.launch'],
                             stdout=subprocess.DEVNULL,
                             stderr=subprocess.DEVNULL)
                
            except Exception as e:
                self.log(f"停止ROS进程时出错: {e}")
            
            finally:
                # 更新UI
                self.ros_running = False
                self.ros_process = None
                self.status_label.config(text="未连接", foreground="red")
                
                # 更新按钮状态（关键修复：确保关闭ROS后串口相关按钮可用）
                self.update_button_states()
                
                # 重置显示
                self.upper_motor_status_label.config(text="已禁用", foreground="blue")
                self.lower_motor_status_label.config(text="已禁用", foreground="blue")
                self.upper_limit_label.config(text="上限位: 断开 | 下限位: 断开")
                self.lower_limit_label.config(text="上限位: 断开 | 下限位: 断开")
                
                self.log("ROS Launch已停止")

    # ====== 配置文件管理 ======
    def save_serial_config(self):
        """保存串口配置到YAML文件"""
        if self.ros_running:
            messagebox.showwarning("警告", "ROS正在运行，请先关闭ROS Launch再保存配置")
            return
            
        port = self.port_combo.get()
        
        if not port:
            messagebox.showwarning("警告", "请先选择串口")
            return
        
        try:
            # 确保config目录存在
            if not os.path.exists(self.config_dir):
                os.makedirs(self.config_dir, exist_ok=True)
                print(f"已创建config目录: {self.config_dir}")
            
            # 保存当前所有配置
            config = {
                'serial_port': port,
                'baudrate': 115200,
                'upper_init_distance': float(self.init_upper_entry.get() or "220.0"),
                'lower_init_distance': float(self.init_lower_entry.get() or "220.0"),
                'upper_cal_distance': float(self.upper_cal_input.get() or "220.0"),
                'lower_cal_distance': float(self.lower_cal_input.get() or "220.0"),
                'upper_move_distance': float(self.upper_move_input.get() or "10.0"),
                'lower_move_distance': float(self.lower_move_input.get() or "10.0"),
                'last_saved': datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            }
            
            with open(self.config_file, 'w') as f:
                yaml.dump(config, f, default_flow_style=False)
            
            messagebox.showinfo("保存成功", f"配置已保存到:\n{self.config_file}")
            self.log(f"配置已保存到: {self.config_file}")
            
        except Exception as e:
            messagebox.showerror("保存失败", f"无法保存配置: {e}")
            self.log(f"保存配置失败: {e}")

    def load_config(self):
        """从YAML文件加载配置"""
        if not os.path.exists(self.config_file):
            self.log(f"配置文件不存在: {self.config_file}")
            return
        
        try:
            with open(self.config_file, 'r') as f:
                config = yaml.safe_load(f)
            
            if config:
                # 加载串口
                if 'serial_port' in config:
                    self.port_combo.set(config['serial_port'])
                
                # 加载初始化距离
                if 'upper_init_distance' in config:
                    self.init_upper_entry.delete(0, tk.END)
                    self.init_upper_entry.insert(0, str(config['upper_init_distance']))
                
                if 'lower_init_distance' in config:
                    self.init_lower_entry.delete(0, tk.END)
                    self.init_lower_entry.insert(0, str(config['lower_init_distance']))
                
                # 加载校准距离
                if 'upper_cal_distance' in config:
                    self.upper_cal_input.delete(0, tk.END)
                    self.upper_cal_input.insert(0, str(config['upper_cal_distance']))
                
                if 'lower_cal_distance' in config:
                    self.lower_cal_input.delete(0, tk.END)
                    self.lower_cal_input.insert(0, str(config['lower_cal_distance']))
                
                # 加载移动距离
                if 'upper_move_distance' in config:
                    self.upper_move_input.delete(0, tk.END)
                    self.upper_move_input.insert(0, str(config['upper_move_distance']))
                
                if 'lower_move_distance' in config:
                    self.lower_move_input.delete(0, tk.END)
                    self.lower_move_input.insert(0, str(config['lower_move_distance']))
                
                self.log("配置已从文件加载")
                
        except Exception as e:
            self.log(f"加载配置失败: {e}")

    def log(self, msg):
        """日志输出到文本框"""
        self.log_text.insert(tk.END, f"{time.strftime('%H:%M:%S')} - {msg}\n")
        self.log_text.see(tk.END)

    def on_closing(self):
        """窗口关闭时的清理"""
        if self.is_connected:
            self.running = False
            if self.ser:
                try:
                    self.ser.close()
                except:
                    pass
        
        if self.ros_running:
            self.stop_ros_launch()
        
        self.root.destroy()


def main():
    """主函数"""
    # 设置显示环境
    if 'DISPLAY' not in os.environ:
        os.environ['DISPLAY'] = ':0'
    
    root = tk.Tk()
    
    # 窗口居中
    window_width = 870
    window_height = 450
    screen_width = root.winfo_screenwidth()
    screen_height = root.winfo_screenheight()
    x = (screen_width - window_width) // 2
    y = (screen_height - window_height) // 2
    root.geometry(f"{window_width}x{window_height}+{x}+{y}")
    
    app = DeliveryModuleTestUI_Ubuntu(root)
    
    # 绑定关闭事件
    root.protocol("WM_DELETE_WINDOW", app.on_closing)
    
    root.mainloop()


if __name__ == "__main__":
    main()