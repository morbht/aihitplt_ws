#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import tkinter as tk
from tkinter import ttk, messagebox
import serial
import serial.tools.list_ports
import threading
import subprocess
import os
import time
import yaml
import math
from datetime import datetime
import glob
import psutil
from pymycobot.ultraArm import ultraArm

class CompactRobotArmDebugTool:
    def __init__(self, root):
        self.root = root
        self.root.title("机械臂调试工具")
        self.root.geometry("700x400")
        
        # 机械臂相关变量
        self.arm = None
        self.serial_connected = False
        self.serial_port = None
        self.baudrate = 115200
        
        # ROS相关变量
        self.ros_process = None
        self.ros_running = False
        self.ros_pid = None
        
        # 配置文件路径
        self.config_dir = os.path.expanduser("/home/aihit/aihitplt_ws/src/aihitplt_hardware_test/config")
        self.config_file = os.path.join(self.config_dir, "arm_config.yaml")
        os.makedirs(self.config_dir, exist_ok=True)
        
        # 初始化状态变量
        self.status_var = tk.StringVar(value="就绪")
        
        # 创建紧凑界面
        self.create_compact_widgets()
        
        # 加载保存的串口
        self.load_saved_port()
        
        # 绑定关闭事件
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
    
    def create_compact_widgets(self):
        """创建紧凑界面组件"""
        # 主容器
        main_container = ttk.Frame(self.root, padding="5")
        main_container.pack(fill=tk.BOTH, expand=True)
        
        # ========== 左边部分 (1-3部分) - 更窄 ==========
        left_frame = ttk.Frame(main_container, width=250)  # 从300改为250
        left_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=False, padx=(0, 5))
        
        # 第一部分：机械臂连接
        self.create_compact_connection_section(left_frame)
        
        # 第二部分：机械臂状态信息
        self.create_arm_status_section(left_frame)
        
        # 第三部分：基础控制
        self.create_compact_basic_control_section(left_frame)
        
        # ========== 右边部分 (4-6部分) - 更宽 ==========
        right_frame = ttk.Frame(main_container, width=450)  # 从400改为450
        right_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=(5, 0))
        
        # 第四部分：机械臂调试区域
        self.create_compact_debug_section(right_frame)
        
        # 第五部分：机械臂串口配置
        self.create_compact_config_section(right_frame)
        
        # 第六部分：moveit工具
        self.create_compact_moveit_section(right_frame)
        
        # 状态栏
        status_bar = ttk.Label(self.root, textvariable=self.status_var, 
                              relief=tk.SUNKEN, anchor=tk.W, padding=(5, 2))
        status_bar.pack(side=tk.BOTTOM, fill=tk.X)
        
        # 初始刷新串口（不调用update_status）
        self.refresh_ports_silent()
    
    def refresh_ports_silent(self):
        """静默刷新串口列表（不更新状态栏）"""
        current_selection = self.port_combo.get()
        
        ports = []
        for p in serial.tools.list_ports.comports():
            if not any(f'ttyS{i}' in p.device for i in range(32)):
                ports.append(p.device)
        
        ports += [p for p in glob.glob('/dev/ttyUSB*') + glob.glob('/dev/ttyACM*') + 
                  glob.glob('/dev/aihitplt*') + glob.glob('COM*') if p not in ports]
        ports.sort()
        
        self.port_combo['values'] = ports
        
        if current_selection and current_selection in ports:
            self.port_combo.set(current_selection)
        elif ports:
            self.port_combo.current(0)
    
    def create_compact_connection_section(self, parent):
        """第一部分：机械臂连接（紧凑版）"""
        frame = ttk.LabelFrame(parent, text="机械臂连接", padding=5)
        frame.pack(fill=tk.X, pady=(0, 5))
        
        # 串口选择
        port_frame = ttk.Frame(frame)
        port_frame.pack(fill=tk.X, pady=2)
        
        ttk.Label(port_frame, text="串口:").pack(side=tk.LEFT)
        
        self.port_combo = ttk.Combobox(port_frame, width=25, state="readonly")
        self.port_combo.pack(side=tk.LEFT, padx=(5, 10))
        
        # 按钮行
        btn_frame = ttk.Frame(frame)
        btn_frame.pack(fill=tk.X, pady=2)
        
        self.refresh_btn = ttk.Button(
            btn_frame, 
            text="刷新", 
            command=self.refresh_ports,
            width=12
        )
        self.refresh_btn.pack(side=tk.LEFT, padx=(17, 10))
        
        self.connect_btn = ttk.Button(
            btn_frame,
            text="连接",
            command=self.toggle_connection,
            width=12
        )
        self.connect_btn.pack(side=tk.LEFT, padx=10)
        
        self.init_btn = ttk.Button(
            btn_frame,
            text="初始化",
            command=self.init_arm,
            width=12,
            state="disabled"
        )
        self.init_btn.pack(side=tk.LEFT, padx=(8, 0))
    
    def create_arm_status_section(self, parent):
        """创建机械臂状态信息部分 - 改为ARMTesterApp风格"""
        frame = ttk.LabelFrame(parent, text="机械臂状态信息", padding=20)  # 减少内边距
        frame.pack(fill=tk.BOTH, pady=(0, 5))
        
        # 第一行：关节角度
        ttk.Label(frame, text="关节角度", width=8).grid(row=0, column=0, sticky=tk.W, pady=3)
        ttk.Label(frame, text=":").grid(row=0, column=1, sticky=tk.W, padx=2)
        self.angles_label = ttk.Label(frame, text="未连接", width=35, anchor=tk.W)
        self.angles_label.grid(row=0, column=2, sticky=tk.W, pady=3)
        
        # 第二行：关节弧度
        ttk.Label(frame, text="关节弧度", width=8).grid(row=1, column=0, sticky=tk.W, pady=3)
        ttk.Label(frame, text=":").grid(row=1, column=1, sticky=tk.W, padx=2)
        self.radians_label = ttk.Label(frame, text="未连接", width=35, anchor=tk.W)
        self.radians_label.grid(row=1, column=2, sticky=tk.W, pady=3)
        
        # 第三行：坐标
        ttk.Label(frame, text="坐标(mm)", width=10).grid(row=2, column=0, sticky=tk.W, pady=3)
        ttk.Label(frame, text=":").grid(row=2, column=1, sticky=tk.W, padx=2)
        self.coords_label = ttk.Label(frame, text="未连接", width=35, anchor=tk.W)
        self.coords_label.grid(row=2, column=2, sticky=tk.W, pady=3)
        
        # 刷新按钮
        refresh_btn = ttk.Button(frame, text="刷新信息", command=self.refresh_status, width=20)
        refresh_btn.grid(row=3, column=0, columnspan=3, pady=(10, 5), sticky=tk.EW)
        
        # 配置列权重
        frame.columnconfigure(0, weight=0)  # 标签列
        frame.columnconfigure(1, weight=0)  # 冒号列
        frame.columnconfigure(2, weight=1)  # 值列（自动扩展）
    
    def create_compact_basic_control_section(self, parent):
        """第三部分：基础控制（紧凑版）"""
        frame = ttk.LabelFrame(parent, text="基础控制", padding=5)
        frame.pack(fill=tk.X)
        
        # 按钮容器 - 两行布局
        btn_frame1 = ttk.Frame(frame)
        btn_frame1.pack(fill=tk.X, pady=2)
        
        self.open_gripper_btn = ttk.Button(
            btn_frame1,
            text="打开夹爪",
            command=lambda: self.control_gripper(100),
            width=12,
            state="disabled"
        )
        self.open_gripper_btn.pack(side=tk.LEFT, padx=(15, 2))
        
        self.close_gripper_btn = ttk.Button(
            btn_frame1,
            text="关闭夹爪",
            command=lambda: self.control_gripper(0),
            width=12,
            state="disabled"
        )
        self.close_gripper_btn.pack(side=tk.LEFT, padx=20)
        
        self.init_pose_btn = ttk.Button(
            btn_frame1,
            text="初始化姿态",
            command=self.set_init_pose,
            width=12,
            state="disabled"
        )
        self.init_pose_btn.pack(side=tk.LEFT)
    
    def create_compact_debug_section(self, parent):
        """第四部分：机械臂调试区域（紧凑版）- 压缩高度"""
        frame = ttk.LabelFrame(parent, text="调试控制", padding=5)
        frame.pack(fill=tk.BOTH, expand=True, pady=(0, 5))
        
        # 创建Notebook选项卡
        notebook = ttk.Notebook(frame)
        notebook.pack(fill=tk.BOTH, expand=True, pady=2)
        
        # 选项卡1：关节控制
        tab1 = ttk.Frame(notebook, padding=2)  # 减少内边距
        notebook.add(tab1, text="关节控制")
        self.create_compact_joint_tab(tab1)
        
        # 选项卡2：坐标控制
        tab2 = ttk.Frame(notebook, padding=2)  # 减少内边距
        notebook.add(tab2, text="坐标控制")
        self.create_compact_coord_tab(tab2)
        
        # 选项卡3：夹爪控制
        tab3 = ttk.Frame(notebook, padding=2)  # 减少内边距
        notebook.add(tab3, text="夹爪控制")
        self.create_compact_gripper_tab(tab3)
    
    def create_compact_joint_tab(self, parent):
        """紧凑版关节控制选项卡 - 压缩高度"""
        # 关节范围
        joint_ranges = [
            (-150, 170),   # J1
            (-20, 90),     # J2
            (-5, 110),     # J3
            (-179, 179)    # J4
        ]
        
        self.joint_vars = []
        self.joint_entries = []
        self.joint_scales = []
        
        for i in range(4):
            frame = ttk.Frame(parent)
            frame.pack(fill=tk.X, pady=1)  # 减少行间距
            
            ttk.Label(frame, text=f"J{i+1}:", width=4).pack(side=tk.LEFT)
            
            # 滑块（适当减小）
            var = tk.DoubleVar(value=0.0)
            scale = tk.Scale(
                frame,
                from_=joint_ranges[i][0],
                to=joint_ranges[i][1],
                orient=tk.HORIZONTAL,
                length=180,  # 减小长度
                variable=var,
                resolution=1,
                showvalue=0
            )
            scale.pack(side=tk.LEFT, padx=(5, 5), fill=tk.X, expand=True)
            self.joint_scales.append(scale)
            
            # 输入框
            entry = ttk.Entry(frame, textvariable=var, width=8)  # 减小宽度
            entry.pack(side=tk.LEFT, padx=(0, 5))
            self.joint_vars.append(var)
            self.joint_entries.append(entry)
        
        # 按钮行
        button_frame = ttk.Frame(parent)
        button_frame.pack(fill=tk.X, pady=(10, 3))  # 减少上下边距
        
        self.read_joints_btn = ttk.Button(
            button_frame,
            text="读取角度",
            command=self.read_current_joints,
            width=12,
            state="disabled"
        )
        self.read_joints_btn.pack(side=tk.LEFT, padx=(20, 60))
        
        self.send_joints_btn = ttk.Button(
            button_frame,
            text="发送角度",
            command=self.send_joints,
            width=12,
            state="disabled"
        )
        self.send_joints_btn.pack(side=tk.LEFT)
    
    def create_compact_coord_tab(self, parent):
        """紧凑版坐标控制选项卡 - 压缩高度"""
        # 坐标范围和默认值
        coord_ranges = [
            (-360, 365.55),    # X
            (-365.55, 365.55), # Y
            (-140, 130),       # Z
            (-179, 179)        # θ
        ]
        coord_defaults = [235.55, 0, 130.0, 0]
        coord_names = ["X:", "Y:", "Z:", "θ:"]
        
        self.coord_vars = []
        self.coord_entries = []
        self.coord_scales = []
        
        for i in range(4):
            frame = ttk.Frame(parent)
            frame.pack(fill=tk.X, pady=1)  # 减少行间距
            
            ttk.Label(frame, text=coord_names[i], width=4).pack(side=tk.LEFT)
            
            # 滑块（适当减小）
            var = tk.DoubleVar(value=coord_defaults[i])
            scale = tk.Scale(
                frame,
                from_=coord_ranges[i][0],
                to=coord_ranges[i][1],
                orient=tk.HORIZONTAL,
                length=180,  # 减小长度
                variable=var,
                resolution=1,
                showvalue=0
            )
            scale.pack(side=tk.LEFT, padx=(5, 5), fill=tk.X, expand=True)
            self.coord_scales.append(scale)
            
            # 输入框
            entry = ttk.Entry(frame, textvariable=var, width=8)  # 减小宽度
            entry.pack(side=tk.LEFT, padx=(0, 5))
            self.coord_vars.append(var)
            self.coord_entries.append(entry)
        
        # 按钮行
        button_frame = ttk.Frame(parent)
        button_frame.pack(fill=tk.X, pady=(10, 3))  # 减少上下边距
        
        self.read_coords_btn = ttk.Button(
            button_frame,
            text="读取坐标",
            command=self.read_current_coords,
            width=12,
            state="disabled"
        )
        self.read_coords_btn.pack(side=tk.LEFT, padx=(20, 60))
        
        self.send_coords_btn = ttk.Button(
            button_frame,
            text="发送坐标",
            command=self.send_coords,
            width=12,
            state="disabled"
        )
        self.send_coords_btn.pack(side=tk.LEFT)
    
    def create_compact_gripper_tab(self, parent):
        """紧凑版夹爪控制选项卡 - 压缩高度"""
        frame = ttk.Frame(parent)
        frame.pack(fill=tk.X, pady=10)  # 减少上下边距
        
        ttk.Label(frame, text="开合度:", width=6).pack(side=tk.LEFT)
        
        # 滑块（适当减小）
        self.gripper_var = tk.DoubleVar(value=50)
        self.gripper_scale = tk.Scale(
            frame,
            from_=0,
            to=100,
            orient=tk.HORIZONTAL,
            length=140,  # 减小长度
            variable=self.gripper_var,
            resolution=1,
            showvalue=0
        )
        self.gripper_scale.pack(side=tk.LEFT, padx=(2, 5), fill=tk.X, expand=True)
        
        # 输入框
        self.gripper_entry = ttk.Entry(frame, textvariable=self.gripper_var, width=8)  # 减小宽度
        self.gripper_entry.pack(side=tk.LEFT)
        
        # 发送按钮
        self.send_gripper_btn = ttk.Button(
            parent,
            text="发送夹爪角度",
            command=self.send_gripper,
            width=15,
            state="disabled"
        )
        self.send_gripper_btn.pack(pady=(10, 0))
    
    def create_compact_config_section(self, parent):
        """第五部分：机械臂串口配置"""
        frame = ttk.LabelFrame(parent, text="串口配置", padding=5)
        frame.pack(fill=tk.X, pady=(0, 5))
        
        self.save_config_btn = ttk.Button(
            frame,
            text="保存机械臂串口号",
            command=self.save_config,
            width=27,
            state="normal"
        )
        self.save_config_btn.pack()
    
    def create_compact_moveit_section(self, parent):
        """第六部分：moveit工具"""
        frame = ttk.LabelFrame(parent, text="MoveIt!工具", padding=5)
        frame.pack(fill=tk.X)
        
        self.moveit_btn = tk.Button(
            frame,
            text="启动MoveIt!工具",
            command=self.toggle_moveit,
            width=25,
            bg="lightgray",
            fg="black"
        )
        self.moveit_btn.pack()
    
    def refresh_ports(self):
        """刷新串口列表"""
        current_selection = self.port_combo.get()
        
        ports = []
        for p in serial.tools.list_ports.comports():
            if not any(f'ttyS{i}' in p.device for i in range(32)):
                ports.append(p.device)
        
        ports += [p for p in glob.glob('/dev/ttyUSB*') + glob.glob('/dev/ttyACM*') + 
                  glob.glob('/dev/aihitplt*') + glob.glob('COM*') if p not in ports]
        ports.sort()
        
        self.port_combo['values'] = ports
        
        if current_selection and current_selection in ports:
            self.port_combo.set(current_selection)
        elif ports:
            self.port_combo.current(0)
        
        self.update_status(f"找到 {len(ports)} 个串口")
    
    def toggle_connection(self):
        """切换连接状态"""
        if not self.serial_connected:
            self.connect_arm()
        else:
            self.disconnect_arm()
    
    def connect_arm(self):
        """连接机械臂"""
        port = self.port_combo.get()
        if not port:
            messagebox.showwarning("警告", "请选择串口")
            return
        
        try:
            self.arm = ultraArm(port, self.baudrate)
            time.sleep(0.5)
            angles = self.arm.get_angles_info()
            
            if angles is not None:
                self.serial_connected = True
                self.serial_port = port
                
                # 更新按钮状态
                self.connect_btn.config(text="断开")
                self.refresh_btn.config(state="disabled")
                self.init_btn.config(state="normal")
                self.open_gripper_btn.config(state="normal")
                self.close_gripper_btn.config(state="normal")
                self.init_pose_btn.config(state="normal")
                self.read_joints_btn.config(state="normal")
                self.send_joints_btn.config(state="normal")
                self.read_coords_btn.config(state="normal")
                self.send_coords_btn.config(state="normal")
                self.send_gripper_btn.config(state="normal")
                self.save_config_btn.config(state="disabled")
                self.moveit_btn.config(state="disabled")
                
                self.update_status(f"已连接到机械臂: {port}")
                self.refresh_status()
                
            else:
                messagebox.showerror("连接失败", "无法读取机械臂角度")
                if self.arm:
                    self.arm = None
                
        except Exception as e:
            messagebox.showerror("连接失败", f"无法连接机械臂:\n{e}")
            self.update_status(f"连接失败: {e}")
    
    def disconnect_arm(self):
        """断开机械臂连接"""
        if self.serial_connected:
            
            
            self.serial_connected = False
            self.arm = None
            
            # 更新按钮状态
            self.connect_btn.config(text="连接")
            self.refresh_btn.config(state="normal")
            self.init_btn.config(state="disabled")
            self.open_gripper_btn.config(state="disabled")
            self.close_gripper_btn.config(state="disabled")
            self.init_pose_btn.config(state="disabled")
            self.read_joints_btn.config(state="disabled")
            self.send_joints_btn.config(state="disabled")
            self.read_coords_btn.config(state="disabled")
            self.send_coords_btn.config(state="disabled")
            self.send_gripper_btn.config(state="disabled")
            self.save_config_btn.config(state="normal")
            self.moveit_btn.config(state="normal")
            
            # 清空状态显示
            self.clear_status_display()
            
            self.update_status("已断开机械臂连接")
    
    def clear_status_display(self):
        """清空状态显示"""
        self.angles_label.config(text="未连接")
        self.radians_label.config(text="未连接")
        self.coords_label.config(text="未连接")
    
    def init_arm(self):
        """机械臂初始化"""
        if not self.serial_connected or not self.arm:
            return
        
        def _init_in_thread():
            try:
                self.arm.go_zero()
                self.update_status("机械臂初始化完成")
                # 初始化后刷新状态
                self.root.after(100, self.refresh_status)
            except Exception as e:
                self.update_status(f"初始化失败: {e}")
        
        thread = threading.Thread(target=_init_in_thread, daemon=True)
        thread.start()
    
    def refresh_status(self):
        """刷新状态信息"""
        if not self.serial_connected or not self.arm:
            return
        
        try:
            # 获取角度
            angles = self.arm.get_angles_info()
            coords = self.arm.get_coords_info()
            
            if angles and coords:
                # 显示角度
                angles_str = " ,".join([f"{angle:>7.2f}°" for angle in angles])
                self.angles_label.config(text=angles_str)
                
                # 显示弧度
                radians = [math.radians(angle) for angle in angles]
                radians_str = " ,".join([f"{rad:7.3f}" for rad in radians])
                self.radians_label.config(text=radians_str)
                
                # 显示坐标
                coords_str = f"{coords[0]:.2f}, {coords[1]:7.2f}, {coords[2]:7.2f}, {coords[3]:7.2f}°"
                self.coords_label.config(text=coords_str)
                
                self.update_status("机械臂信息已更新")
            
        except Exception as e:
            self.update_status(f"刷新状态失败: {e}")
    
    def control_gripper(self, value):
        """控制夹爪"""
        if not self.serial_connected or not self.arm:
            return
        
        try:
            self.arm.set_gripper_state(value, 50)
            self.update_status(f"夹爪设置到: {value}")
        except Exception as e:
            self.update_status(f"控制夹爪失败: {e}")
    
    def set_init_pose(self):
        """设置初始化姿态"""
        if not self.serial_connected or not self.arm:
            return
        
        try:
            self.arm.set_angles([0, 0, 0, 0], 50)
            self.update_status("已设置归零姿态")
            # 设置后刷新状态
            self.root.after(100, self.refresh_status)
        except Exception as e:
            self.update_status(f"设置姿态失败: {e}")
    
    def read_current_joints(self):
        """读取当前关节角度"""
        if not self.serial_connected or not self.arm:
            return
        
        try:
            angles = self.arm.get_angles_info()
            if angles:
                for i, angle in enumerate(angles):
                    if i < len(self.joint_vars):
                        self.joint_vars[i].set(f"{angle:.1f}")
                self.update_status("已读取关节角度")
        except Exception as e:
            self.update_status(f"读取角度失败: {e}")
    
    def send_joints(self):
        """发送关节角度"""
        if not self.serial_connected or not self.arm:
            return
        
        try:
            degrees = []
            for var in self.joint_vars:
                degrees.append(float(var.get()))
            
            self.arm.set_angles(degrees, 50)
            self.update_status(f"已发送关节角度: {degrees}")
            # 发送后刷新状态
            self.root.after(500, self.refresh_status)
        except Exception as e:
            self.update_status(f"发送角度失败: {e}")
    
    def read_current_coords(self):
        """读取当前坐标"""
        if not self.serial_connected or not self.arm:
            return
        
        try:
            coords = self.arm.get_coords_info()
            if coords:
                for i, coord in enumerate(coords):
                    if i < len(self.coord_vars):
                        self.coord_vars[i].set(f"{coord:.1f}")
                self.update_status("已读取当前坐标")
        except Exception as e:
            self.update_status(f"读取坐标失败: {e}")
    
    def send_coords(self):
        """发送坐标"""
        if not self.serial_connected or not self.arm:
            return
        
        try:
            coords = []
            for var in self.coord_vars:
                coords.append(float(var.get()))
            
            self.arm.set_coords(coords, 50)
            self.update_status(f"已发送坐标: {coords}")
            # 发送后刷新状态
            self.root.after(500, self.refresh_status)
        except Exception as e:
            self.update_status(f"发送坐标失败: {e}")
    
    def send_gripper(self):
        """发送夹爪角度"""
        if not self.serial_connected or not self.arm:
            return
        
        try:
            value = int(self.gripper_var.get())
            self.arm.set_gripper_state(value, 50)
            self.update_status(f"已发送夹爪角度: {value}")
        except Exception as e:
            self.update_status(f"发送夹爪角度失败: {e}")
    
    def save_config(self):
        """保存配置"""
        port = self.port_combo.get()
        if not port:
            messagebox.showwarning("警告", "请先选择串口")
            return
        
        try:
            config = {
                'arm_port': port,
                'baudrate': self.baudrate,
                'last_updated': datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            }
            
            with open(self.config_file, 'w') as f:
                yaml.dump(config, f, default_flow_style=False)
            
            messagebox.showinfo("保存成功", 
                              f"串口配置已保存\n{self.config_file}")
            
            self.update_status(f"配置已保存: {port}")
            
        except Exception as e:
            messagebox.showerror("保存失败", f"保存配置失败:\n{str(e)}")
    
    def load_saved_port(self):
        """加载保存的串口"""
        if not os.path.exists(self.config_file):
            return
        
        try:
            with open(self.config_file, 'r') as f:
                config = yaml.safe_load(f)
            
            if config and 'arm_port' in config:
                saved_port = config['arm_port']
                self.port_combo.set(saved_port)
                self.update_status(f"已加载保存的串口: {saved_port}")
                
        except Exception as e:
            print(f"加载保存的串口失败: {e}")
    
    def toggle_moveit(self):
        """切换moveit工具"""
        if not self.ros_running:
            self.start_moveit()
        else:
            self.stop_moveit()
    
    def start_moveit(self):
        """启动moveit工具"""
        try:
            # 这里使用示例launch文件，请根据实际修改
            roslaunch_cmd = 'roslaunch aihitplt_arm_moveit  aihitArm_moveit.launch'
            
            # 使用gnome-terminal打开新窗口
            cmd = [
                'gnome-terminal',
                '--title=机械臂MoveIt!',
                '--geometry=80x24+200+200',
                '--',
                'bash', '-c',
                f'{roslaunch_cmd}'
            ]
            
            self.ros_process = subprocess.Popen(
                cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                start_new_session=True
            )
            
            self.ros_pid = self.ros_process.pid
            self.ros_running = True
            
            # 更新按钮状态
            self.moveit_btn.config(
                text="关闭MoveIt!工具",
                bg="green",
                fg="white"
            )
            self.connect_btn.config(state="disabled")
            self.refresh_btn.config(state="disabled")
            self.save_config_btn.config(state="disabled")
            
            self.update_status("已启动MoveIt工具")
            
        except FileNotFoundError:
            messagebox.showerror("启动失败", "未找到gnome-terminal。")
        except Exception as e:
            messagebox.showerror("启动失败", f"无法启动MoveIt工具:\n{e}")
    
    def kill_process_tree(self, pid):
        """终止进程树"""
        try:
            process = psutil.Process(pid)
            children = process.children(recursive=True)
            
            for child in children:
                try:
                    child.terminate()
                except:
                    pass
            
            gone, alive = psutil.wait_procs(children, timeout=3)
            
            for child in alive:
                try:
                    child.kill()
                except:
                    pass
            
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
    
    def stop_moveit(self):
        """停止moveit工具"""
        if self.ros_running:
            try:
                if self.ros_pid:
                    self.kill_process_tree(self.ros_pid)
                
                # 终止roscore和相关进程
                self._kill_ros_processes()
                
            except Exception as e:
                print(f"停止ROS进程时出错: {e}")
                
            finally:
                self.ros_running = False
                self.ros_process = None
                self.ros_pid = None
                
                # 恢复按钮状态
                self.moveit_btn.config(
                    text="启动MoveIt!工具",
                    bg="lightgray",
                    fg="black"
                )
                self.connect_btn.config(state="normal")
                self.refresh_btn.config(state="normal")
                self.save_config_btn.config(state="normal")
                
                self.update_status("MoveIt工具已停止")
    
    def _kill_ros_processes(self):
        """终止所有与ROS相关的进程"""
        try:
            processes_to_kill = ['aihitArm_moveit.launch']
            for proc_name in processes_to_kill:
                subprocess.run(['pkill', '-f', proc_name], 
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
        if self.serial_connected:
            self.disconnect_arm()
        
        if self.ros_running:
            self.stop_moveit()
        
        self.root.destroy()

def main():
    """主函数"""
    if 'DISPLAY' not in os.environ:
        os.environ['DISPLAY'] = ':0'
    
    root = tk.Tk()
    
    # 设置窗口位置居中
    window_width = 700
    window_height = 500
    screen_width = root.winfo_screenwidth()
    screen_height = root.winfo_screenheight()
    x = (screen_width - window_width) // 2
    y = (screen_height - window_height) // 2
    root.geometry(f"{window_width}x{window_height}+{x}+{y}")
    
    # 创建应用
    app = CompactRobotArmDebugTool(root)
    
    # 运行主循环
    root.mainloop()

if __name__ == "__main__":
    main()