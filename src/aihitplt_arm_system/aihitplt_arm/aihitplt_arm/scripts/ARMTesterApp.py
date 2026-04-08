#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import tkinter as tk
from tkinter import ttk, scrolledtext, messagebox
import serial.tools.list_ports
import math
from pymycobot.ultraArm import ultraArm
import threading
import time
import cv2
from PIL import Image, ImageTk
from tkinter import PhotoImage
import os
import sys


class SmartSortingPlatform:
    def __init__(self, root):
        self.root = root
        self.root.title("智能分拣实训平台测试上位机")
        
        # 获取当前 .py 文件的目录
        if getattr(sys, 'frozen', False):
            # 打包后的路径
            script_dir = os.path.dirname(sys.executable)
        else:
            # 正常脚本运行的路径
            script_dir = os.path.dirname(os.path.abspath(__file__))
        ico_path = os.path.join(script_dir, "ARMTesterApp.png")
        icon = PhotoImage(file=ico_path)  # 支持 PNG、GIF
        self.root.iconphoto(False, icon)

        # 窗口大小
        win_width = 800
        win_height = 780

        # 获取屏幕尺寸
        screen_width = self.root.winfo_screenwidth()
        screen_height = self.root.winfo_screenheight()

        # 计算窗口左上角坐标，让窗口水平居中，垂直偏上约三分之二屏幕高度
        x = (screen_width - win_width) // 2
        y = (screen_height - win_height) * 1 // 3  # 偏上三分之二的位置

        # 设置窗口大小和位置
        self.root.geometry(f"{win_width}x{win_height}+{x}+{y}")

        # 初始化机械臂变量
        self.ua = None
        self.serial_ports = []
        self.test_running = False
        self.test_thread = None
        self.cap = None
        self.camera_thread = None
        self.camera_running = False
        
        # 创建左右主框架
        self.left_frame = ttk.Frame(root, padding="10")
        self.left_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        self.right_frame = ttk.Frame(root, padding="10")
        self.right_frame.grid(row=0, column=1, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # 1. 机械臂连接部分
        self.create_connection_frame()
        
        # 2. 机械臂信息显示部分
        self.create_info_frame()
        
        # 3. 夹爪控制部分
        self.create_gripper_frame()
        
        # 4. 机械臂测试部分
        self.create_test_frame()
        
        # 5. 机械臂调试部分
        self.create_debug_frame()
        
        # 6. 相机部分
        self.create_camera_frame()
        
        # 7. 日志部分
        self.create_log_frame()
        
        # 更新串口列表
        self.update_serial_ports()
        
    def create_connection_frame(self):
        """创建机械臂连接部分"""
        frame = ttk.LabelFrame(self.left_frame, text="机械臂连接", padding="10")
        frame.grid(row=0, column=0, sticky=(tk.W, tk.E), pady=5)
        
        # 串口选择
        ttk.Label(frame, text="串口:").grid(row=0, column=0, sticky=tk.W)
        self.port_combobox = ttk.Combobox(frame, values=self.serial_ports, width=15)
        self.port_combobox.grid(row=0, column=1, sticky=tk.W)
        
        # 刷新按钮
        refresh_btn = ttk.Button(frame, text="刷新串口", command=self.update_serial_ports)
        refresh_btn.grid(row=0, column=2, padx=5)
        
        # 连接按钮
        self.connect_btn = ttk.Button(frame, text="连接", command=self.toggle_arm_connection)
        self.connect_btn.grid(row=1, column=0, columnspan=3, pady=5, sticky=(tk.W, tk.E))
        
        # 初始化按钮
        self.init_btn = ttk.Button(frame, text="机械臂初始化", command=self.init_arm, state=tk.DISABLED)
        self.init_btn.grid(row=2, column=0, columnspan=3, sticky=(tk.W, tk.E))
    
    def create_info_frame(self):
        """创建机械臂信息显示部分"""
        frame = ttk.LabelFrame(self.left_frame, text="机械臂状态信息", padding="10")
        frame.grid(row=1, column=0, sticky=(tk.W, tk.E), pady=5)
        
        # 角度信息
        ttk.Label(frame, text="关节角度(deg):").grid(row=0, column=0, sticky=tk.W)
        self.angles_label = ttk.Label(frame, text="未连接", width=30, anchor=tk.W)
        self.angles_label.grid(row=0, column=1, sticky=tk.W)
        
        ttk.Label(frame, text="关节弧度(rad):").grid(row=1, column=0, sticky=tk.W)
        self.radians_label = ttk.Label(frame, text="未连接", width=30, anchor=tk.W)
        self.radians_label.grid(row=1, column=1, sticky=tk.W)
        
        # 坐标信息
        ttk.Label(frame, text="坐标(mm):").grid(row=2, column=0, sticky=tk.W)
        self.coords_label = ttk.Label(frame, text="未连接", width=30, anchor=tk.W)
        self.coords_label.grid(row=2, column=1, sticky=tk.W)
        
        # 刷新按钮
        refresh_btn = ttk.Button(frame, text="刷新信息", command=self.update_arm_info)
        refresh_btn.grid(row=3, column=0, columnspan=2, pady=5, sticky=(tk.W, tk.E))
    
    def create_gripper_frame(self):
        """创建夹爪控制部分 - 保持原布局只居中按钮"""
        frame = ttk.LabelFrame(self.left_frame, text="基础控制", padding="10")
        frame.grid(row=2, column=0, sticky=(tk.W, tk.E), pady=5)
        
        # 配置列的权重使内容居中
        frame.columnconfigure(0, weight=1)  # 第一列
        frame.columnconfigure(1, weight=1)  # 第二列
        
        # 打开夹爪按钮 - 保持原位置但居中
        open_btn = ttk.Button(frame, text="打开夹爪", command=self.open_gripper)
        open_btn.grid(row=0, column=0, padx=5, sticky="ew")
        
        # 关闭夹爪按钮 - 保持原位置但居中
        close_btn = ttk.Button(frame, text="关闭夹爪", command=self.close_gripper)
        close_btn.grid(row=0, column=1, padx=5, sticky="ew")
        
        # 初始化姿态按钮 - 保持跨两列但居中
        init_pose_btn = ttk.Button(frame, text="初始化姿态", command=self.init_pose)
        init_pose_btn.grid(row=1, column=0, columnspan=2, pady=5, sticky="ew")
        
        # 在两侧添加空白列实现整体居中效果
        frame.columnconfigure(0, weight=1, uniform="gripper_col")
        frame.columnconfigure(1, weight=1, uniform="gripper_col")
    
    def create_test_frame(self):
        """创建机械臂测试部分 - 仅水平居中"""
        frame = ttk.LabelFrame(self.left_frame, text="机械臂测试", padding="10")
        frame.grid(row=3, column=0, sticky=(tk.W, tk.E), pady=5)
        
        # 配置列权重实现水平居中
        frame.columnconfigure(0, weight=1)
        
        # 测试按钮 - 水平居中
        self.test_btn = ttk.Button(frame, text="开始测试", command=self.toggle_test)
        self.test_btn.grid(row=0, column=0, pady=(0,5), sticky="ew")
        
        # 测试状态标签 - 水平居中
        self.test_status = ttk.Label(frame, text="测试未运行")
        self.test_status.grid(row=1, column=0)
    
    def create_debug_frame(self):
        """创建机械臂调试部分"""
        frame = ttk.LabelFrame(self.right_frame, text="机械臂调试", padding="10")
        frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S), pady=5)
        
        # 配置主框架权重
        self.right_frame.columnconfigure(0, weight=1)
        self.right_frame.rowconfigure(0, weight=1)
        # 创建笔记本部件
        notebook = ttk.Notebook(frame)
        notebook.pack(fill=tk.BOTH, expand=True)
        
        # 1. 关节角度控制
        angle_frame = ttk.Frame(notebook, padding="5")
        self.create_angle_control(angle_frame)
        notebook.add(angle_frame, text="关节角度控制")
        
        # 2. 坐标控制
        coord_frame = ttk.Frame(notebook, padding="5")
        self.create_coord_control(coord_frame)
        notebook.add(coord_frame, text="坐标控制")
        
        # 3. 夹爪控制
        gripper_frame = ttk.Frame(notebook, padding="5")
        self.create_gripper_control(gripper_frame)
        notebook.add(gripper_frame, text="夹爪控制")
    
    def create_angle_control(self, parent):
        """创建关节角度控制部分"""
        # 必须先配置父容器的列权重
        parent.columnconfigure(2, weight=1)  # 滑块所在列
        # 关节角度范围
        angle_ranges = [
            (-150, 170),   # J1
            (-20, 90),     # J2
            (-5, 110),     # J3
            (-179, 179)    # J4
        ]
        
        self.angle_vars = []
        self.angle_entries = []
        self.angle_scales = []
        
        for i in range(4):
            # 标签
            ttk.Label(parent, text=f"关节 {i+1}:").grid(row=i, column=0, sticky=tk.W)
            
            # 输入框
            var = tk.DoubleVar(value=0)
            entry = ttk.Entry(parent, textvariable=var, width=8)
            entry.grid(row=i, column=1, padx=5)
            self.angle_entries.append(entry)
            self.angle_vars.append(var)
            
            # 滑块
            scale = ttk.Scale(
                parent, 
                from_=angle_ranges[i][0], 
                to=angle_ranges[i][1], 
                variable=var,
                command=lambda v, idx=i: self.update_angle_entry(v, idx)
            )
            scale.grid(row=i, column=2, sticky=(tk.W, tk.E), padx=5)
            self.angle_scales.append(scale)
        
        # 按钮框架
        btn_frame = ttk.Frame(parent)
        btn_frame.grid(row=4, column=0, columnspan=3, pady=5, sticky=(tk.W, tk.E))
        
        # 读取当前角度按钮
        read_btn = ttk.Button(btn_frame, text="读取当前角度", command=self.read_current_angles)
        read_btn.pack(side=tk.LEFT, expand=True, padx=5)
        
        # 发送角度按钮
        send_btn = ttk.Button(btn_frame, text="发送角度", command=self.send_angles)
        send_btn.pack(side=tk.LEFT, expand=True, padx=5)
    
    def create_coord_control(self, parent):
        """创建坐标控制部分"""
        # 必须先配置父容器的列权重
        parent.columnconfigure(2, weight=1)  # 滑块所在列
        # 坐标范围
        coord_ranges = [
            (-360, 365.55),    # X
            (-365.55, 365.55),  # Y
            (-140, 130),       # Z
            (-179, 179)        # θ
        ]
        
        self.coord_vars = []
        self.coord_entries = []
        self.coord_scales = []
        
        labels = ["X:", "Y:", "Z:", "θ:"]
        
        # 设置默认值
        default_values = [235.55, 0, 130.0, 0]
        
        for i in range(4):
            # 标签
            ttk.Label(parent, text=labels[i]).grid(row=i, column=0, sticky=tk.W)
            
            # 输入框
            var = tk.DoubleVar(value=default_values[i])
            entry = ttk.Entry(parent, textvariable=var, width=8)
            entry.grid(row=i, column=1, padx=5)
            self.coord_entries.append(entry)
            self.coord_vars.append(var)
            
            # 滑块
            scale = ttk.Scale(
                parent, 
                from_=coord_ranges[i][0], 
                to=coord_ranges[i][1], 
                variable=var,
                command=lambda v, idx=i: self.update_coord_entry(v, idx)
            )
            scale.grid(row=i, column=2, sticky=(tk.W, tk.E), padx=5)
            self.coord_scales.append(scale)
        
        # 按钮框架
        btn_frame = ttk.Frame(parent)
        btn_frame.grid(row=4, column=0, columnspan=3, pady=5, sticky=(tk.W, tk.E))
        
        # 读取当前坐标按钮
        read_btn = ttk.Button(btn_frame, text="读取当前坐标", command=self.read_current_coords)
        read_btn.pack(side=tk.LEFT, expand=True, padx=5)
        
        # 发送坐标按钮
        send_btn = ttk.Button(btn_frame, text="发送坐标", command=self.send_coords)
        send_btn.pack(side=tk.LEFT, expand=True, padx=5)
    
    def create_gripper_control(self, parent):
        """创建夹爪控制部分"""
        # 配置列权重
        parent.columnconfigure(2, weight=1)  # 滑块所在列
        # 夹爪范围 (0-100)
        ttk.Label(parent, text="夹爪开合度:").grid(row=0, column=0, sticky=tk.W)
        
        self.gripper_var = tk.IntVar(value=0)
        
        # 输入框
        entry = ttk.Entry(parent, textvariable=self.gripper_var, width=8)
        entry.grid(row=0, column=1, padx=5)
        
        # 滑块
        scale = ttk.Scale(
            parent, 
            from_=0, 
            to=100, 
            variable=self.gripper_var,
            command=lambda v: self.update_gripper_entry(v)
        )
        scale.grid(row=0, column=2, sticky=(tk.W, tk.E), padx=5)
        
        # 发送按钮
        send_btn = ttk.Button(parent, text="发送", command=self.send_gripper)
        send_btn.grid(row=1, column=0, columnspan=3, pady=5, sticky=(tk.W, tk.E))
    
    def create_camera_frame(self):
        """创建相机部分"""
        frame = ttk.LabelFrame(self.right_frame, text="视觉检测", padding="10")
        frame.grid(row=1, column=0, sticky=(tk.W, tk.E, tk.N, tk.S), pady=5)
        
        # 相机选择
        ttk.Label(frame, text="相机设备:").grid(row=0, column=0, sticky=tk.W)
        self.camera_combobox = ttk.Combobox(frame, values=self.get_camera_devices(), width=15)
        self.camera_combobox.grid(row=0, column=1, sticky=tk.W)
        
        # 刷新按钮
        refresh_btn = ttk.Button(frame, text="刷新设备", command=self.update_camera_devices)
        refresh_btn.grid(row=0, column=2, padx=5)
        
        # 打开按钮
        self.camera_btn = ttk.Button(frame, text="打开相机", command=self.toggle_camera)
        self.camera_btn.grid(row=0, column=3, padx=5)
        
        # 相机画面 - 使用自适应Canvas
        self.camera_canvas = tk.Canvas(frame, bg='white')
        self.camera_canvas.grid(row=1, column=0, columnspan=4, pady=5, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # 添加提示文本
        self.camera_canvas.create_text(self.camera_canvas.winfo_reqwidth()//2, 
                                    self.camera_canvas.winfo_reqheight()//2, 
                                    text="相机未开启", fill="black", tags="camera_text")
        
        # 设置frame的行列权重
        frame.rowconfigure(1, weight=1)
        frame.columnconfigure(0, weight=1)
    
    def create_log_frame(self):
        """创建日志部分"""
        frame = ttk.LabelFrame(self.right_frame, text="系统日志", padding="10")
        frame.grid(row=2, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # 日志文本框
        self.log_text = scrolledtext.ScrolledText(frame, width=15, height=7)
        self.log_text.pack(fill=tk.BOTH, expand=True)
        
        # 添加初始日志
        self.log("智能分拣实训平台测试上位机已启动")
    
    # 辅助方法
    def update_angle_entry(self, value, idx):
        """更新角度输入框"""
        self.angle_vars[idx].set(round(float(value), 2))
    
    def update_coord_entry(self, value, idx):
        """更新坐标输入框"""
        self.coord_vars[idx].set(round(float(value), 2))
    
    def update_gripper_entry(self, value):
        """更新夹爪输入框"""
        self.gripper_var.set(round(float(value)))
    
    def log(self, message):
        """添加日志"""
        self.log_text.insert(tk.END, f"{message}\n")
        self.log_text.see(tk.END)
    
    # 串口相关方法
    def update_serial_ports(self):
        """更新串口列表"""
        self.serial_ports = [port.device for port in serial.tools.list_ports.comports()]
        self.port_combobox['values'] = self.serial_ports
        if self.serial_ports:
            self.port_combobox.set(self.serial_ports[0])
        self.log("串口列表已更新")
    
    def toggle_arm_connection(self):
        """切换机械臂连接状态"""
        if self.ua is None:
            # 连接机械臂
            port = self.port_combobox.get()
            if not port:
                self.log("错误: 请选择串口")
                return
            
            try:
                self.ua = ultraArm(port, 115200)
                self.log(f"已连接到机械臂, 串口: {port}")
                self.connect_btn.config(text="断开连接")
                self.init_btn.config(state=tk.NORMAL)
                self.update_arm_info()
            except Exception as e:
                self.log(f"连接错误: {str(e)}")
                self.ua = None
        else:
            # 断开连接
            try:
                if self.test_running:
                    self.stop_test()
                
                if self.camera_running:
                    self.toggle_camera()
                
                self.ua = None
                self.connect_btn.config(text="连接")
                self.init_btn.config(state=tk.DISABLED)
                self.angles_label.config(text="未连接")
                self.radians_label.config(text="未连接")
                self.coords_label.config(text="未连接")
                self.log("已断开机械臂连接")
            except Exception as e:
                self.log(f"断开连接错误: {str(e)}")
    
    def init_arm(self):
        """初始化机械臂"""
        if self.ua is None:
            self.log("错误: 机械臂未连接")
            return
        
        # 禁用按钮防止重复点击
        self.init_btn.config(state=tk.DISABLED)
        self.connect_btn.config(state=tk.DISABLED)
        
        # 在新线程中执行初始化
        def init_thread():
            try:
                self.log("机械臂初始化中...")
                self.ua.go_zero()
                self.log("机械臂已初始化")
                self.update_arm_info()
            except Exception as e:
                self.log(f"初始化错误: {str(e)}")
            finally:
                # 恢复按钮状态
                self.init_btn.config(state=tk.NORMAL)
                self.connect_btn.config(state=tk.NORMAL)
        
        threading.Thread(target=init_thread, daemon=True).start()
    
    # 机械臂信息相关方法
    def update_arm_info(self):
        """更新机械臂信息"""
        if self.ua is None:
            return
        
        try:
            angles = self.ua.get_angles_info()
            coords = self.ua.get_coords_info()
            
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
                
                self.log("机械臂信息已更新")
        except Exception as e:
            self.log(f"获取信息错误: {str(e)}")
    
    # 夹爪控制方法
    def open_gripper(self):
        """打开夹爪"""
        if self.ua is None:
            self.log("错误: 机械臂未连接")
            return
        
        try:
            self.ua.set_gripper_state(100, 50)
            self.log("夹爪已打开")
            self.gripper_var.set(100)
        except Exception as e:
            self.log(f"打开夹爪错误: {str(e)}")
    
    def close_gripper(self):
        """关闭夹爪"""
        if self.ua is None:
            self.log("错误: 机械臂未连接")
            return
        
        try:
            self.ua.set_gripper_state(0, 50)
            self.log("夹爪已关闭")
            self.gripper_var.set(0)
        except Exception as e:
            self.log(f"关闭夹爪错误: {str(e)}")
    
    def init_pose(self):
        """初始化姿态"""
        if self.ua is None:
            self.log("错误: 机械臂未连接")
            return
        
        try:
            self.ua.set_gripper_state(100,50)
            self.ua.set_angles([0, 0, 0, 0], 50)
            time.sleep(0.5)
            self.log("机械臂已回到初始姿态")
            self.update_arm_info()
        except Exception as e:
            self.log(f"初始化姿态错误: {str(e)}")
    
    # 机械臂测试方法
    def toggle_test(self):
        """切换测试状态"""
        if self.ua is None:
            self.log("错误: 机械臂未连接")
            return
        
        if not self.test_running:
            self.start_test()
        else:
            self.stop_test()
    
    def start_test(self):
        """开始测试"""
        self.test_running = True
        self.test_btn.config(text="停止测试")
        self.test_status.config(text="测试运行中...")
        self.log("测试开始")
        
        # 在新线程中运行测试
        self.test_thread = threading.Thread(target=self.run_test, daemon=True)
        self.test_thread.start()
    
    def stop_test(self):
        """停止测试"""
        self.test_running = False
        if self.test_thread and self.test_thread.is_alive():
            self.test_thread.join(timeout=1)
        self.test_btn.config(text="开始测试")
        self.test_status.config(text="测试已停止")
        self.log("测试停止")
    
    def run_test(self):
        """运行测试"""
        while self.test_running and self.ua:
            try:
                # 检查是否停止
                if not self.test_running:
                    break
                    
                # self.ua.set_gripper_state(100, 50)
                self.ua.set_angles([0, -20, 0, 60], 100)
                
                time.sleep(2)
                # # 检查是否停止
                # if not self.test_running:
                #     break
                if not self.test_running: break
                self.ua.set_angles([40, -20, 10, -60], 100)
                time.sleep(2)
                if not self.test_running: break
                self.ua.set_gripper_state(0, 50)
                time.sleep(1.5)
                if not self.test_running: break
                self.ua.set_gripper_state(100, 50)
                time.sleep(1)
                # 检查是否停止
                if not self.test_running:
                    break
                
                self.update_arm_info()
            except Exception as e:
                self.log(f"测试错误: {str(e)}")
                break
    
    # 调试控制方法
    def read_current_angles(self):
        """读取当前角度"""
        if self.ua is None:
            self.log("错误: 机械臂未连接")
            return
        
        try:
            angles = self.ua.get_angles_info()
            if angles:
                for i in range(4):
                    self.angle_vars[i].set(round(angles[i], 2))
                self.log("当前角度已读取")
        except Exception as e:
            self.log(f"读取角度错误: {str(e)}")
    
    def send_angles(self):
        """发送角度"""
        if self.ua is None:
            self.log("错误: 机械臂未连接")
            return
        
        try:
            angles = [self.angle_vars[i].get() for i in range(4)]
            self.ua.set_angles(angles, 50)
            time.sleep(0.1)
            self.log(f"角度已发送: {angles}")
            self.update_arm_info()
        except Exception as e:
            self.log(f"发送角度错误: {str(e)}")
    
    def read_current_coords(self):
        """读取当前坐标"""
        if self.ua is None:
            self.log("错误: 机械臂未连接")
            return
        
        try:
            coords = self.ua.get_coords_info()
            if coords:
                for i in range(4):
                    self.coord_vars[i].set(round(coords[i], 2))
                self.log("当前坐标已读取")
        except Exception as e:
            self.log(f"读取坐标错误: {str(e)}")
    
    def send_coords(self):
        """发送坐标"""
        if self.ua is None:
            self.log("错误: 机械臂未连接")
            return
        
        try:
            coords = [self.coord_vars[i].get() for i in range(4)]
            self.ua.set_coords(coords, 50)
            time.sleep(0.1)
            self.log(f"坐标已发送: {coords}")
            self.update_arm_info()
        except Exception as e:
            self.log(f"发送坐标错误: {str(e)}")
    
    def send_gripper(self):
        """发送夹爪控制"""
        if self.ua is None:
            self.log("错误: 机械臂未连接")
            return
        
        try:
            degree = self.gripper_var.get()
            self.ua.set_gripper_state(degree, 50)
            time.sleep(0.1)
            self.log(f"夹爪已设置为: {degree}%")
        except Exception as e:
            self.log(f"设置夹爪错误: {str(e)}")
    
    # 相机相关方法
    def get_camera_devices(self):
        """获取可用相机设备"""
        devices = []
        for i in range(5):  # 检查前5个设备
            try:
                cap = cv2.VideoCapture(i)
                if cap.isOpened():
                    devices.append(f"相机 {i}")
                    cap.release()
            except:
                continue
        return devices if devices else ["无可用相机"]
    
    def update_camera_devices(self):
        """更新相机设备列表"""
        devices = self.get_camera_devices()
        self.camera_combobox['values'] = devices
        if devices:
            self.camera_combobox.set(devices[0])
        self.log("相机设备列表已更新")
    
    def toggle_camera(self):
        """切换相机状态（修复闪烁和花屏问题）"""
        if self.cap is None:
            # === 打开相机 ===
            device_str = self.camera_combobox.get()
            if not device_str.startswith("相机"):
                self.log("错误: 请选择有效的相机设备")
                return
            
            device_idx = int(device_str.split()[-1])
            
            try:
                # 使用DSHOW后端并设置合适的缓冲大小
                self.cap = cv2.VideoCapture(device_idx, cv2.CAP_DSHOW)
                self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # 减少缓冲区
                
                if not self.cap.isOpened():
                    self.log(f"错误: 无法打开相机 {device_idx}")
                    self.cap = None
                    return
                
                # 先清空画布再启动线程
                self.camera_canvas.delete("all")
                self.camera_running = True
                self.camera_btn.config(text="关闭相机")
                self.log(f"相机 {device_idx} 已打开")
                
                # 启动相机线程（增加帧率控制）
                self.camera_thread = threading.Thread(
                    target=self.update_camera, 
                    daemon=True
                )
                self.camera_thread.start()
                
            except Exception as e:
                self.log(f"打开相机错误: {str(e)}")
                if self.cap:
                    self.cap.release()
                    self.cap = None
        else:
            # === 关闭相机 ===
            self.camera_running = False  # 先标记停止
            
            # 等待线程结束（最多1秒）
            if self.camera_thread and self.camera_thread.is_alive():
                self.camera_thread.join(timeout=1.0)
            
            # 先释放相机资源再更新界面
            if self.cap:
                self.cap.release()
                self.cap = None
            
            # 清空画布并显示关闭状态
            self.camera_canvas.delete("all")
            self.camera_canvas.create_text(
                self.camera_canvas.winfo_width()//2,
                self.camera_canvas.winfo_height()//2,
                text="相机已关闭", 
                fill="white", 
                tags="camera_text"
            )
            self.camera_btn.config(text="打开相机")
            self.log("相机已关闭")

    def update_camera(self):
        """更新相机画面（修复闪烁问题）"""
        last_frame_time = time.time()
        
        while self.camera_running and self.cap is not None:
            try:
                # 控制帧率（30FPS）
                if (time.time() - last_frame_time) < 1/15:
                    time.sleep(0.005)
                    continue
                    
                # 清空缓冲区（解决延迟）
                for _ in range(2):
                    self.cap.grab()
                    
                ret, frame = self.cap.read()
                last_frame_time = time.time()
                
                if ret:
                    canvas_width = self.camera_canvas.winfo_width()
                    canvas_height = self.camera_canvas.winfo_height()
                    
                    if canvas_width > 0 and canvas_height > 0:
                        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                        
                        # 保持宽高比的缩放
                        h, w = frame.shape[:2]
                        ratio = min(canvas_width/w, canvas_height/h)
                        new_size = (int(w*ratio), int(h*ratio))
                        frame = cv2.resize(frame, new_size)
                        
                        # 居中显示
                        x_offset = (canvas_width - new_size[0]) // 2
                        y_offset = (canvas_height - new_size[1]) // 2
                        
                        img = Image.fromarray(frame)
                        img_tk = ImageTk.PhotoImage(image=img)
                        
                        # 原子化更新
                        self.camera_canvas.delete("all")
                        self.camera_canvas.create_image(
                            x_offset, y_offset, 
                            anchor=tk.NW, 
                            image=img_tk
                        )
                        self.camera_canvas.image = img_tk
                        
            except Exception as e:
                if self.camera_running:  # 只记录非主动停止的错误
                    self.log(f"相机更新错误: {str(e)}")
                break
    
    def on_closing(self):
        """关闭窗口时的清理工作"""
        # 停止测试
        if self.test_running:
            self.test_running = False
            if self.test_thread and self.test_thread.is_alive():
                self.test_thread.join(timeout=1)
        
        # 关闭相机
        if self.camera_running:
            self.camera_running = False
            if self.cap:
                self.cap.release()
            if self.camera_thread and self.camera_thread.is_alive():
                self.camera_thread.join(timeout=1)
        
        self.root.destroy()

if __name__ == "__main__":
    root = tk.Tk()
    app = SmartSortingPlatform(root)
    
    # 设置窗口关闭时的处理
    root.protocol("WM_DELETE_WINDOW", app.on_closing)
    
    # 设置窗口大小和位置
    # root.geometry("1000x800")
    root.resizable(True, True)
    
    root.mainloop()