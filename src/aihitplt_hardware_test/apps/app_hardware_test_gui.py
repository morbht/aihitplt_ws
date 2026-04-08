#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import tkinter as tk
from tkinter import ttk
import socket
import subprocess
import os
import rospy
from geometry_msgs.msg import Twist
import threading
import time

class AGVControlGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("硬件测试程序")
        
        # 设置窗口大小和位置
        window_width = 260
        window_height = 510
        self.root.geometry(f"{window_width}x{window_height}")
        self.root.resizable(False, False)
        
        # 设置背景色
        self.root.configure(bg='#cccccc')
        
        # 初始化按钮状态字典
        self.button_states = {}
        
        # ROS相关
        self.ros_initialized = False
        self.cmd_pub = None
        
        # 方向控制参数
        self.linear_speed = 0.2  # 线速度 m/s
        self.angular_speed = 0.5  # 角速度 rad/s
        
        # 方向按钮的持续状态
        self.direction_active = None
        self.direction_timer = None
        
        # 创建主框架
        self.main_frame = tk.Frame(root, bg='#cccccc')
        self.main_frame.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        # 创建分页控件
        self.create_tab_widget()
    
    def get_local_ip(self):
        """获取本地IP地址"""
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(("8.8.8.8", 80))
            ip_address = s.getsockname()[0]
            s.close()
            return ip_address
        except Exception:
            return "无法获取IP"
    
    def create_tab_widget(self):
        """创建分页控件"""
        # 创建Notebook（分页容器）
        self.notebook = ttk.Notebook(self.main_frame)
        self.notebook.pack(fill=tk.BOTH, expand=True)
        
        # 创建AGV分页
        self.agv_tab = tk.Frame(self.notebook, bg='#cccccc')
        self.notebook.add(self.agv_tab, text="AGV")
        
        # 创建上装分页
        self.shangzhuang_tab = tk.Frame(self.notebook, bg='#cccccc')
        self.notebook.add(self.shangzhuang_tab, text="上装")
        
        # 设置分页样式
        style = ttk.Style()
        style.configure("TNotebook", background='#cccccc')
        style.configure("TNotebook.Tab", 
                       font=("Arial", 11, "bold"),
                       padding=[20, 5])
        
        # 创建两个分页的内容
        self.create_agv_content()
        self.create_shangzhuang_content()
    
    def create_agv_content(self):
        """创建AGV分页内容"""
        content_frame = tk.Frame(self.agv_tab, bg='#f0f0f0')
        content_frame.pack(fill=tk.BOTH, expand=True)
        
        # IP地址显示
        ip_frame = tk.Frame(content_frame, bg='#f0f0f0')
        ip_frame.pack(fill=tk.X, padx=20, pady=(1, 1))
        
        ip_address = self.get_local_ip()
        ip_label = tk.Label(ip_frame, text=f"IP地址: {ip_address}", 
                           bg='#f0f0f0', font=("Arial",10, "bold"),
                           fg='#333333')
        ip_label.pack()
        
        # 主按钮容器
        buttons_container = tk.Frame(content_frame, bg='#f0f0f0')
        buttons_container.pack(fill=tk.BOTH, expand=True, padx=0)
        
        # ========== 显示可视化界面 ==========
        self.vis_button = self.create_button(
            "显示可视化界面", 
            "visualization", 
            buttons_container,
            command=self.toggle_visualization
        )
        
        # ========== AGV下位机通信 ==========
        self.comm_button = self.create_button(
            "打开下位机通信", 
            "communication", 
            buttons_container,
            command=self.toggle_communication
        )
        
        # 方向控制按钮框架
        direction_frame = tk.Frame(buttons_container, bg='#f0f0f0')
        direction_frame.pack(fill=tk.X, padx=3, pady=(5, 5))
        
        directions = [
            ("↑", "forward", "前进"),
            ("↓", "backward", "后退"),
            ("←", "left", "左转"),
            ("→", "right", "右转"),
        ]
        
        # 创建方向按钮（长按功能）
        self.direction_buttons = {}
        for text, command, description in directions:
            btn = tk.Button(direction_frame, text=text, width=3, height=1,
                          bg='#cccccc', fg='black',
                          font=("Arial", 14, "bold"),
                          state='disabled',
                          relief=tk.RAISED,
                          borderwidth=1)
            
            # 绑定鼠标按下和释放事件
            btn.bind("<ButtonPress-1>", lambda e, c=command: self.on_direction_press(c))
            btn.bind("<ButtonRelease-1>", lambda e, c=command: self.on_direction_release(c))
            btn.pack(side=tk.LEFT, padx=2, pady=0)
            
            self.direction_buttons[command] = btn
        
        # ========== 多传感器可视化显示 ==========
        self.sensor_button = self.create_button(
            "多传感器可视化显示", 
            "sensor_visualization", 
            buttons_container,
            command=self.toggle_sensor_visualization
        )
        
        # ========== 启动键盘控制 ==========
        self.keyboard_button = self.create_button(
            "启动键盘控制", 
            "keyboard_control", 
            buttons_container,
            command=self.toggle_keyboard_control
        )
        
        self.depth_cam_button = self.create_button(
            "打开深度相机",
            "depth_camera",
            buttons_container,
            command=self.toggle_depth_cam
        )

        self.lidar_button = self.create_button(
            "启动激光雷达",
            "lidar",
            buttons_container,
            command=self.toggle_lidar
        )

        self.panel_button = self.create_button(
            "旋钮屏幕测试",
            "screen_test",
            buttons_container,
            command=self.toggle_panel
        )

        self.mag_button = self.create_button(
            "磁导航传感器测试",
            "magnetic_sensor",
            buttons_container,
            command=self.toggle_mag
        )

        self.mic_button = self.create_button(
            "麦克风测试",
            "microphone_test",
            buttons_container,
            command=self.toggle_mic
        )

        self.wifi_button = self.create_button(
            "WIFI名称修改",
            "wifi_change",
            buttons_container,
            command=self.toggle_wifi
        )


        # agv_features = [
        #     ("", "")
        # ]
        
        # for text, feature in agv_features:
        #     self.create_button(
        #         text, 
        #         feature, 
        #         buttons_container,
        #         command=lambda f=feature: self.toggle_feature(f)
        #     )

        # 扩展框架
        expand_frame = tk.Frame(buttons_container, bg='#f0f0f0')
        expand_frame.pack(fill=tk.BOTH, expand=True)
    
    def create_shangzhuang_content(self):
        """创建上装分页内容"""
        content_frame = tk.Frame(self.shangzhuang_tab, bg='#f0f0f0')
        content_frame.pack(fill=tk.BOTH, expand=True)

        self.guide_deli_estop_test = self.create_button(
            "迎宾模块和送餐模块急停按钮测试",
            "emergency_stop",
            content_frame,
            command = self.toggle_guide_deli_estop_test
        )

        self.guide_deli_estop_test = self.create_button(
            "安防模块传感器系统测试",
            "security_sensors",
            content_frame,
            command = self.toggle_security_sensors
        )

        self.security_camera_test = self.create_button(
            "安防模块工业云台相机测试",
            "camera_test",
            content_frame,
            command = self.toggle_security_camera
        )

        self.spray_test = self.create_button(
            "喷雾模块硬件系统测试",
            "spray_test",
            content_frame,
            command = self.toggle_spray_test
        )

        self.AI_sensors_test = self.create_button(
            "AI套件传感器系统测试",
            "ai_sensors",
            content_frame,
            command = self.toggle_AI_sensors_test
        )

        self.AI_mic_test = self.create_button(
            "AI套件麦克风阵列测试",
            "ai_microphone",
            content_frame,
            command = self.toggle_AI_mic_test
        )

        self.industrial_sensors = self.create_button(
            "工业物流传感器系统测试",
            "industrial_sensors",
            content_frame,
            command = self.toggle_industrial_sensors
        )

        self.arm_test = self.create_button(
            "机械臂调试工具",
            "robot_arm",
            content_frame,
            command = self.toggle_arm_test
        )


        shangzhuang_features = [
            # ("迎宾模块和送餐模块急停按钮测试", "emergency_stop"),
            # ("安防模块传感器系统测试", "security_sensors"),
            # ("安防模块工业云台相机测试", "camera_test"),
            # ("喷雾模块硬件系统测试", "spray_test"),
            # ("AI套件传感器系统测试", "ai_sensors"),
            # ("AI套件麦克风阵列测试", "ai_microphone"),
            # ("工业物流传感器系统测试", "industrial_sensors"),
            ("送物模块硬件系统测试", "delivery_test"),
            # ("机械臂调试工具", "robot_arm"),
            # ("迎宾模块和机械臂USB摄像头测试", "usb_camera")
        ]
        
        for text, feature in shangzhuang_features:
            btn = tk.Button(content_frame, text=text, 
                           command=lambda f=feature: self.toggle_feature(f),
                           bg='#e0e0e0',
                           fg='black',
                           font=("Arial", 11),
                           height=0,
                           width=45,
                           wraplength=400,
                           relief=tk.RAISED,
                           borderwidth=2)
            btn.pack(fill=tk.BOTH, expand=True, padx=5, pady=3)

            
            self.button_states[feature] = {
                'button': btn,
                'active': False,
                'tab': 'shangzhuang'
            }
        self.usb_cam_test = self.create_button(
            "迎宾模块和机械臂USB摄像头测试",
            "usb_camera",
            content_frame,
            command = self.toggle_usb_camera
        )
    
    def create_button(self, text, feature, parent_frame, command):
        """创建按钮的通用方法"""
        btn = tk.Button(parent_frame, text=text,
                       command=command,
                       bg='#e0e0e0',
                       fg='black',
                       font=("Arial", 11),
                       height=0,
                       width=45,
                       wraplength=400,
                       relief=tk.RAISED,
                       borderwidth=2)
        btn.pack(fill=tk.BOTH, expand=True, padx=5, pady=2)
        
        self.button_states[feature] = {
            'button': btn,
            'active': False,
            'tab': 'agv'
        }
        
        return btn
    
    def toggle_visualization(self):
        """切换可视化界面状态"""
        feature = "visualization"
        button_info = self.button_states[feature]
        
        if button_info['active']:
            # 关闭可视化界面
            success = self.stop_ros_launch('rviz_display.launch')
            if success:
                button_info['button'].config(text="显示可视化界面", bg='#e0e0e0', fg='black')
                button_info['active'] = False
                print("关闭可视化界面")
        else:
            # 启动可视化界面
            success = self.start_ros_launch('rviz_display.launch', 'AGV可视化界面')
            if success:
                button_info['button'].config(text="关闭可视化界面", bg='#4CAF50', fg='white')
                button_info['active'] = True
                print("启动可视化界面")
    
    def toggle_communication(self):
        """切换下位机通信状态"""
        feature = "communication"
        button_info = self.button_states[feature]
        
        if button_info['active']:
            # 关闭下位机通信
            success = self.stop_ros_launch('bringup_test.launch')
            if success:
                button_info['button'].config(text="打开下位机通信", bg='#e0e0e0', fg='black')
                button_info['active'] = False
                print("关闭下位机通信")
            
            # 关闭通信后，箭头按钮恢复灰色禁用状态
            for btn in self.direction_buttons.values():
                btn.config(state='disabled', bg='#cccccc')
            
            # 停止所有方向控制
            self.stop_all_motion()
        
        else:
            # 打开下位机通信
            success = self.start_ros_launch('bringup_test.launch', 'AGV下位机通信')
            if success:
                button_info['button'].config(text="关闭下位机通信", bg='#4CAF50', fg='white')
                button_info['active'] = True
                print("打开下位机通信")
                
                # 打开通信后，初始化ROS并启用箭头按钮
                self.init_ros_node()
                for btn in self.direction_buttons.values():
                    btn.config(state='normal', bg='#2196F3', fg='white')
    
    def toggle_sensor_visualization(self):
        """切换多传感器可视化显示状态"""
        feature = "sensor_visualization"
        button_info = self.button_states[feature]
        
        if button_info['active']:
            # 关闭多传感器可视化显示
            success = self.stop_ros_launch('aihitplt_sensor_panel_gui.launch')
            if success:
                button_info['button'].config(text="多传感器可视化显示", bg='#e0e0e0', fg='black')
                button_info['active'] = False
                print("关闭多传感器可视化显示")
        else:
            # 启动多传感器可视化显示
            success = self.start_ros_launch('aihitplt_sensor_panel_gui.launch', '多传感器可视化')
            if success:
                button_info['button'].config(text="关闭传感器可视化", bg='#4CAF50', fg='white')
                button_info['active'] = True
                print("启动多传感器可视化显示")
    
    def toggle_keyboard_control(self):
        """切换键盘控制状态"""
        feature = "keyboard_control"
        button_info = self.button_states[feature]
        
        if button_info['active']:
            # 关闭键盘控制
            success = self.stop_ros_launch('aihitplt_keyboard_teleop.launch')
            if success:
                button_info['button'].config(text="启动键盘控制", bg='#e0e0e0', fg='black')
                button_info['active'] = False
                print("关闭键盘控制")
        
        else:
            # 启动键盘控制
            success = self.start_ros_launch('aihitplt_keyboard_teleop.launch', 'AGV键盘控制')
            if success:
                button_info['button'].config(text="关闭键盘控制", bg='#4CAF50', fg='white')
                button_info['active'] = True
                print("启动键盘控制")

    def toggle_depth_cam(self):
        """切换键盘控制状态"""
        feature = "depth_camera"
        button_info = self.button_states[feature]
        
        if button_info['active']:
            # 关闭键盘控制
            success = self.stop_ros_launch('aihitplt_camera.launch')
            if success:
                button_info['button'].config(text="启动深度相机", bg='#e0e0e0', fg='black')
                button_info['active'] = False
                print("关闭深度相机")
        
        else:
            # 启动键盘控制
            success = self.start_ros_launch('aihitplt_camera.launch', '深度相机')
            if success:
                button_info['button'].config(text="关闭深度相机", bg='#4CAF50', fg='white')
                button_info['active'] = True
                print("启动深度相机")

    def toggle_lidar(self):
        """切换键盘控制状态"""
        feature = "lidar"
        button_info = self.button_states[feature]
        
        if button_info['active']:
            # 关闭键盘控制
            success = self.stop_ros_launch('aihitplt_lidar.launch')
            if success:
                button_info['button'].config(text="启动激光雷达", bg='#e0e0e0', fg='black')
                button_info['active'] = False
                print("关闭激光雷达")
        
        else:
            # 启动键盘控制
            success = self.start_ros_launch('aihitplt_lidar.launch', '激光雷达')
            if success:
                button_info['button'].config(text="关闭激光雷达", bg='#4CAF50', fg='white')
                button_info['active'] = True
                print("启动激光雷达")

    def toggle_panel(self):
        """旋钮屏测试"""
        feature = "screen_test"
        button_info = self.button_states[feature]
        
        if button_info['active']:
            # 关闭键盘控制
            success = self.stop_ros_launch('aihitplt_round_panel_gui.launch')
            if success:
                button_info['button'].config(text="旋钮屏测试", bg='#e0e0e0', fg='black')
                button_info['active'] = False
                print("关闭旋钮屏测试")
        
        else:
            # 启动键盘控制
            success = self.start_ros_launch('aihitplt_round_panel_gui.launch', '旋钮屏')
            if success:
                button_info['button'].config(text="关闭旋钮屏测试", bg='#4CAF50', fg='white')
                button_info['active'] = True
                print("旋钮屏测试")


    def toggle_mag(self):
        """磁导航测试"""
        feature = "magnetic_sensor"
        button_info = self.button_states[feature]
        
        if button_info['active']:
            # 关闭键盘控制
            success = self.stop_ros_launch('aihitplt_mag_gui.launch')
            if success:
                button_info['button'].config(text="磁导航测试", bg='#e0e0e0', fg='black')
                button_info['active'] = False
                print("关闭磁导航测试")
        
        else:
            # 启动键盘控制
            success = self.start_ros_launch('aihitplt_mag_gui.launch', '磁导航')
            if success:
                button_info['button'].config(text="关闭磁导航测试", bg='#4CAF50', fg='white')
                button_info['active'] = True
                print("磁导航测试")

    def toggle_mic(self):
        """麦克风测试"""
        feature = "microphone_test"
        button_info = self.button_states[feature]
        
        if button_info['active']:
            # 关闭键盘控制
            success = self.stop_ros_launch('aihitplt_micophone_test.launch')
            if success:
                button_info['button'].config(text="麦克风测试", bg='#e0e0e0', fg='black')
                button_info['active'] = False
                print("关闭麦克风测试")
        
        else:
            # 启动键盘控制
            success = self.start_ros_launch('aihitplt_micophone_test.launch', '麦克风测试')
            if success:
                button_info['button'].config(text="关闭麦克风测试", bg='#4CAF50', fg='white')
                button_info['active'] = True
                print("麦克风测试")

    def toggle_wifi(self):
        """WIFIxiugai"""
        feature = "wifi_change"
        button_info = self.button_states[feature]
        
        if button_info['active']:
            # 关闭键盘控制
            success = self.stop_ros_launch('aihit_wifi_sh.launch')
            if success:
                button_info['button'].config(text="WIFI名称修改", bg='#e0e0e0', fg='black')
                button_info['active'] = False
                print("关闭WIFI名称修改")
        
        else:
            # 启动键盘控制
            success = self.start_ros_launch('aihit_wifi_sh.launch', 'WIFI名称修改')
            if success:
                button_info['button'].config(text="关闭WIFI名称修改", bg='#4CAF50', fg='white')
                button_info['active'] = True
                print("WIFI名称修改")

    def toggle_guide_deli_estop_test(self):
        """guide_deli_estop_test"""
        feature = "emergency_stop"
        button_info = self.button_states[feature]
        
        if button_info['active']:
            # 关闭键盘控制
            success = self.stop_ros_launch('aihitplt_guide_deli_estop_gui.launch')
            if success:
                button_info['button'].config(text="迎宾模块和送餐模块急停按钮测试", bg='#e0e0e0', fg='black')
                button_info['active'] = False
                print("关闭迎宾与送餐急停测试")
        
        else:
            # 启动键盘控制
            success = self.start_ros_launch('aihitplt_guide_deli_estop_gui.launch', '迎宾模块和送餐模块急停按钮测试')
            if success:
                button_info['button'].config(text="关闭迎宾与送餐急停测试", bg='#4CAF50', fg='white')
                button_info['active'] = True
                print("迎宾模块和送餐模块急停按钮测试")

    def toggle_security_sensors(self):
        """guide_deli_estop_test"""
        feature = "security_sensors"
        button_info = self.button_states[feature]
        
        if button_info['active']:
            # 关闭键盘控制
            success = self.stop_ros_launch('aihitplt_security_sensors_gui.launch')
            if success:
                button_info['button'].config(text="安防模块传感器测试", bg='#e0e0e0', fg='black')
                button_info['active'] = False
                print("关闭迎安防模块传感器测试")
        
        else:
            # 启动键盘控制
            success = self.start_ros_launch('aihitplt_security_sensors_gui.launch', '安防模块传感器测试')
            if success:
                button_info['button'].config(text="关闭安防模块传感器测试", bg='#4CAF50', fg='white')
                button_info['active'] = True
                print("安防模块传感器测试")

    def toggle_security_camera(self):
        """security_camera_test"""
        feature = "camera_test"
        button_info = self.button_states[feature]
        
        if button_info['active']:
            # 关闭键盘控制
            success = self.stop_ros_launch('aihitplt_pan_cam_gui.launch')
            if success:
                button_info['button'].config(text="安防模块工业云台相机测试", bg='#e0e0e0', fg='black')
                button_info['active'] = False
                print("关闭安防模块工业云台相机测试")
        
        else:
            # 启动键盘控制
            success = self.start_ros_launch('aihitplt_pan_cam_gui.launch', '安防模块工业云台相机测试')
            if success:
                button_info['button'].config(text="关闭安防模块工业云台相机测试", bg='#4CAF50', fg='white')
                button_info['active'] = True
                print("安防模块工业云台相机测试")
        
    def toggle_spray_test(self):
        """spray_test"""
        feature = "spray_test"
        button_info = self.button_states[feature]
        
        if button_info['active']:
            # 关闭键盘控制
            success = self.stop_ros_launch('aihitplt_spray_gui.launch')
            if success:
                button_info['button'].config(text="喷雾模块硬件测试", bg='#e0e0e0', fg='black')
                button_info['active'] = False
                print("关闭喷雾模块硬件测试")
        
        else:
            # 启动键盘控制
            success = self.start_ros_launch('aihitplt_spray_gui.launch', '喷雾模块硬件测试')
            if success:
                button_info['button'].config(text="关闭喷雾模块硬件测试", bg='#4CAF50', fg='white')
                button_info['active'] = True
                print("喷雾模块硬件测试")

    def toggle_AI_sensors_test(self):
        """test_test_test"""
        feature = "ai_sensors"
        button_info = self.button_states[feature]
        
        if button_info['active']:
            # 关闭键盘控制
            success = self.stop_ros_launch('aihitplt_pan_tilt_gui.launch')
            if success:
                button_info['button'].config(text="AI套件传感器系统测试", bg='#e0e0e0', fg='black')
                button_info['active'] = False
                print("关闭AI套件传感器系统测试")
        
        else:
            # 启动键盘控制
            success = self.start_ros_launch('aihitplt_pan_tilt_gui.launch', 'AI套件传感器系统测试')
            if success:
                button_info['button'].config(text="关闭AI套件传感器系统测试", bg='#4CAF50', fg='white')
                button_info['active'] = True
                print("AI套件传感器系统测试")


    def toggle_AI_mic_test(self):
        """test_test_test"""
        feature = "ai_microphone"
        button_info = self.button_states[feature]
        
        if button_info['active']:
            # 关闭键盘控制
            success = self.stop_ros_launch('aihitplt_mic_gui.launch')
            if success:
                button_info['button'].config(text="AI套件麦克风阵列测试", bg='#e0e0e0', fg='black')
                button_info['active'] = False
                print("关闭AI套件麦克风阵列测试")
        
        else:
            # 启动键盘控制
            success = self.start_ros_launch('aihitplt_mic_gui.launch', 'AI套件麦克风阵列测试')
            if success:
                button_info['button'].config(text="关闭AI套件麦克风阵列测试", bg='#4CAF50', fg='white')
                button_info['active'] = True
                print("AI套件麦克风阵列测试")

    def toggle_industrial_sensors(self):
        """test_test_test"""
        feature = "industrial_sensors"
        button_info = self.button_states[feature]
        
        if button_info['active']:
            # 关闭键盘控制
            success = self.stop_ros_launch('aihitplt_deli_sensor_gui.launch')
            if success:
                button_info['button'].config(text="工业物流传感器系统测试", bg='#e0e0e0', fg='black')
                button_info['active'] = False
                print("关闭工业物流传感器系统测试")
        
        else:
            # 启动键盘控制
            success = self.start_ros_launch('aihitplt_deli_sensor_gui.launch', '工业物流传感器系统测试')
            if success:
                button_info['button'].config(text="关闭工业物流传感器系统测试", bg='#4CAF50', fg='white')
                button_info['active'] = True
                print("工业物流传感器系统测试")

    def toggle_arm_test(self):
        """test_test_test"""
        feature = "robot_arm"
        button_info = self.button_states[feature]
        
        if button_info['active']:
            # 关闭键盘控制
            success = self.stop_ros_launch('aihitplt_arm_gui.launch')
            if success:
                button_info['button'].config(text="机械臂调试工具", bg='#e0e0e0', fg='black')
                button_info['active'] = False
                print("关闭机械臂调试工具")
        
        else:
            # 启动键盘控制
            success = self.start_ros_launch('aihitplt_arm_gui.launch', '机械臂调试工具')
            if success:
                button_info['button'].config(text="关闭机械臂调试工具", bg='#4CAF50', fg='white')
                button_info['active'] = True
                print("机械臂调试工具")

    def toggle_usb_camera(self):
        """test_test_test"""
        feature = "usb_camera"
        button_info = self.button_states[feature]
        
        if button_info['active']:
            # 关闭键盘控制
            success = self.stop_ros_launch('usb_cam-test.launch')
            if success:
                button_info['button'].config(text="迎宾模块和机械臂USB摄像头测试", bg='#e0e0e0', fg='black')
                button_info['active'] = False
                print("关闭USB摄像头测试")
        
        else:
            # 启动键盘控制
            success = self.start_ros_launch('usb_cam-test.launch', '迎宾模块和机械臂USB摄像头测试')
            if success:
                button_info['button'].config(text="关闭USB摄像头测试", bg='#4CAF50', fg='white')
                button_info['active'] = True
                print("迎宾模块和机械臂USB摄像头测试")
                

    
    def start_ros_launch(self, launch_file, title):
        """启动ROS launch文件"""
        try:
            # 使用gnome-terminal打开新终端运行ROS命令
            cmd = [
                'gnome-terminal',
                f'--title={title}',
                '--',
                'bash', '-c',
                'source ~/.bashrc && '
                f'roslaunch aihitplt_hardware_test {launch_file}; '
                'read'
            ]
            
            subprocess.Popen(cmd)
            print(f"启动{title}: {launch_file}")
            return True
            
        except Exception as e:
            print(f"启动{title}失败: {e}")
            return False
    
    def stop_ros_launch(self, launch_file):
        """停止ROS launch进程"""
        try:
            # 停止对应的launch进程
            subprocess.run(['pkill', '-f', launch_file], 
                         stdout=subprocess.DEVNULL, 
                         stderr=subprocess.DEVNULL)
            print(f"停止{launch_file}")
            return True
            
        except Exception as e:
            print(f"停止{launch_file}失败: {e}")
            return False
    
    def init_ros_node(self):
        """初始化ROS节点（仅当下位机通信打开时初始化）"""
        if not self.button_states['communication']['active']:
            return
        
        def ros_init():
            try:
                if not rospy.is_shutdown():
                    rospy.init_node('agv_gui_control', anonymous=True, disable_signals=True)
                    self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
                    time.sleep(0.5)  # 等待发布者建立连接
                    self.ros_initialized = True
                    print("ROS节点初始化成功，准备发送控制命令")
                else:
                    print("ROS Master已关闭")
            except rospy.ROSException as e:
                print(f"ROS节点初始化失败: {e}")
            except Exception as e:
                print(f"初始化失败: {e}")
        
        # 在后台线程中初始化ROS
        ros_thread = threading.Thread(target=ros_init, daemon=True)
        ros_thread.start()
    
    def publish_twist(self, linear_x=0.0, angular_z=0.0):
        """发布Twist消息到/cmd_vel话题"""
        if not self.button_states['communication']['active']:
            print("下位机通信未开启，无法发送命令")
            return False
        
        if not hasattr(self, 'cmd_pub') or self.cmd_pub is None:
            print("ROS发布器未初始化")
            return False
        
        try:
            twist = Twist()
            twist.linear.x = linear_x
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = angular_z
            
            self.cmd_pub.publish(twist)
            print(f"发布命令: linear.x={linear_x:.2f}, angular.z={angular_z:.2f}")
            return True
            
        except Exception as e:
            print(f"发布Twist消息失败: {e}")
            return False
    
    def on_direction_press(self, direction):
        """方向按钮按下事件"""
        if not self.button_states['communication']['active']:
            return  # 如果通信未开启，不响应
        
        self.direction_active = direction
        self.direction_buttons[direction].config(bg='#FF9800')  # 按下时变为橙色
        
        # 开始持续发送命令
        self.start_motion(direction)
        
        print(f"开始持续发送{direction}命令")
    
    def on_direction_release(self, direction):
        """方向按钮释放事件"""
        if not self.button_states['communication']['active']:
            return
        
        # 恢复按钮颜色（按下时为橙色，释放后恢复蓝色）
        if self.direction_active == direction:
            self.direction_buttons[direction].config(bg='#2196F3')
        
        # 停止运动
        self.stop_motion()
        
        print(f"停止发送{direction}命令")
    
    def start_motion(self, direction):
        """开始运动 - 持续发送命令"""
        # 根据方向设置运动参数
        direction_params = {
            'forward': (self.linear_speed, 0.0),      # 前进
            'backward': (-self.linear_speed, 0.0),    # 后退
            'left': (0.0, self.angular_speed),        # 左转
            'right': (0.0, -self.angular_speed)       # 右转
        }
        
        if direction in direction_params:
            linear_x, angular_z = direction_params[direction]
            
            # 发布第一次命令
            self.publish_twist(linear_x, angular_z)
            
            # 设置持续发布定时器（100ms间隔）
            self.direction_timer = self.root.after(100, 
                lambda lx=linear_x, az=angular_z: self.continuous_publish(lx, az))
    
    def continuous_publish(self, linear_x, angular_z):
        """持续发布运动命令"""
        if self.direction_active is None:
            return  # 如果方向已经释放，停止发送
        
        # 持续发布命令
        self.publish_twist(linear_x, angular_z)
        
        # 设置下一次发布（100ms后）
        self.direction_timer = self.root.after(100, 
            lambda lx=linear_x, az=angular_z: self.continuous_publish(lx, az))
    
    def stop_motion(self):
        """停止运动 - 发送一次停止命令"""
        self.direction_active = None
        
        # 取消定时器
        if self.direction_timer:
            self.root.after_cancel(self.direction_timer)
            self.direction_timer = None
        
        # 发送一次停止命令
        self.publish_twist(0.0, 0.0)
        print("发送停止命令")
    
    def stop_all_motion(self):
        """停止所有运动"""
        self.stop_motion()
        
        # 恢复所有方向按钮的颜色
        for btn in self.direction_buttons.values():
            if self.button_states['communication']['active']:
                btn.config(bg='#2196F3')
            else:
                btn.config(bg='#cccccc')
    
    def toggle_feature(self, feature):
        """切换其他功能状态"""
        if feature in self.button_states:
            button_info = self.button_states[feature]
            
            if button_info['active']:
                # 关闭功能
                button_info['button'].config(bg='#e0e0e0', fg='black')
                button_info['active'] = False
                print(f"关闭 {feature} 功能")
                
            else:
                # 启动功能
                button_info['button'].config(bg='#4CAF50', fg='white')
                button_info['active'] = True
                print(f"启动 {feature} 功能")

def main():
    """主函数"""
    if 'DISPLAY' not in os.environ:
        os.environ['DISPLAY'] = ':0'
    
    root = tk.Tk()
    app = AGVControlGUI(root)
    root.mainloop()

if __name__ == "__main__":
    main()