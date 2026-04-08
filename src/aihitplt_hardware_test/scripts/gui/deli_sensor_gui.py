#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
物流称重传感器测试程序
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
import queue
import glob
from datetime import datetime

# ROS相关导入
try:
    import rospy
    from std_msgs.msg import String, Float32, Int32, Bool
except ImportError:
    rospy = None
    print("警告: 未安装ROS Python库，ROS话题功能将不可用")

class WeightSensorGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("物流称重传感器测试程序")
        self.root.geometry("680x440")
        
        # 串口相关
        self.serial_port = None
        self.serial_connected = False
        self.serial_thread = None
        self.stop_serial_thread = False
        
        # ROS进程相关
        self.ros_process = None
        self.ros_pid = None
        self.ros_running = False
        self.ros_node_initialized = False
        
        # ROS话题相关
        self.weight_sub = None
        self.cal_factor_sub = None
        self.emergency_sub = None
        self.device_state_sub = None
        self.cmd_pub = None
        
        # 数据队列
        self.data_queue = queue.Queue()
        self.status_queue = queue.Queue()
        
        # 当前数据状态
        self.current_weight = 0.0
        self.calibration_factor = 110.0  # 默认校准因子
        self.emergency_stop = False
        self.device_state = 0  # 0-正常, 1-归零中, 2-校准中, 3-初始化中
        
        # 配置路径
        self.config_dir = os.path.expanduser("/home/aihit/aihitplt_ws/src/aihitplt_hardware_test/config")
        self.config_file = os.path.join(self.config_dir, "weight_sensor_config.yaml")
        
        # 确保config目录存在
        if not os.path.exists(self.config_dir):
            try:
                os.makedirs(self.config_dir, exist_ok=True)
                print(f"已创建config目录: {self.config_dir}")
            except Exception as e:
                print(f"创建config目录失败: {e}")
        
        self.default_config = {
            'serial_port': '/dev/ttyUSB0',
            'baudrate': 115200
        }
        
        # 保存默认按钮颜色（用于ROS启动按钮）
        temp_btn = tk.Button(self.root)
        self.default_bg = temp_btn.cget("bg")
        self.default_fg = temp_btn.cget("fg")
        temp_btn.destroy()
        
        # 创建界面
        self.create_interface()
        
        # 初始化
        self.refresh_serial_ports()
        self.load_config()
        
        # 启动更新线程
        self.start_update_thread()
        
        # 绑定关闭事件
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        
        self.update_status("就绪")
    
    def create_interface(self):
        """创建界面"""
        # 主框架
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.pack(fill=tk.BOTH, expand=True)
        
        # 左右分栏
        left_frame = ttk.Frame(main_frame)
        left_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 5))
        
        right_frame = ttk.Frame(main_frame)
        right_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=(5, 0))
        
        # ====== 左侧面板 ======
        # 1. 系统连接
        self.create_system_connection(left_frame)
        
        # 2. 称重传感器状态
        self.create_sensor_status(left_frame)
        
        # 3. 称重传感器校准
        self.create_sensor_calibration(left_frame)
        
        # ====== 右侧面板 ======
        # 1. 称重显示
        self.create_weight_display(right_frame)
        
        # 2. 急停状态
        self.create_emergency_status(right_frame)
        
        # 3. 串口配置
        self.create_serial_config(right_frame)
        
        # 4. ROS Launch启动
        self.create_ros_launch(right_frame)
        
        # ====== 底部状态栏 ======
        self.status_var = tk.StringVar(value="就绪")
        status_bar = ttk.Label(self.root, textvariable=self.status_var,
                              relief=tk.SUNKEN, anchor=tk.W, padding=(5, 2))
        status_bar.pack(side=tk.BOTTOM, fill=tk.X)
    
    def create_system_connection(self, parent):
        """创建系统连接模块"""
        frame = ttk.LabelFrame(parent, text="系统连接", padding="10")
        frame.pack(fill=tk.X, pady=(0, 10))
        
        # 串口选择行
        port_frame = ttk.Frame(frame)
        port_frame.pack(fill=tk.X, pady=(0, 10))
        
        ttk.Label(port_frame, text="串口号:", width=7).pack(side=tk.LEFT, padx=(0, 0))
        
        self.port_combo = ttk.Combobox(port_frame, width=20, state="readonly")
        self.port_combo.pack(side=tk.LEFT, padx=(0, 10))
        
        self.refresh_btn = ttk.Button(port_frame, text="刷新", width=8,
                                     command=self.refresh_serial_ports)
        self.refresh_btn.pack(side=tk.LEFT, padx=(0, 10))
        
        self.connect_btn = ttk.Button(port_frame, text="连接", width=8,
                                     command=self.toggle_serial_connection)
        self.connect_btn.pack(side=tk.LEFT)
    
    def create_sensor_status(self, parent):
        """创建称重传感器状态模块"""
        frame = ttk.LabelFrame(parent, text="称重传感器状态", padding="8 8 8 8")
        frame.pack(fill=tk.X, pady=(0, 10))
        
        # 当前校准因子行
        cal_frame = ttk.Frame(frame)
        cal_frame.pack(fill=tk.X, pady=(0, 8))
        
        ttk.Label(cal_frame, text="当前校准因子:", width=12).pack(side=tk.LEFT)
        self.cal_factor_var = tk.StringVar(value="110.0")
        self.cal_factor_label = ttk.Label(cal_frame, textvariable=self.cal_factor_var,
                                        font=('Arial', 10, 'bold'))
        self.cal_factor_label.pack(side=tk.LEFT, padx=(5, 0))
        
        # 手动设置行 - 所有组件在同一行
        row_frame = ttk.Frame(frame)
        row_frame.pack(fill=tk.X)
        
        # 标签
        ttk.Label(row_frame, text="设置校准因子:", width=12).pack(side=tk.LEFT)
        
        # 输入框
        self.new_cal_var = tk.StringVar()
        self.new_cal_entry = ttk.Entry(row_frame, textvariable=self.new_cal_var, width=12)
        self.new_cal_entry.pack(side=tk.LEFT, padx=(5, 15))  # 输入框右边距10
        
        # 两个按钮紧挨着
        self.reset_btn = ttk.Button(row_frame, text="重置传感器", width=10,
                                command=self.reset_sensor, state="disabled")
        self.reset_btn.pack(side=tk.LEFT, padx=(0, 5))  # 按钮之间间距5
        
        self.set_cal_btn = ttk.Button(row_frame, text="设置参数", width=10,
                                    command=self.set_calibration_factor, state="disabled")
        self.set_cal_btn.pack(side=tk.LEFT, padx=(0, 0))  # 最右边不留间距
    
    def create_sensor_calibration(self, parent):
        """创建称重传感器校准模块"""
        frame = ttk.LabelFrame(parent, text="称重传感器校准", padding="10")
        frame.pack(fill=tk.X, pady=(0, 10))
        
        # 校准重量输入
        weight_frame = ttk.Frame(frame)
        weight_frame.pack(fill=tk.X, pady=(0, 10))
        
        ttk.Label(weight_frame, text="校准重量 (g):", width=11).pack(side=tk.LEFT)
        
        self.cal_weight_var = tk.StringVar()
        self.cal_weight_entry = ttk.Entry(weight_frame, textvariable=self.cal_weight_var, width=12)
        self.cal_weight_entry.pack(side=tk.LEFT, padx=(10, 16))
        
        self.start_cal_btn = ttk.Button(weight_frame, text="开始校准", width=10,
                                       command=self.start_calibration, state="disabled")
        self.start_cal_btn.pack(side=tk.LEFT)
        
        # 校准说明
        desc_frame = ttk.LabelFrame(frame, text="校准说明", padding="8")
        desc_frame.pack(fill=tk.X)
        
        instructions = [
            "1. 先点击[归零]清空重量",
            "2. 放置已知重量的校准块", 
            "3. 输入实际重量，点击[开始校准]"
        ]
        
        for i, instruction in enumerate(instructions):
            ttk.Label(desc_frame, text=instruction, anchor=tk.W).pack(fill=tk.X, pady=2)
    
    def create_weight_display(self, parent):
        """创建称重显示模块 - 修改：只在同一位置显示重量或状态"""
        frame = ttk.LabelFrame(parent, text="称重显示 (范围: 2.0-20000.0g)", padding="16")
        frame.pack(fill=tk.X, pady=(0, 10))
        
        # 按钮和重量显示框
        display_frame = ttk.Frame(frame)
        display_frame.pack(fill=tk.X, expand=True)
        
        # 归零按钮（左边）
        self.zero_btn = ttk.Button(display_frame, text="归零", width=10, 
                                  command=self.zero_sensor, state="disabled")
        self.zero_btn.pack(side=tk.LEFT, padx=(0, 20))
        
        # 重量显示框（右边）- 改为一个大的显示标签
        weight_display = ttk.Frame(display_frame)
        weight_display.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)
        
        # 创建显示标签 - 用于显示重量或状态
        self.weight_display_var = tk.StringVar(value="0.0 g")
        self.weight_display_label = ttk.Label(weight_display, textvariable=self.weight_display_var,
                                            font=('Arial', 24, 'bold'),
                                            foreground="blue")
        self.weight_display_label.pack(expand=True)
        
        # 移除原来的重量状态标签，因为现在要替换重量显示
    
    def create_emergency_status(self, parent):
        """创建急停状态模块"""
        frame = ttk.LabelFrame(parent, text="急停状态", padding="14")
        frame.pack(fill=tk.X, pady=(0, 7))
        
        # 急停状态显示框 - 使用普通tk.Label而不是ttk.Label，因为它支持height
        self.emergency_var = tk.StringVar(value="急停状态: 未知")
        self.emergency_label = tk.Label(frame, textvariable=self.emergency_var,
                                       font=('Arial', 14),
                                       width=20, height=3,
                                       relief=tk.RIDGE)
        self.emergency_label.pack()
        
        # 状态颜色指示
        self.update_emergency_display()
    
    def create_serial_config(self, parent):
        """创建串口配置模块"""
        frame = ttk.LabelFrame(parent, text="串口配置", padding="10")
        frame.pack(fill=tk.X, pady=(0, 10))
        
        self.save_port_btn = ttk.Button(frame, text="保存物流称重系统串口号", width=30,
                                       command=self.save_serial_port)
        self.save_port_btn.pack(pady=5)
    
    def create_ros_launch(self, parent):
        """创建ROS Launch启动模块"""
        frame = ttk.LabelFrame(parent, text="ROS Launch启动", padding="10")
        frame.pack(fill=tk.X)
        
        # 使用tk.Button而不是ttk.Button，以便设置背景色
        self.ros_launch_btn = tk.Button(frame, text="启动物流称重系统launch文件", 
                                      command=self.toggle_ros_launch,
                                      font=('Arial', 10), width=28)
        self.ros_launch_btn.pack(pady=5)
        
    
    def refresh_serial_ports(self):
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
        # 如果ROS正在运行，不能直接连接串口
        if self.ros_running:
            messagebox.showwarning("警告", "ROS正在运行，请先关闭ROS Launch再连接串口")
            return
            
        port = self.port_combo.get()
        
        if not port:
            messagebox.showwarning("警告", "请选择串口")
            return
        
        try:
            if not os.path.exists(port):
                messagebox.showerror("错误", f"串口不存在: {port}")
                return
            
            # 连接串口
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
            
            # 更新UI状态
            self.connect_btn.config(text="断开")
            self.refresh_btn.config(state="disabled")
            self.reset_btn.config(state="normal")
            self.set_cal_btn.config(state="normal")
            self.zero_btn.config(state="normal")
            self.start_cal_btn.config(state="normal")
            self.save_port_btn.config(state="disabled")
            
            # 启动数据读取线程
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
            
            # 更新UI状态
            self.connect_btn.config(text="连接")
            self.refresh_btn.config(state="normal")
            self.reset_btn.config(state="disabled")
            self.set_cal_btn.config(state="disabled")
            self.zero_btn.config(state="disabled")
            self.start_cal_btn.config(state="disabled")
            self.save_port_btn.config(state="normal")
            
            # 重置显示
            self.weight_display_var.set("0.0 g")
            self.emergency_var.set("急停状态: 未知")
            self.update_emergency_display()
            self.cal_factor_var.set("110.0")
            
            self.update_status("已断开连接")
    
    def read_serial_data(self):
        """读取串口数据"""
        while self.serial_connected and not self.stop_serial_thread:
            try:
                if self.serial_port and self.serial_port.in_waiting > 0:
                    data = self.serial_port.readline().decode('utf-8', errors='ignore').strip()
                    
                    if data:
                        # 解析数据
                        self.parse_sensor_data(data)
                        
                        # 发送到队列更新UI
                        self.data_queue.put(data)
            
            except Exception as e:
                if not self.stop_serial_thread:
                    self.status_queue.put(f"串口读取错误: {e}")
                
                time.sleep(0.1)
            
            time.sleep(0.01)
    
    def parse_sensor_data(self, data):
        """解析传感器数据"""
        # 解析协议格式: DATA:ES=0,CAL=110.0,WEIGHT=0.0,STATE=0
        import re
        
        try:
            # 解析完整数据包
            pattern = r"DATA:ES=(\d+),CAL=([\d.]+),WEIGHT=([\d.]+),STATE=(\d+)"
            match = re.match(pattern, data)
            
            if match:
                self.emergency_stop = bool(int(match.group(1)))
                self.calibration_factor = float(match.group(2))
                self.current_weight = float(match.group(3))
                self.device_state = int(match.group(4))
                
                # 在GUI线程中更新显示
                self.root.after(0, self.update_ui_from_data)
                
                return
            
            # 解析急停单独消息
            if data.startswith("ES:"):
                if data == "ES:1":
                    self.emergency_stop = True
                elif data == "ES:0":
                    self.emergency_stop = False
                
                # 在GUI线程中更新显示
                self.root.after(0, self.update_emergency_display)
                
        except Exception as e:
            print(f"解析数据失败: {e}, 数据: {data}")
    
    def update_ui_from_data(self):
        """从数据更新UI显示"""
        try:
            # 更新校准因子
            self.cal_factor_var.set(f"{self.calibration_factor:.1f}")
            
            # 根据设备状态显示不同内容
            if self.device_state == 0:  # 正常
                # 正常状态下显示重量
                self.weight_display_var.set(f"{self.current_weight:.1f} g")
                self.weight_display_label.config(foreground="blue")
            elif self.device_state == 1:  # 归零中
                # 归零状态下显示状态信息，替换重量数字
                self.weight_display_var.set("归零中...")
                self.weight_display_label.config(foreground="orange")
            elif self.device_state == 2:  # 校准中
                # 校准状态下显示状态信息，替换重量数字
                self.weight_display_var.set("校准中...")
                self.weight_display_label.config(foreground="orange")
            
            # 更新急停状态
            self.update_emergency_display()
        except Exception as e:
            print(f"更新UI失败: {e}")
    
    def update_emergency_display(self):
        """更新急停状态显示"""
        if not self.serial_connected and not self.ros_node_initialized:
            self.emergency_var.set("急停状态: 未知")
            self.emergency_label.config(background="light gray", foreground="black")
            return
        
        if self.emergency_stop:
            self.emergency_var.set("急停状态: 按下")
            self.emergency_label.config(background="red", foreground="white")
        else:
            self.emergency_var.set("急停状态: 松开")
            self.emergency_label.config(background="green", foreground="white")
    
    def send_command(self, command):
        """发送命令（串口或ROS话题）"""
        if self.serial_connected:
            # 通过串口发送命令
            try:
                self.serial_port.write(f"{command}\n".encode('utf-8'))
                self.update_status(f"已发送命令: {command}")
            except Exception as e:
                messagebox.showerror("发送失败", f"无法发送命令: {e}")
        elif self.ros_node_initialized and self.cmd_pub:
            # 通过ROS话题发送命令
            try:
                msg = String()
                msg.data = command
                self.cmd_pub.publish(msg)
                self.update_status(f"已通过ROS话题发送命令: {command}")
            except Exception as e:
                messagebox.showerror("发送失败", f"无法通过ROS话题发送命令: {e}")
        else:
            messagebox.showwarning("警告", "未连接到串口或ROS节点")
    
    def zero_sensor(self):
        """传感器归零"""
        self.send_command("z")
        # 立即更新显示为"归零中..."
        self.weight_display_var.set("归零中...")
        self.weight_display_label.config(foreground="orange")
    
    def reset_sensor(self):
        """重置传感器"""
        self.send_command("r")
        self.update_status("已发送重置指令")
    
    def set_calibration_factor(self):
        """设置校准因子"""
        try:
            new_factor = self.new_cal_var.get()
            if not new_factor:
                messagebox.showwarning("警告", "请输入校准因子")
                return
            
            factor = float(new_factor)
            if factor < 1.0 or factor > 1000.0:
                messagebox.showwarning("警告", "校准因子范围: 1.0-1000.0")
                return
            
            # 发送命令格式: k[factor]，例如 k120
            self.send_command(f"k{factor}")
            
        except ValueError:
            messagebox.showerror("错误", "请输入有效的数字")
        except Exception as e:
            messagebox.showerror("发送失败", f"无法发送设置指令: {e}")
    
    def start_calibration(self):
        """开始校准"""
        try:
            cal_weight = self.cal_weight_var.get()
            if not cal_weight:
                messagebox.showwarning("警告", "请输入校准重量")
                return
            
            weight = float(cal_weight)
            if weight <= 0:
                messagebox.showwarning("警告", "校准重量必须大于0")
                return
            
            # 发送命令格式: c[weight]，例如 c12
            self.send_command(f"c{weight}")
            # 立即更新显示为"校准中..."
            self.weight_display_var.set("校准中...")
            self.weight_display_label.config(foreground="orange")
            
        except ValueError:
            messagebox.showerror("错误", "请输入有效的数字")
        except Exception as e:
            messagebox.showerror("发送失败", f"无法发送校准指令: {e}")
    
    def save_serial_port(self):
        """保存串口号 - 修复路径问题"""
        port = self.port_combo.get()
        
        if not port:
            messagebox.showwarning("警告", "请先选择串口")
            return
        
        try:
            # 确保config目录存在
            if not os.path.exists(self.config_dir):
                os.makedirs(self.config_dir, exist_ok=True)
                print(f"已创建config目录: {self.config_dir}")
            
            config = {
                'serial_port': port,
                'baudrate': 115200,
                'last_saved': datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            }
            
            with open(self.config_file, 'w') as f:
                yaml.dump(config, f, default_flow_style=False)
            
            messagebox.showinfo("保存成功", f"串口 {port} 已保存成功\n配置文件路径: {self.config_file}")
            self.update_status("配置已保存")
            
        except Exception as e:
            messagebox.showerror("保存失败", f"无法保存配置: {e}\n请检查路径: {self.config_file}")
            print(f"保存失败详细: {e}")
    
    def load_config(self):
        """加载配置"""
        if not os.path.exists(self.config_file):
            print(f"配置文件不存在: {self.config_file}")
            return
        
        try:
            with open(self.config_file, 'r') as f:
                config = yaml.safe_load(f)
            
            if config and 'serial_port' in config:
                self.port_combo.set(config['serial_port'])
                self.update_status("配置已加载")
                print(f"从配置文件加载串口: {config['serial_port']}")
                
        except Exception as e:
            print(f"加载配置失败: {e}")
    
    def toggle_ros_launch(self):
        """切换ROS Launch"""
        if not self.ros_running:
            self.start_ros_launch()
        else:
            self.stop_ros_launch()
    
    def start_ros_launch(self):
        """启动ROS Launch"""
        try:
            # 首先断开串口连接（如果已连接）
            if self.serial_connected:
                self.disconnect_serial()
            
            # 构建roslaunch命令
            roslaunch_cmd = 'roslaunch aihitplt_hardware_test logi_scale.launch'
            
            # 在新终端中启动
            cmd = [
                'gnome-terminal',
                '--title=物流称重系统',
                '--geometry=80x24+100+100',
                '--',
                'bash', '-c',
                f'source ~/.bashrc && '
                f'{roslaunch_cmd}; '
                f'echo "ROS Launch运行中...按Enter键关闭终端"; '
                f'read'
            ]
            
            # 启动进程
            self.ros_process = subprocess.Popen(
                cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                start_new_session=True
            )
            
            self.ros_pid = self.ros_process.pid
            self.ros_running = True
            
            # 更新UI - 按钮变绿，文本变白
            self.ros_launch_btn.config(text="关闭物流称重系统", 
                                     bg="green", fg="white")
            
            # 禁用串口相关按钮
            self.connect_btn.config(state="disabled")
            self.refresh_btn.config(state="disabled")
            self.save_port_btn.config(state="disabled")
            
            # 启用控制按钮（通过ROS话题）
            self.reset_btn.config(state="normal")
            self.set_cal_btn.config(state="normal")
            self.zero_btn.config(state="normal")
            self.start_cal_btn.config(state="normal")
            
            self.update_status("ROS Launch已启动，正在初始化ROS节点...")
            
            # 在子线程中初始化ROS节点
            threading.Thread(target=self.init_ros_node, daemon=True).start()
            
        except Exception as e:
            messagebox.showerror("启动失败", f"无法启动ROS Launch:\n{e}")
    
    def init_ros_node(self):
        """初始化ROS节点"""
        if rospy is None:
            self.root.after(0, lambda: self.update_status("未安装ROS Python库，无法初始化ROS节点"))
            return
        
        try:
            # 等待ROS Master启动
            import rosgraph
            max_attempts = 10
            for i in range(max_attempts):
                try:
                    rosgraph.Master('/rostopic').getPid()
                    break
                except:
                    if i < max_attempts - 1:
                        time.sleep(1)
                        self.root.after(0, lambda i=i: self.update_status(f"等待ROS Master启动... ({i+1}/{max_attempts})"))
                    else:
                        raise Exception("ROS Master未启动")
            
            # 初始化ROS节点
            rospy.init_node('weight_sensor_gui_node', anonymous=True, disable_signals=True)
            
            # 创建话题订阅器
            self.weight_sub = rospy.Subscriber(
                '/logi_scale/weight',
                Float32,
                self.weight_callback
            )
            
            self.cal_factor_sub = rospy.Subscriber(
                '/logi_scale/calibration_factor',
                Float32,
                self.cal_factor_callback
            )
            
            self.emergency_sub = rospy.Subscriber(
                '/logi_scale/emergency_stop',
                Bool,
                self.emergency_callback
            )
            
            self.device_state_sub = rospy.Subscriber(
                '/logi_scale/device_state',
                Int32,
                self.device_state_callback
            )
            
            # 创建命令发布器
            self.cmd_pub = rospy.Publisher(
                '/logi_scale/control',
                String,
                queue_size=10
            )
            
            # 等待发布器建立连接
            time.sleep(0.5)
            
            self.ros_node_initialized = True
            
            self.root.after(0, lambda: self.update_status("ROS节点初始化成功，已连接ROS话题"))
            self.root.after(0, lambda: self.update_status("现在通过ROS话题与传感器通信"))
            
        except Exception as e:
            error_msg = f"ROS节点初始化失败: {e}"
            print(error_msg)
            self.root.after(0, lambda: self.update_status(error_msg))
    
    def weight_callback(self, msg):
        """重量话题回调函数"""
        self.current_weight = msg.data
        if self.device_state == 0:  # 仅在正常状态下更新重量显示
            self.root.after(0, lambda w=msg.data: self.weight_display_var.set(f"{w:.1f} g"))
    
    def cal_factor_callback(self, msg):
        """校准因子话题回调函数"""
        self.calibration_factor = msg.data
        self.root.after(0, lambda c=msg.data: self.cal_factor_var.set(f"{c:.1f}"))
    
    def emergency_callback(self, msg):
        """急停话题回调函数"""
        self.emergency_stop = msg.data
        self.root.after(0, self.update_emergency_display)
    
    def device_state_callback(self, msg):
        """设备状态话题回调函数"""
        self.device_state = msg.data
        
        # 在GUI线程中更新显示
        self.root.after(0, lambda: self.update_weight_display_by_state())
    
    def update_weight_display_by_state(self):
        """根据设备状态更新重量显示"""
        if self.device_state == 0:  # 正常
            self.weight_display_var.set(f"{self.current_weight:.1f} g")
            self.weight_display_label.config(foreground="blue")
        elif self.device_state == 1:  # 归零中
            self.weight_display_var.set("归零中...")
            self.weight_display_label.config(foreground="orange")
        elif self.device_state == 2:  # 校准中
            self.weight_display_var.set("校准中...")
            self.weight_display_label.config(foreground="orange")
    
    def stop_ros_launch(self):
        """停止ROS Launch"""
        if self.ros_running:
            try:
                # 清理ROS节点
                self.ros_node_initialized = False
                
                # 清理ROS话题
                if self.weight_sub:
                    self.weight_sub.unregister()
                if self.cal_factor_sub:
                    self.cal_factor_sub.unregister()
                if self.emergency_sub:
                    self.emergency_sub.unregister()
                if self.device_state_sub:
                    self.device_state_sub.unregister()
                if self.cmd_pub:
                    self.cmd_pub.unregister()
                
                # 终止gnome-terminal进程
                if self.ros_pid:
                    import psutil
                    try:
                        process = psutil.Process(self.ros_pid)
                        for child in process.children(recursive=True):
                            try:
                                child.terminate()
                            except:
                                pass
                        process.terminate()
                    except:
                        pass
                
                # 终止roscore和相关进程
                subprocess.run(['pkill', '-f', 'logi_scale.launch'],
                             stdout=subprocess.DEVNULL,
                             stderr=subprocess.DEVNULL)
                subprocess.run(['pkill', '-f', 'logi_scale_node'],
                             stdout=subprocess.DEVNULL,
                             stderr=subprocess.DEVNULL)
                
            except Exception as e:
                print(f"停止ROS进程时出错: {e}")
            
            finally:
                # 更新UI
                self.ros_running = False
                self.ros_process = None
                self.ros_pid = None
                
                # 恢复按钮状态和颜色
                self.ros_launch_btn.config(text="启动物流称重系统launch文件", 
                                         bg=self.default_bg, fg=self.default_fg)
                
                # 恢复串口相关按钮
                self.connect_btn.config(state="normal")
                self.refresh_btn.config(state="normal")
                self.save_port_btn.config(state="normal")
                
                # 禁用控制按钮（因为没有连接）
                self.reset_btn.config(state="disabled")
                self.set_cal_btn.config(state="disabled")
                self.zero_btn.config(state="disabled")
                self.start_cal_btn.config(state="disabled")
                
                # 重置显示
                self.weight_display_var.set("0.0 g")
                self.emergency_var.set("急停状态: 未知")
                self.update_emergency_display()
                self.cal_factor_var.set("110.0")
                
                self.root.after(0, lambda: self.update_status("ROS Launch已停止"))
    
    def start_update_thread(self):
        """启动更新线程"""
        def update_display():
            while True:
                try:
                    # 处理串口数据
                    try:
                        data = self.data_queue.get(timeout=0.1)
                        # 数据已在parse_sensor_data中处理
                        pass
                    except queue.Empty:
                        pass
                    
                    # 处理状态消息
                    try:
                        status = self.status_queue.get(timeout=0.1)
                        self.root.after(0, lambda s=status: self.update_status(s))
                    except queue.Empty:
                        pass
                    
                except Exception as e:
                    print(f"更新线程错误: {e}")
                    time.sleep(0.5)
        
        thread = threading.Thread(target=update_display, daemon=True)
        thread.start()
    
    def update_status(self, message):
        """更新状态栏"""
        self.status_var.set(message)
    
    def on_closing(self):
        """窗口关闭时的清理"""
        # 断开串口
        if self.serial_connected:
            self.disconnect_serial()
        
        # 停止ROS
        if self.ros_running:
            self.stop_ros_launch()
        
        self.root.destroy()


def main():
    """主函数"""
    # 设置显示环境
    if 'DISPLAY' not in os.environ:
        os.environ['DISPLAY'] = ':0'
    
    # 创建主窗口
    root = tk.Tk()
    
    # 窗口居中
    window_width = 800
    window_height = 600
    screen_width = root.winfo_screenwidth()
    screen_height = root.winfo_screenheight()
    x = (screen_width - window_width) // 2
    y = (screen_height - window_height) // 2
    root.geometry(f"{window_width}x{window_height}+{x}+{y}")
    
    # 创建应用
    app = WeightSensorGUI(root)
    
    # 运行主循环
    root.mainloop()


if __name__ == "__main__":
    main()