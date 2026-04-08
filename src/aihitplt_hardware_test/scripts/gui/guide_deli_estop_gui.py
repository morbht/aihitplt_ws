#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# 迎宾模块和送餐模块急停按钮测试程序

import tkinter as tk
from tkinter import ttk, messagebox
import serial
import serial.tools.list_ports
import threading
import subprocess
import os
import time
import sys
import yaml
from datetime import datetime
import glob
import queue
import psutil
import rospy
from std_msgs.msg import Bool
import rospkg

class GuideDeliveryEmergencyTestApp:
    def __init__(self, root):
        self.root = root
        self.root.title("迎宾与送餐模块急停按钮测试")
        self.root.geometry("400x480")  # 调整为500x600
        
        # ROS包路径
        self.rospack = rospkg.RosPack()
        self.pkg_path = None
        self.config_file = None
        self.launch_file = None
        
        # 初始化ROS包路径
        self._init_paths()
        
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
        
        # ROS话题订阅
        self.emergency_sub = None
        
        # 急停按钮状态
        self.emergency_status = "未知"
        
        # 数据队列用于线程间通信
        self.status_queue = queue.Queue()
        
        # 保存默认按钮颜色（用于恢复）
        temp_btn = tk.Button(self.root)
        self.default_bg = temp_btn.cget("bg")
        self.default_fg = temp_btn.cget("fg")
        temp_btn.destroy()
        
        # 创建界面
        self.create_widgets()
        
        # 自动刷新串口列表
        self.refresh_serial_ports()
        
        # 加载保存的串口
        self.load_saved_port()
        
        # 启动状态更新线程
        self.status_update_thread = threading.Thread(
            target=self.update_status_from_queue,
            daemon=True
        )
        self.status_update_thread.start()
        
        # 绑定关闭事件
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
    
    def _init_paths(self):
        """初始化路径配置"""
        try:
            # 获取ROS包路径
            self.pkg_path = self.rospack.get_path('aihitplt_hardware_test')
            self.config_file = os.path.join(self.pkg_path, 'config', 'emergency_stop_port.yaml')
            self.launch_file = os.path.join(self.pkg_path, 'launch', 'guide_deli_estop.launch')
            
            print(f"ROS包路径: {self.pkg_path}")
            print(f"配置文件路径: {self.config_file}")
            print(f"Launch文件路径: {self.launch_file}")
            
            # 确保config目录存在
            config_dir = os.path.join(self.pkg_path, 'config')
            if not os.path.exists(config_dir):
                os.makedirs(config_dir, exist_ok=True)
                print(f"创建config目录: {config_dir}")
                
        except rospkg.ResourceNotFound as e:
            print(f"ROS包未找到: {e}")
            # 尝试使用默认路径
            default_path = "/home/aihit/aihitplt_ws/src/aihitplt_hardware_test"
            if os.path.exists(default_path):
                self.pkg_path = default_path
                self.config_file = os.path.join(self.pkg_path, 'config', 'emergency_stop_port.yaml')
                self.launch_file = os.path.join(self.pkg_path, 'launch', 'guide_deli_estop.launch')
                
                # 确保config目录存在
                config_dir = os.path.join(self.pkg_path, 'config')
                if not os.path.exists(config_dir):
                    os.makedirs(config_dir, exist_ok=True)
            else:
                print("警告: 未找到ROS包路径")
                self.pkg_path = None
        except Exception as e:
            print(f"初始化路径时出错: {e}")
    
    def create_widgets(self):
        """创建界面 - 简洁布局"""
        # 主框架
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.pack(fill=tk.BOTH, expand=True)
        
        # 第一部分：串口连接
        self._create_part1(main_frame)
        
        # 第二部分：急停按钮状态
        self._create_part2(main_frame)
        
        # 第三部分：保存串口号
        self._create_part3(main_frame)
        
        # 第四部分：启动launch文件
        self._create_part4(main_frame)
        
        # 状态栏 - 减少上边距
        self.status_var = tk.StringVar(value="就绪")
        status_bar = ttk.Label(main_frame, textvariable=self.status_var, 
                              relief=tk.SUNKEN, anchor=tk.W, padding=(5, 2))
        status_bar.pack(side=tk.BOTTOM, fill=tk.X, pady=(2, 0))  # 减少pady
    
    def _create_part1(self, parent):
        """创建第一部分：串口连接 - 改为一行布局"""
        frame = ttk.LabelFrame(parent, text="急停按钮连接", padding=8)
        frame.pack(fill=tk.X, pady=(0, 10))
        
        # 单行布局：串口选择 + 刷新按钮 + 连接按钮
        control_frame = ttk.Frame(frame)
        control_frame.pack(fill=tk.X, pady=3)
        
        # 串口选择标签
        tk.Label(control_frame, text="串口:", font=('Arial', 9)).pack(side=tk.LEFT, padx=2)
        
        # 为串口选择框创建一个单独的容器frame来限制宽度
        combo_frame = ttk.Frame(control_frame)
        combo_frame.pack(side=tk.LEFT, padx=5)
        
        self.port_combo = ttk.Combobox(combo_frame, state="readonly", width=20)
        self.port_combo.pack()
        
        # 刷新串口按钮
        self.refresh_btn = ttk.Button(control_frame, text="刷新", 
                                    command=self.refresh_serial_ports, width=8)
        self.refresh_btn.pack(side=tk.LEFT, padx=5)
        
        # 连接按钮（简化文本）
        self.connect_btn = ttk.Button(control_frame, text="连接", 
                                    command=self.toggle_serial_connection, width=8)
        self.connect_btn.pack(side=tk.LEFT, padx=5)
        
    def _create_part2(self, parent):
        """创建第二部分：急停按钮状态"""
        frame = ttk.LabelFrame(parent, text="急停按钮状态", padding=10)
        frame.pack(fill=tk.X, pady=(0, 10))
        
        # 状态按钮
        self.status_btn = tk.Button(frame, text="未知", font=('Arial', 12), 
                                    width=15, height=2, state="disabled", bg="lightgray")
        self.status_btn.pack(pady=5)
        
        # 状态说明
        status_text = tk.Label(frame, text="状态说明:", font=('Arial', 9), justify=tk.LEFT)
        status_text.pack(anchor=tk.W, pady=(5, 0))
        
        status_detail = tk.Label(frame, text="按下=绿色 | 松开=灰色 | 未知=灰色", 
                                font=('Arial', 8), fg="gray", justify=tk.LEFT)
        status_detail.pack(anchor=tk.W)
    
    def _create_part3(self, parent):
        """创建第三部分：保存串口号"""
        frame = ttk.LabelFrame(parent, text="串口配置", padding=10)
        frame.pack(fill=tk.X, pady=(0, 10))
        
        if self.pkg_path:
            self.save_btn = ttk.Button(frame, text="保存急停按钮串口号", 
                                      command=self.save_serial_port, width=28)
            self.save_btn.pack(pady=5)
        else:
            self.save_btn = ttk.Button(frame, text="保存急停按钮串口号 (ROS包未找到)", 
                                      state="disabled", width=28)
            self.save_btn.pack(pady=5)

    
    def _create_part4(self, parent):
        """创建第四部分：启动launch文件"""
        frame = ttk.LabelFrame(parent, text="ROS Launch控制", padding=10)
        frame.pack(fill=tk.X, pady=(0, 5))  # 减少下边距，让日志栏更靠近
        
        if self.pkg_path and os.path.exists(self.launch_file):
            # 使用tk.Button而不是ttk.Button，以便设置背景色
            self.launch_btn = tk.Button(frame, text="启动急停launch文件", 
                                        command=self.toggle_ros_launch,
                                        font=('Arial', 10), width =26)
            self.launch_btn.pack(pady=5)
        else:
            self.launch_btn = tk.Button(frame, text="启动launch文件 (文件未找到)", 
                                        state="disabled", font=('Arial', 10), width=26)
            self.launch_btn.pack(pady=5)
    
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
    
    def _normalize_port_path(self, port):
        """规范化串口路径格式"""
        if not port:
            return port
        
        if port.startswith('//dev/'):
            port = port.replace('//dev/', '/dev/')
        elif not port.startswith('/dev/'):
            port = f'/dev/{port}'
        
        return port
    
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
            
            # 更新按钮状态
            self.connect_btn.config(text="断开")  # 连接后改为"断开"
            self.refresh_btn.config(state="disabled")
            if self.pkg_path:
                self.save_btn.config(state="disabled")
            self.launch_btn.config(state="disabled")
            
            # 启动读取线程
            self.serial_thread = threading.Thread(
                target=self.read_serial_data,
                daemon=True
            )
            self.serial_thread.start()
            
            self.update_status(f"已连接到串口: {port}")
            
            # 更新按钮状态为未知（等待数据）
            self.update_emergency_status("未知")
            
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
            self.connect_btn.config(text="连接")  # 断开后改回"连接"
            self.refresh_btn.config(state="normal")
            if self.pkg_path:
                self.save_btn.config(state="normal")
            
            # 如果ROS没有运行，启用launch按钮
            if not self.ros_running and self.pkg_path and os.path.exists(self.launch_file):
                self.launch_btn.config(state="normal")
            
            self.update_status("已断开串口连接")
            
            # 更新按钮状态为未知
            self.update_emergency_status("未知")
    
    def read_serial_data(self):
        """读取串口数据"""
        while self.serial_connected and not self.stop_serial_thread:
            try:
                if self.serial_port and self.serial_port.in_waiting > 0:
                    data = self.serial_port.readline().decode('utf-8', errors='ignore').strip()
                    
                    if data:
                        # 根据guide_deli_estop.py的协议解析数据
                        if data == "P":  # 急停按下
                            self.update_emergency_status("按下")
                        elif data == "R":  # 急停释放
                            self.update_emergency_status("松开")
                        else:
                            # 无法解析的数据
                            self.update_emergency_status("未知")
                            self.update_status(f"接收到无法解析的数据: {data}")
            
            except Exception as e:
                if not self.stop_serial_thread:
                    print(f"串口读取错误: {e}")
                    self.update_status(f"串口读取错误: {e}")
                    self.update_emergency_status("未知")
                break
            
            time.sleep(0.01)
    
    def update_emergency_status(self, status):
        """更新急停按钮状态"""
        self.emergency_status = status
        self.status_queue.put(status)
    
    def update_status_from_queue(self):
        """从队列中获取状态并更新显示"""
        while True:
            try:
                status = self.status_queue.get(timeout=0.1)
                self.root.after(0, lambda s=status: self._update_status_gui(s))
            except queue.Empty:
                continue
            except Exception as e:
                print(f"更新状态错误: {e}")
    
    def _update_status_gui(self, status):
        """在GUI线程中更新状态显示"""
        self.status_btn.config(text=status)
        
        if status == "按下":
            self.status_btn.config(bg="green", fg="white")
        elif status == "松开":
            self.status_btn.config(bg="lightgray", fg="black")
        else:  # 未知
            self.status_btn.config(bg="lightgray", fg="black")
    
    def save_serial_port(self):
        """保存串口号到功能包config文件夹"""
        if not self.pkg_path:
            messagebox.showerror("错误", "未找到ROS包路径，无法保存配置")
            return
        
        port = self.port_combo.get()
        if not port:
            messagebox.showwarning("警告", "请先选择串口")
            return
        
        port = self._normalize_port_path(port)
        
        try:
            # 确保config目录存在
            config_dir = os.path.join(self.pkg_path, 'config')
            if not os.path.exists(config_dir):
                os.makedirs(config_dir, exist_ok=True)
                print(f"创建config目录: {config_dir}")
            
            # 保存到配置文件
            config = {
                'emergency_stop_port': port,
                'baudrate': 115200,
                'last_updated': datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            }
            
            with open(self.config_file, 'w') as f:
                yaml.dump(config, f, default_flow_style=False)
            
            # 更新launch文件中的串口号
            self.update_launch_file(port)
            
            
            messagebox.showinfo("保存成功", 
                              f"串口 {port} 已保存成功\n"
                              f"配置文件: {self.config_file}\n"
                              f"Launch文件已更新")
            
        except Exception as e:
            messagebox.showerror("保存失败", f"保存串口失败:\n{str(e)}")
            print(f"保存失败: {e}")
    
    def update_launch_file(self, port):
        """更新launch文件中的串口号"""
        if not self.pkg_path or not os.path.exists(self.launch_file):
            messagebox.showwarning("警告", f"未找到launch文件: {self.launch_file}")
            return
        
        try:
            with open(self.launch_file, 'r') as f:
                content = f.read()
            
            # 更新port参数
            import re
            
            # 方法1: 使用正则表达式替换
            pattern1 = r'<arg name="port" default="[^"]*"'
            replacement1 = f'<arg name="port" default="{port}"'
            
            # 方法2: 如果第一种模式不匹配，尝试其他格式
            pattern2 = r'port:\s*["\'][^"\']*["\']'
            replacement2 = f'port: "{port}"'
            
            # 首先尝试方法1
            if re.search(pattern1, content):
                new_content = re.sub(pattern1, replacement1, content)
            elif re.search(pattern2, content):
                new_content = re.sub(pattern2, replacement2, content)
            else:
                # 如果都没有匹配，在<launch>标签后添加参数定义
                new_content = content.replace('<launch>', f'<launch>\n\n    <arg name="port" default="{port}" />')
            
            with open(self.launch_file, 'w') as f:
                f.write(new_content)
            
            print(f"已更新launch文件: {self.launch_file}")
            self.update_status(f"已更新launch文件串口为: {port}")
                
        except Exception as e:
            print(f"更新launch文件失败: {e}")
            self.update_status(f"更新launch文件失败: {e}")
    
    def load_saved_port(self):
        """从功能包config文件夹加载保存的串口"""
        if not self.pkg_path or not os.path.exists(self.config_file):
            return
        
        try:
            with open(self.config_file, 'r') as f:
                config = yaml.safe_load(f)
            
            if config and 'emergency_stop_port' in config:
                saved_port = config['emergency_stop_port']
                saved_port = self._normalize_port_path(saved_port)
                
                # 设置串口选择框
                self.port_combo.set(saved_port)
                
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
        """启动ROS launch文件"""
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
            
            # 检查launch文件是否存在
            if not os.path.exists(self.launch_file):
                messagebox.showerror("错误", f"未找到launch文件:\n{self.launch_file}")
                return
            
            # 构建roslaunch命令
            roslaunch_cmd = f'roslaunch aihitplt_hardware_test guide_deli_estop.launch'
            
            print(f"ROS启动命令: {roslaunch_cmd}")
            print(f"使用的串口: {port}")
            
            # 使用gnome-terminal打开新窗口
            cmd = [
                'gnome-terminal',
                '--title=旋钮屏幕测试 - ROS Launch',
                '--geometry=80x24+100+100',
                '--',
                'bash', '-c',
                f'source /opt/ros/noetic/setup.bash && '
                f'source ~/aihitplt_ws/devel/setup.bash && '
                f'{roslaunch_cmd}; '
                f'echo "按Enter键关闭终端..."; '
                f'read'
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
            
            # 更新按钮状态和颜色（像第二个程序一样变成绿色）
            self.launch_btn.config(text="关闭急停launch文件", bg="green", fg="white")
            self.connect_btn.config(state="disabled")
            self.refresh_btn.config(state="disabled")
            self.save_btn.config(state="disabled")
            
            self.update_status(f"正在启动ROS launch文件...")
            
            # 在子线程中等待并初始化ROS节点，避免GUI卡顿
            def delayed_ros_init():
                time.sleep(1)
                self.init_ros_node()
            
            # 启动延迟初始化线程
            init_thread = threading.Thread(target=delayed_ros_init, daemon=True)
            init_thread.start()
            
        except FileNotFoundError:
            messagebox.showerror("启动失败", "未找到gnome-terminal。")
            self.update_status("启动失败: 未找到gnome-terminal")
            
        except Exception as e:
            messagebox.showerror("启动失败", f"无法启动ROS launch文件:\n{e}")
            print(f"启动失败: {e}")
            self.update_status(f"启动失败: {e}")
    
    def init_ros_node(self):
        """初始化ROS节点 - 在子线程中执行避免卡顿"""
        def ros_init_thread():
            try:
                # 检查ROS Master是否已启动
                import rosgraph
                try:
                    rosgraph.Master('/rostopic').getPid()
                except:
                    print("等待ROS Master启动...")
                    time.sleep(2)  # 等待2秒而不是3秒
                    # 再次检查
                    try:
                        rosgraph.Master('/rostopic').getPid()
                    except:
                        print("ROS Master未启动，继续尝试初始化...")
                
                print("开始初始化ROS节点...")
                rospy.init_node('emergency_stop_gui_node', anonymous=True, disable_signals=True)
                
                # 创建话题订阅器
                self.emergency_sub = rospy.Subscriber(
                    '/e_stop',
                    Bool,
                    self.emergency_callback
                )
                
                # 等待订阅器建立连接
                time.sleep(0.5)  # 减少等待时间
                
                self.ros_node_initialized = True
                print("ROS节点初始化成功")
                self.root.after(0, lambda: self.update_status("ROS节点初始化成功，已连接话题"))
                
            except Exception as e:
                print(f"ROS节点初始化失败: {e}")
                self.root.after(0, lambda: self.update_status(f"ROS节点初始化失败: {e}"))
        
        # 在子线程中初始化ROS
        ros_thread = threading.Thread(target=ros_init_thread, daemon=True)
        ros_thread.start()
    
    def emergency_callback(self, msg):
        """急停按钮话题回调函数"""
        try:
            # 根据guide_deli_estop.py的消息格式
            if msg.data:  # True表示急停按下
                self.update_emergency_status("按下")
            else:  # False表示急停释放
                self.update_emergency_status("松开")
                
        except Exception as e:
            print(f"解析ROS消息失败: {e}")
            self.update_emergency_status("未知")
    
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
                
                # 清理ROS订阅器
                if self.emergency_sub:
                    self.emergency_sub.unregister()
                
                # 终止gnome-terminal进程
                if self.ros_pid:
                    self.kill_process_tree(self.ros_pid)
                
                # 终止roscore和相关进程
                self._kill_ros_processes()
                
            except Exception as e:
                print(f"停止ROS进程时出错: {e}")
                
            finally:
                # 更新状态和按钮颜色（恢复默认颜色）
                self.ros_running = False
                self.ros_process = None
                self.ros_pid = None
                
                # 恢复按钮状态和颜色
                self.launch_btn.config(text="启动急停launch文件", bg=self.default_bg, fg=self.default_fg)
                self.connect_btn.config(state="normal")
                self.refresh_btn.config(state="normal")
                if self.pkg_path:
                    self.save_btn.config(state="normal")    
                
                self.update_status("ROS进程已停止")
                
                # 更新按钮状态为未知
                self.update_emergency_status("未知")
                
                # 检查并清理残留的ROS进程
                self._cleanup_ros_processes()
    
    def _kill_ros_processes(self):
        """终止所有与ROS相关的进程"""
        try:
            # 更精确的终止方式：只终止特定的launch文件
            launch_files_to_kill = [
                'guide_deli_estop.launch'
            ]
            
            for launch_file in launch_files_to_kill:
                subprocess.run(['pkill', '-f', launch_file], 
                            stdout=subprocess.DEVNULL, 
                            stderr=subprocess.DEVNULL)
            
            time.sleep(1)
            
        except Exception as e:
            print(f"终止ROS进程时出错: {e}")
    
    def _cleanup_ros_processes(self):
        """清理残留的ROS进程"""
        try:
            processes_to_clean = [
                'guide_deli_estop.py',
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
    root.title("旋钮屏幕测试")
    
    # 窗口居中
    window_width = 500
    window_height = 600
    screen_width = root.winfo_screenwidth()
    screen_height = root.winfo_screenheight()
    x = (screen_width - window_width) // 2
    y = (screen_height - window_height) // 2
    root.geometry(f"{window_width}x{window_height}+{x}+{y}")
    
    # 创建应用
    app = GuideDeliveryEmergencyTestApp(root)
    
    # 运行主循环
    root.mainloop()

if __name__ == "__main__":
    main()