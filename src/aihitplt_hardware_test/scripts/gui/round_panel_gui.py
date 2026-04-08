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
import rospkg
import glob
import psutil
import struct
from datetime import datetime
import rospy
from std_msgs.msg import Bool

class RoundPanelTester:
    def __init__(self, root):
        self.root = root
        self.root.title("旋钮屏幕测试")
        self.root.geometry("450x500")
        
        # 串口相关变量
        self.serial_port = None
        self.serial_connected = False
        self.serial_thread = None
        self.stop_serial_thread = False
        
        # ROS相关变量
        self.ros_process = None
        self.ros_running = False
        self.ros_pid = None
        self.ros_node_initialized = False
        
        # ROS话题相关
        self.touch_sub = None
        self.TOUCH_TOPIC = "/round_panel"
        
        # 触摸状态
        self.touch_state = "未知"
        self.last_press_time = 0
        
        # ROS包路径
        self.pkg_path = None
        self.config_file = None
        self._init_ros_path()
        
        # 创建界面
        self.create_widgets()
        
        # 加载保存的串口
        self.load_saved_port()
        
        # 绑定关闭事件
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
    
    def _init_ros_path(self):
        """初始化ROS包路径"""
        try:
            rospack = rospkg.RosPack()
            self.pkg_path = rospack.get_path('aihitplt_hardware_test')
            self.config_file = os.path.join(self.pkg_path, 'config', 'knob_port.yaml')
            print(f"找到ROS包路径: {self.pkg_path}")
        except Exception as e:
            print(f"ROS包加载警告: {e}")
    
    def create_widgets(self):
        """创建界面组件"""
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.pack(fill=tk.BOTH, expand=True)
        
        # ========== 第一部分：串口连接 ==========
        frame1 = ttk.LabelFrame(main_frame, text="设备连接", padding=10)
        frame1.pack(fill=tk.X, pady=(0, 10))
        
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
        
        # ========== 第二部分：触摸状态显示 ==========
        frame2 = ttk.LabelFrame(main_frame, text="触摸状态", padding=10)
        frame2.pack(fill=tk.BOTH, expand=True, pady=(0, 10))
        
        # 触摸状态按钮
        self.touch_status_btn = tk.Button(
            frame2,
            text="未知",
            width=20,
            height=3,
            state="disabled",
            font=("Arial", 12),
            bg="gray",
            fg="white"
        )
        self.touch_status_btn.pack(expand=True)
        
        # 状态信息标签
        self.status_info = ttk.Label(frame2, text="等待连接...", foreground="gray")
        self.status_info.pack(pady=5)
        
        # ========== 第三部分：保存设备号 ==========
        frame3 = ttk.LabelFrame(main_frame, text="设备配置", padding=10)
        frame3.pack(fill=tk.X, pady=(0, 10))
        
        self.save_btn = ttk.Button(
            frame3,
            text="保存设备号",
            command=self.save_serial_port,
            width=28  # 与安防传感器程序保持一致
        )
        self.save_btn.pack()
        
        # ========== 第四部分：ROS启动 ==========
        frame4 = ttk.LabelFrame(main_frame, text="ROS启动", padding=10)
        frame4.pack(fill=tk.X)
        
        # 创建启动按钮 - 大小与保存按钮一致
        self.launch_btn = tk.Button(
            frame4,
            text="启动launch文件",
            command=self.toggle_ros_launch,
            width=26,  # 与安防传感器程序保持一致
            bg="lightgray",
            fg="black"
        )
        self.launch_btn.pack()
        
        # ========== 状态栏 ==========
        self.status_var = tk.StringVar(value="就绪")
        status_bar = ttk.Label(main_frame, textvariable=self.status_var, 
                              relief=tk.SUNKEN, anchor=tk.W)
        status_bar.pack(side=tk.BOTTOM, fill=tk.X, pady=(10, 0))
        
        # 初始化设备列表
        self.refresh_ports()
        
        # 初始状态
        self.update_button_states()
    
    def refresh_ports(self):
        """刷新设备列表"""
        # 保存当前选择的值
        current_selection = self.port_combo.get()
        
        devices = []
        
        # 获取标准串口（过滤ttyS0-ttyS31）
        for p in serial.tools.list_ports.comports():
            if not any(f'ttyS{i}' in p.device for i in range(32)):
                devices.append(p.device)
        
        # 添加其他设备
        other_devices = glob.glob('/dev/ttyUSB*') + glob.glob('/dev/ttyACM*') + glob.glob('/dev/aihitplt*')
        for device in other_devices:
            if device not in devices:
                devices.append(device)
        
        devices.sort()
        
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
        """切换串口连接状态"""
        if not self.serial_connected:
            self.connect_serial()
        else:
            self.disconnect_serial()
    
    def connect_serial(self):
        """连接设备"""
        port = self.port_combo.get()
        if not port:
            messagebox.showwarning("警告", "请选择设备")
            return
        
        try:
            # 先关闭可能存在的连接
            if self.serial_port:
                try:
                    self.serial_port.close()
                except:
                    pass
            
            # 连接到设备
            self.serial_port = serial.Serial(
                port=port,
                baudrate=115200,
                timeout=0.1
            )
            
            self.serial_connected = True
            self.stop_serial_thread = False
            
            # 更新按钮状态
            self.connect_btn.config(text="关闭")
            self.refresh_btn.config(state="disabled")
            self.launch_btn.config(state="disabled")
            self.save_btn.config(state="disabled")
            
            # 重置触摸状态
            self.touch_state = "未知"
            self.last_press_time = 0
            self.update_touch_display()
            self.status_info.config(text="已连接，等待触摸...")
            
            # 启动串口读取线程
            self.serial_thread = threading.Thread(
                target=self.read_serial_data,
                daemon=True
            )
            self.serial_thread.start()
            
            self.update_status(f"已连接到设备: {port}")
            
        except Exception as e:
            messagebox.showerror("连接失败", f"无法连接设备:\n{e}")
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
            self.launch_btn.config(state="normal")
            self.save_btn.config(state="normal")
            
            # 重置触摸状态
            self.touch_state = "未知"
            self.update_touch_display()
            self.status_info.config(text="已断开连接")
            
            self.update_status("已断开设备连接")
    
    def read_serial_data(self):
        """读取串口数据"""
        while self.serial_connected and not self.stop_serial_thread:
            try:
                # 读取数据
                if self.serial_port and self.serial_port.in_waiting >= 2:
                    data = self.serial_port.read(2)
                    
                    if len(data) == 2:
                        value = struct.unpack('>H', data)[0]
                        
                        # 处理点击事件
                        if value == 0x0001:
                            current_time = time.time()
                            
                            # 防抖动：至少间隔0.3秒
                            if current_time - self.last_press_time > 0.3:
                                self.last_press_time = current_time
                                self.root.after(0, self.handle_press_event)
                
                time.sleep(0.01)
                
            except Exception as e:
                if not self.stop_serial_thread:
                    print(f"设备读取错误: {e}")
                    self.root.after(0, self.disconnect_serial)
                break
    
    def handle_press_event(self):
        """处理按下事件"""
        # 设置触摸状态为按下
        self.touch_state = "按下"
        self.update_touch_display()
        
        # 更新状态信息
        self.status_info.config(text="触摸按下")
        
        # 0.5秒后自动松开
        self.root.after(500, self.auto_release)
    
    def auto_release(self):
        """自动松开"""
        self.touch_state = "松开"
        self.update_touch_display()
        self.status_info.config(text="已松开")
    
    def update_touch_display(self):
        """更新触摸状态显示"""
        display_state = self.touch_state
        self.touch_status_btn.config(text=display_state)
        
        if display_state == "按下":
            self.touch_status_btn.config(bg="green", fg="white")
        else:
            self.touch_status_btn.config(bg="gray", fg="white")
    
    def init_ros_node(self):
        """初始化ROS节点"""
        try:
            if not rospy.is_shutdown():
                # 初始化节点
                rospy.init_node('round_panel_tester_gui', anonymous=True, disable_signals=True)
                
                # 订阅触摸话题
                self.touch_sub = rospy.Subscriber(
                    self.TOUCH_TOPIC,
                    Bool,
                    self.ros_touch_callback
                )
                
                self.ros_node_initialized = True
                print("ROS节点初始化成功")
                self.update_status("ROS节点初始化成功，已订阅触摸话题")
                
                return True
                
        except Exception as e:
            print(f"ROS节点初始化失败: {e}")
            self.update_status(f"ROS节点初始化失败: {e}")
            return False
    
    def ros_touch_callback(self, msg):
        """ROS触摸话题回调函数"""
        try:
            # 处理触摸消息
            if msg.data:  # 如果收到True消息
                current_time = time.time()
                
                # 防抖动：至少间隔0.3秒
                if current_time - self.last_press_time > 0.3:
                    self.last_press_time = current_time
                    self.root.after(0, self.handle_press_event)
                    
        except Exception as e:
            print(f"ROS话题数据处理失败: {e}")
    
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
    
    def save_serial_port(self):
        """保存设备号到配置文件"""
        if not self.pkg_path:
            messagebox.showerror("错误", "未找到ROS包路径")
            return
        
        port = self.port_combo.get()
        if not port:
            messagebox.showwarning("警告", "请先选择设备")
            return
        
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
                
            messagebox.showinfo("保存成功",
                                f"设备 {port} 已保存成功\n" 
                                f"配置文件: {self.config_file}")
            
            self.update_status(f"已保存设备: {port}")
            
        except Exception as e:
            messagebox.showerror("保存失败", f"保存设备失败:\n{str(e)}")
    
    def load_saved_port(self):
        """加载保存的设备"""
        if not self.pkg_path or not os.path.exists(self.config_file):
            return
        
        try:
            with open(self.config_file, 'r') as f:
                config = yaml.safe_load(f)
            
            if config and 'port' in config:
                saved_port = config['port']
                self.port_combo.set(saved_port)
                self.update_status(f"已加载保存的设备: {saved_port}")
                
        except Exception as e:
            print(f"加载保存的设备失败: {e}")
    
    def toggle_ros_launch(self):
        """切换ROS launch文件"""
        if not self.ros_running:
            self.start_ros_launch()
        else:
            self.stop_ros_launch()
    
    def start_ros_launch(self):
        """启动ROS launch文件"""
        try:
            # 获取当前设备
            port = self.port_combo.get()
            if not port:
                messagebox.showwarning("警告", "请先选择设备")
                return
            
            # 构建roslaunch命令
            roslaunch_cmd = f'roslaunch aihitplt_hardware_test aihitplt_round_panel.launch'
            
            print(f"启动命令: {roslaunch_cmd}")
            print(f"使用设备: {port}")
            
            # 使用gnome-terminal打开新窗口
            cmd = [
                'gnome-terminal',
                '--title=旋钮屏幕 - ROS Launch',
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
            
            # 重置触摸状态
            self.touch_state = "未知"
            self.update_touch_display()
            self.status_info.config(text="ROS已启动，等待触摸话题...")
            
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
                    if self.touch_sub:
                        self.touch_sub.unregister()
                    
                    # 关闭节点
                    try:
                        rospy.signal_shutdown("GUI关闭")
                    except:
                        pass
                    
                    self.ros_node_initialized = False
                
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
                
                # 恢复按钮状态
                self.launch_btn.config(
                    text="启动launch文件",
                    bg="lightgray",
                    fg="black"
                )
                self.connect_btn.config(state="normal")
                self.refresh_btn.config(state="normal")
                self.save_btn.config(state="normal")
                
                # 重置触摸状态
                self.touch_state = "未知"
                self.update_touch_display()
                self.status_info.config(text="ROS已停止")
                
                self.update_status("ROS进程已停止")
    
    def _kill_ros_processes(self):
        """终止所有与ROS相关的进程"""
        try:
            # 更精确的终止方式：只终止特定的launch文件
            launch_files_to_kill = [
                'aihitplt_round_panel.launch'
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
        # 停止串口连接
        if self.serial_connected:
            self.disconnect_serial()
        
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
    
    root = tk.Tk()
    
    # 设置窗口大小和位置
    window_width = 500
    window_height = 400
    screen_width = root.winfo_screenwidth()
    screen_height = root.winfo_screenheight()
    x = (screen_width - window_width) // 2
    y = (screen_height - window_height) // 2
    root.geometry(f"{window_width}x{window_height}+{x}+{y}")
    
    # 创建应用
    app = RoundPanelTester(root)
    
    # 运行主循环
    root.mainloop()

if __name__ == "__main__":
    main()