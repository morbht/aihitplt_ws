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
import queue
import json
import struct
from datetime import datetime
import rospy
from std_msgs.msg import String

class SecuritySensorTester:
    def __init__(self, root):
        self.root = root
        self.root.title("安防模块传感器系统测试")
        self.root.geometry("450x540")
        
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
        self.sensor_sub = None
        self.SENSOR_TOPIC = "/security_sensors"
        
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
            self.config_file = os.path.join(self.pkg_path, 'config', 'security_sensors_port.yaml')
            print(f"找到ROS包路径: {self.pkg_path}")
        except Exception as e:
            print(f"ROS包加载警告: {e}")
    
    def create_widgets(self):
        """创建界面组件"""
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.pack(fill=tk.BOTH, expand=True)
        
        # ========== 第一部分：串口连接 ==========
        frame1 = ttk.LabelFrame(main_frame, text="串口连接", padding=10)
        frame1.pack(fill=tk.X, pady=(0, 10))
        
        # 串口选择部分 - 单行布局
        port_select_frame = ttk.Frame(frame1)
        port_select_frame.pack(fill=tk.X, pady=5)
        
        ttk.Label(port_select_frame, text="串口:").pack(side=tk.LEFT, padx=(0, 5))
        
        # 缩短串口选择栏
        self.port_combo = ttk.Combobox(port_select_frame, width=25, state="readonly")
        self.port_combo.pack(side=tk.LEFT, padx=(0, 15))
        
        # 刷新按钮 - 与串口选择在同一行
        self.refresh_btn = ttk.Button(
            port_select_frame, 
            text="刷新", 
            command=self.refresh_ports,
            width=8
        )
        self.refresh_btn.pack(side=tk.LEFT, padx=(0, 15))
        
        # 连接按钮 - 与串口选择在同一行
        self.connect_btn = ttk.Button(
            port_select_frame, 
            text="连接",
            command=self.toggle_serial_connection,
            width=8
        )
        self.connect_btn.pack(side=tk.LEFT)
        
        # ========== 第二部分：传感器数据显示 ==========
        frame2 = ttk.LabelFrame(main_frame, text="传感器数据", padding=10)
        frame2.pack(fill=tk.BOTH, expand=True, pady=(0, 10))
        
        # 创建传感器数据显示区域
        self.create_sensor_display(frame2)
        
        # ========== 第三部分：保存串口号 ==========
        frame3 = ttk.LabelFrame(main_frame, text="串口配置", padding=10)
        frame3.pack(fill=tk.X, pady=(0, 10))
        
        self.save_btn = ttk.Button(
            frame3,
            text="保存串口号",
            command=self.save_serial_port,
            width=28
        )
        self.save_btn.pack()
        
        # ========== 第四部分：ROS启动 ==========
        frame4 = ttk.LabelFrame(main_frame, text="ROS启动", padding=10)
        frame4.pack(fill=tk.X)
        
        # 创建启动按钮 - 使用tk.Button以便设置颜色
        self.launch_btn = tk.Button(
            frame4,
            text="启动launch文件",
            command=self.toggle_ros_launch,
            width=26,
            bg="lightgray",  # 使用标准灰色
            fg="black"
        )
        self.launch_btn.pack()
        
        # ========== 状态栏 ==========
        self.status_var = tk.StringVar(value="就绪")
        status_bar = ttk.Label(main_frame, textvariable=self.status_var, 
                              relief=tk.SUNKEN, anchor=tk.W)
        status_bar.pack(side=tk.BOTTOM, fill=tk.X, pady=(10, 0))
        
        # 初始化串口列表
        self.refresh_ports()
        
        # 初始状态
        self.update_button_states()
    
    def create_sensor_display(self, parent):
        """创建传感器数据显示区域"""
        # 传感器列表
        sensors = [
            ("酒精传感器:", "alcohol"),
            ("烟雾传感器:", "smoke"),
            ("光照强度:", "light"),
            ("声音强度:", "sound"),
            ("急停状态:", "emergency_stop"),
            ("CO2浓度:", "eCO2"),
            ("甲醛浓度:", "eCH2O"),
            ("TVOC浓度:", "TVOC"),
            ("PM2.5:", "PM25"),
            ("PM10:", "PM10"),
            ("温度:", "temperature"),
            ("湿度:", "humidity")
        ]
        
        # 创建标签显示传感器数据
        self.sensor_labels = {}
        
        for i, (name, key) in enumerate(sensors):
            row = i // 2
            col = (i % 2) * 2
            
            # 传感器名称标签
            name_label = ttk.Label(parent, text=name, font=("Arial", 10))
            name_label.grid(row=row, column=col, padx=(10, 5), pady=5, sticky=tk.W)
            
            # 传感器值标签
            value_label = ttk.Label(parent, text="--", font=("Arial", 10, "bold"), width=12)
            value_label.grid(row=row, column=col+1, padx=(0, 10), pady=5, sticky=tk.W)
            
            self.sensor_labels[key] = value_label
        
        # 配置网格权重
        for i in range(6):  # 6行
            parent.grid_rowconfigure(i, weight=1)
        for i in range(4):  # 4列
            parent.grid_columnconfigure(i, weight=1)
    
    def refresh_ports(self):
        # 保存当前选择的值
        current_selection = self.port_combo.get()
        
        ports = []
        for p in serial.tools.list_ports.comports():
            if not any(f'ttyS{i}' in p.device for i in range(32)):
                ports.append(p.device)
        ports += [p for p in glob.glob('/dev/ttyUSB*') + glob.glob('/dev/ttyACM*') + glob.glob('/dev/aihitplt*') if p not in ports]
        ports.sort()
        
        # 更新下拉框的值
        self.port_combo['values'] = ports
        
        # 如果之前有选择的值且该值仍然存在，保持选择
        if current_selection and current_selection in ports:
            self.port_combo.set(current_selection)
        elif ports:
            # 否则选择第一个
            self.port_combo.current(0)
        
        self.update_status(f"找到 {len(ports)} 个串口")
    
    def toggle_serial_connection(self):
        """切换串口连接状态"""
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
        
        try:
            # 先关闭可能存在的连接
            if self.serial_port:
                try:
                    self.serial_port.close()
                except:
                    pass
            
            # 连接到串口
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
            
            # 启动串口读取线程
            self.serial_thread = threading.Thread(
                target=self.read_serial_data,
                daemon=True
            )
            self.serial_thread.start()
            
            self.update_status(f"已连接到串口: {port}")
            
        except Exception as e:
            messagebox.showerror("连接失败", f"无法连接串口:\n{e}")
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
            self.connect_btn.config(text="连接")
            self.refresh_btn.config(state="normal")
            self.launch_btn.config(state="normal")
            self.save_btn.config(state="normal")
            
            # 重置传感器显示
            self.reset_sensor_display()
            
            self.update_status("已断开串口连接")
    
    def read_serial_data(self):
        """读取串口数据"""
        # 安防传感器的帧格式
        FRAME_FORMAT = '<2sI 4H B 5H 2f B'
        FRAME_SIZE = 34
        FRAME_HEADER = b'\xAA\x55'
        
        buffer = bytearray()
        
        while self.serial_connected and not self.stop_serial_thread:
            try:
                # 读取数据
                if self.serial_port and self.serial_port.in_waiting > 0:
                    data = self.serial_port.read(self.serial_port.in_waiting)
                    buffer.extend(data)
                
                # 解析完整帧
                while len(buffer) >= FRAME_SIZE:
                    # 查找帧头
                    if buffer[0:2] != FRAME_HEADER:
                        buffer.pop(0)
                        continue
                    
                    # 提取完整帧
                    if len(buffer) >= FRAME_SIZE:
                        frame = bytes(buffer[:FRAME_SIZE])
                        
                        try:
                            # 解析数据
                            data = struct.unpack(FRAME_FORMAT, frame)
                            
                            # 校验和验证
                            checksum = sum(frame[:-1]) & 0xFF
                            if checksum == data[14]:
                                # 提取传感器数据
                                sensor_data = {
                                    'alcohol': int(data[2]),
                                    'smoke': int(data[3]),
                                    'light': int(data[4]),
                                    'sound': int(data[5]),
                                    'emergency_stop': int(data[6]),
                                    'eCO2': int(data[7]),
                                    'eCH2O': float(data[8]/100.0),
                                    'TVOC': float(data[9]/100.0),
                                    'PM25': int(data[10]),
                                    'PM10': int(data[11]),
                                    'temperature': float(data[12]),
                                    'humidity': float(data[13])
                                }
                                
                                # 在主线程中更新显示
                                self.root.after(0, lambda sd=sensor_data: self.update_sensor_display(sd))
                            
                            # 移除已处理的帧
                            buffer = buffer[FRAME_SIZE:]
                            
                        except struct.error as e:
                            buffer.pop(0)
                            print(f"帧解析错误: {e}")
                    else:
                        break
                
                time.sleep(0.01)
                
            except Exception as e:
                if not self.stop_serial_thread:
                    print(f"串口读取错误: {e}")
                    self.root.after(0, self.disconnect_serial)
                break
    
    def init_ros_node(self):
        """初始化ROS节点"""
        try:
            if not rospy.is_shutdown():
                # 初始化节点
                rospy.init_node('security_sensor_tester_gui', anonymous=True, disable_signals=True)
                
                # 订阅传感器话题
                self.sensor_sub = rospy.Subscriber(
                    self.SENSOR_TOPIC,
                    String,
                    self.ros_sensor_callback
                )
                
                self.ros_node_initialized = True
                print("ROS节点初始化成功")
                self.update_status("ROS节点初始化成功，已订阅传感器话题")
                
                return True
                
        except Exception as e:
            print(f"ROS节点初始化失败: {e}")
            self.update_status(f"ROS节点初始化失败: {e}")
            return False
    
    def ros_sensor_callback(self, msg):
        """ROS传感器话题回调函数"""
        try:
            # 解析JSON数据
            sensor_data = json.loads(msg.data)
            
            # 在主线程中更新显示
            self.root.after(0, lambda sd=sensor_data: self.update_sensor_display(sd))
            
        except Exception as e:
            print(f"ROS话题数据解析失败: {e}")
    
    def update_sensor_display(self, data):
        """更新传感器显示"""
        try:
            # 更新各个传感器显示
            self.sensor_labels['alcohol'].config(text=str(data.get('alcohol', '--')))
            self.sensor_labels['smoke'].config(text=str(data.get('smoke', '--')))
            self.sensor_labels['light'].config(text=str(data.get('light', '--')))
            self.sensor_labels['sound'].config(text=str(data.get('sound', '--')))
            
            # 急停状态特殊显示
            estop = data.get('emergency_stop', 1)
            if estop == 0:
                self.sensor_labels['emergency_stop'].config(text="触发", foreground="red")
            else:
                self.sensor_labels['emergency_stop'].config(text="正常", foreground="green")
            
            # 其他传感器
            self.sensor_labels['eCO2'].config(text=f"{data.get('eCO2', '--')} ppm")
            self.sensor_labels['eCH2O'].config(text=f"{data.get('eCH2O', '--'):.2f} mg/m³")
            self.sensor_labels['TVOC'].config(text=f"{data.get('TVOC', '--'):.2f} mg/m³")
            self.sensor_labels['PM25'].config(text=f"{data.get('PM25', '--')} μg/m³")
            self.sensor_labels['PM10'].config(text=f"{data.get('PM10', '--')} μg/m³")
            self.sensor_labels['temperature'].config(text=f"{data.get('temperature', '--'):.1f} °C")
            self.sensor_labels['humidity'].config(text=f"{data.get('humidity', '--'):.1f} %")
            
        except Exception as e:
            print(f"更新显示错误: {e}")
    
    def reset_sensor_display(self):
        """重置传感器显示为默认值"""
        for key, label in self.sensor_labels.items():
            label.config(text="--", foreground="black")
        
        # 急停状态恢复默认
        self.sensor_labels['emergency_stop'].config(text="--", foreground="black")
    
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
        """保存串口号到配置文件"""
        if not self.pkg_path:
            messagebox.showerror("错误", "未找到ROS包路径")
            return
        
        port = self.port_combo.get()
        if not port:
            messagebox.showwarning("警告", "请先选择串口")
            return
        
        try:
            # 创建config目录
            config_dir = os.path.join(self.pkg_path, 'config')
            os.makedirs(config_dir, exist_ok=True)
            
            # 保存配置到YAML文件
            config = {
                'security_sensor_port': port,
                'baudrate': 115200,
                'last_updated': datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            }
            
            with open(self.config_file, 'w') as f:
                yaml.dump(config, f, default_flow_style=False)
                
            messagebox.showinfo("保存成功",
                                f"串口 {port} 已保存成功\n" 
                                f"配置文件: {self.config_file}\n"
                               )


            self.update_status(f"已保存串口: {port}")
            
        except Exception as e:
            messagebox.showerror("保存失败", f"保存串口失败:\n{str(e)}")
    
    def load_saved_port(self):
        """加载保存的串口"""
        if not self.pkg_path or not os.path.exists(self.config_file):
            return
        
        try:
            with open(self.config_file, 'r') as f:
                config = yaml.safe_load(f)
            
            if config and 'security_sensor_port' in config:
                saved_port = config['security_sensor_port']
                self.port_combo.set(saved_port)
                self.update_status(f"已加载保存的串口: {saved_port}")
                
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
            # 获取当前串口
            port = self.port_combo.get()
            if not port:
                messagebox.showwarning("警告", "请先选择串口")
                return
            
            # 构建roslaunch命令
            roslaunch_cmd = f'roslaunch aihitplt_hardware_test aihitplt_security_sensors.launch'
            
            print(f"启动命令: {roslaunch_cmd}")
            print(f"使用串口: {port}")
            
            # 使用gnome-terminal打开新窗口
            cmd = [
                'gnome-terminal',
                '--title=安防模块传感器系统 - ROS Launch',
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
                    if self.sensor_sub:
                        self.sensor_sub.unregister()
                    
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
                
                # 恢复按钮状态 - 按钮恢复原色
                self.launch_btn.config(
                    text="启动launch文件",
                    bg="lightgray",
                    fg="black"
                )
                self.connect_btn.config(state="normal")
                self.refresh_btn.config(state="normal")
                self.save_btn.config(state="normal")
                
                # 重置传感器显示
                self.reset_sensor_display()
                
                self.update_status("ROS进程已停止")
    
    def _kill_ros_processes(self):
        """终止所有与ROS相关的进程"""
        try:
            processes_to_kill = ['aihitplt_security_sensors.launch']
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
    window_width = 600
    window_height = 500
    screen_width = root.winfo_screenwidth()
    screen_height = root.winfo_screenheight()
    x = (screen_width - window_width) // 2
    y = (screen_height - window_height) // 2
    root.geometry(f"{window_width}x{window_height}+{x}+{y}")
    
    # 创建应用
    app = SecuritySensorTester(root)
    
    # 运行主循环
    root.mainloop()

if __name__ == "__main__":
    main()