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
from datetime import datetime
import glob
import signal
import rospy
from std_msgs.msg import Bool

class SprayModuleTester:
    def __init__(self, root):
        self.root = root
        self.root.title("喷雾模块硬件系统测试")
        self.root.geometry("450x420")
        
        # 串口相关
        self.serial_port = None
        self.serial_connected = False
        self.serial_thread = None
        self.stop_serial_thread = False
        
        # ROS相关
        self.ros_process = None
        self.ros_running = False
        self.ros_output_thread = None
        self.spray_pub = None
        self.ros_node_initialized = False

        # 获取ROS包路径
        self.pkg_path = None
        self.config_file = None
        try:
            rospack = rospkg.RosPack()
            self.pkg_path = rospack.get_path('aihitplt_hardware_test')
            self.config_file = os.path.join(self.pkg_path, 'config', 'spray_port.yaml')
        except Exception as e:
            print(f"ROS包加载警告: {e}")
            # 尝试使用默认路径
            default_path = "/home/aihit/aihitplt_ws/src/aihitplt_hardware_test"
            if os.path.exists(default_path):
                self.pkg_path = default_path
                self.config_file = os.path.join(self.pkg_path, 'config', 'spray_port.yaml')
        
        # 保存默认按钮颜色
        self.default_button_bg = None
        self.default_button_fg = None
        
        # 创建界面
        self.create_widgets()
        
        # 加载保存的串口
        self.load_saved_port()
        
        # 绑定关闭事件
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
    
    def create_widgets(self):
        """创建界面"""
        
        # 第一部分：串口连接
        frame1 = ttk.LabelFrame(self.root, text="串口连接", padding=10)
        frame1.pack(fill="x", padx=10, pady=(10, 5))
        
        ttk.Label(frame1, text="选择串口:").grid(row=0, column=0, padx=5, pady=5, sticky="w")
        
        self.port_combo = ttk.Combobox(frame1, width=20, state="readonly")
        self.port_combo.grid(row=0, column=1, padx=5, pady=5)
        
        self.refresh_btn = ttk.Button(frame1, text="刷新串口", 
                                     command=self.refresh_ports)
        self.refresh_btn.grid(row=0, column=2, padx=5, pady=5)
        
        self.connect_btn = ttk.Button(frame1, text="连接", 
                                     command=self.toggle_serial_connection)
        self.connect_btn.grid(row=0, column=3, padx=5, pady=5)
        
        # 第二部分：喷雾控制
        frame2 = ttk.LabelFrame(self.root, text="喷雾控制", padding=10)
        frame2.pack(fill="x", padx=10, pady=5)
        
        # 配置grid列权重
        for i in range(5):
            frame2.columnconfigure(i, weight=1)
        
        # 启动喷雾按钮在第一列，向右对齐
        self.start_spray_btn = ttk.Button(
            frame2,
            text="启动喷雾",
            command=self.start_spray,
            state="disabled",
            width=12
        )
        self.start_spray_btn.grid(row=0, column=0, padx=(30, 10), pady=5, sticky="e")
        
        # 喷雾状态显示在第三列，居中
        self.spray_status_var = tk.StringVar(value="喷雾状态: 未知")
        self.spray_status_label = ttk.Label(
            frame2, 
            textvariable=self.spray_status_var,
            font=("Arial", 10, "bold"),
            width=20
        )
        self.spray_status_label.grid(row=0, column=2, padx=2, pady=5)
        
        # 中间留一个空白列（第三列）
        # 关闭喷雾按钮在第四列，向左对齐
        self.stop_spray_btn = ttk.Button(
            frame2,
            text="关闭喷雾",
            command=self.stop_spray,
            state="disabled",
            width=12
        )
        self.stop_spray_btn.grid(row=0, column=4, padx=(10, 30), pady=5, sticky="w")
        
        # 喷雾状态显示
        self.spray_status_var = tk.StringVar(value="喷雾状态: 未知")
        self.spray_status_label = ttk.Label(frame2, textvariable=self.spray_status_var)
        self.spray_status_label.grid(row=1, column=1, columnspan=2, pady=5)
        
        # 第三部分：保存串口号
        frame3 = ttk.LabelFrame(self.root, text="串口配置", padding=10)
        frame3.pack(fill="x", padx=10, pady=5)
        

        if self.pkg_path:
            self.save_btn = ttk.Button(
                frame3,
                text="保存喷雾模块硬件系统串口号",
                command=self.save_serial_port,
                width=30
            )
        else:
            self.save_btn = ttk.Button(
                frame3,
                text="保存喷雾模块硬件系统串口号 (ROS包未找到)",
                state="disabled",
                width=30
            )
        self.save_btn.pack()
        
        # 第四部分：启动ROS launch文件
        frame4 = ttk.LabelFrame(self.root, text="ROS启动", padding=10)
        frame4.pack(fill="x", padx=10, pady=5)
        
        # 创建启动按钮，使用tk.Button以便设置颜色
        # 使用相同的宽度设置
        self.launch_btn = tk.Button(
            frame4,
            text="启动喷雾模块硬件系统launch文件",
            command=self.toggle_ros_launch,
            width=28
        )
        self.launch_btn.pack()
        
        # 保存默认按钮颜色
        self.default_button_bg = self.launch_btn.cget("background")
        self.default_button_fg = self.launch_btn.cget("foreground")
        
        # 状态栏
        self.status_var = tk.StringVar(value="就绪")
        status_bar = ttk.Label(self.root, textvariable=self.status_var, 
                              relief=tk.SUNKEN, anchor=tk.W)
        status_bar.pack(side=tk.BOTTOM, fill=tk.X)
        
        # 初始化串口列表
        self.refresh_ports()
    
    def refresh_ports(self):
        """刷新串口列表"""
        
        ports = []
        
        # 1. 获取标准串口
        for port in serial.tools.list_ports.comports():
            port_path = port.device
            # 过滤掉ttyS0-ttyS31这些没用的串口
            if not any(f'ttyS{i}' in port_path for i in range(0, 32)):
                ports.append(port_path)
        
        # 2. 获取其他可能的设备（如ttyUSB, ttyACM等）
        other_ports = glob.glob('/dev/ttyUSB*') + glob.glob('/dev/ttyACM*') + glob.glob('/dev/aihitplt*')
        for port in other_ports:
            if port not in ports:
                ports.append(port)
        
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
        
        # 确保串口路径格式正确
        if not port.startswith('/dev/'):
            port = f'/dev/{port}'
        
        # 检查串口是否存在
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
              
            # 根据您的bridge.py文件，波特率使用115200
            self.serial_port = serial.Serial(
                port=port,
                baudrate=115200,  # 根据您的aihitplt_spray_bridge.py使用115200
                timeout=0.1,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE
            )
            
            self.serial_connected = True
            self.stop_serial_thread = False
            
            # 按照要求：连接串口时，刷新串口按钮禁用，保存按钮禁用，ROS启动按钮禁用
            self.connect_btn.config(text="关闭")
            self.refresh_btn.config(state="disabled")
            self.save_btn.config(state="disabled")
            self.launch_btn.config(state="disabled")
            
            # 启用喷雾控制按钮
            self.start_spray_btn.config(state="normal")
            self.stop_spray_btn.config(state="normal")
            
            # 更新喷雾状态显示
            self.spray_status_var.set("喷雾状态: 已连接")
            
            # 发送测试命令检查连接
            self.send_serial_command("test\n")
            
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
            self.connect_btn.config(text="连接")
            self.refresh_btn.config(state="normal")
            self.save_btn.config(state="normal")
            self.launch_btn.config(state="normal")
            
            # 禁用喷雾控制按钮
            self.start_spray_btn.config(state="disabled")
            self.stop_spray_btn.config(state="disabled")
            
            # 更新喷雾状态显示
            self.spray_status_var.set("喷雾状态: 已断开")
            
            self.update_status("已断开串口连接")
    
    def read_serial_data(self):
        """读取串口数据"""
        while self.serial_connected and not self.stop_serial_thread:
            try:
                if self.serial_port and self.serial_port.in_waiting > 0:
                    data = self.serial_port.readline().decode('utf-8', errors='ignore').strip()
                    if data:
                    
                        if "on" in data.lower() or "开启" in data:
                            self.spray_status_var.set("喷雾状态: 开启")
                        elif "off" in data.lower() or "关闭" in data:
                            self.spray_status_var.set("喷雾状态: 关闭")
                        
                
                time.sleep(0.01)
                
            except Exception as e:
                if not self.stop_serial_thread:
                    print(f"串口读取错误: {e}")
                break
    
    def send_serial_command(self, command):
        """通过串口发送命令"""
        if self.serial_connected and self.serial_port:
            try:
                # 根据您的bridge.py，命令应该是"on\n"或"off\n"
                self.serial_port.write(command.encode())
                print(f"发送串口命令: {command}")
                return True
            except Exception as e:
                print(f"发送命令失败: {str(e)}")
                return False
        return False
    
    def start_spray(self):
        """启动喷雾"""
        success = False
        
        if self.ros_running and self.spray_pub:
            # 使用ROS话题发布启动命令
            msg = Bool()
            msg.data = True
            self.spray_pub.publish(msg)
            success = True
            self.spray_status_var.set("喷雾状态: 开启")
        
        elif self.serial_connected:
            # 使用串口发送启动命令
            # 根据您的bridge.py，发送"on\n"命令
            success = self.send_serial_command("on\n")
            if success:
                self.update_status("已通过串口发送启动喷雾命令")
                self.spray_status_var.set("喷雾状态: 开启")
        
        if success:
            messagebox.showinfo("成功", "已发送启动喷雾命令")
        else:
            messagebox.showerror("错误", "发送启动喷雾命令失败")
    
    def stop_spray(self):
        """关闭喷雾"""
        success = False
        
        if self.ros_running and self.spray_pub:
            # 使用ROS话题发布关闭命令
            msg = Bool()
            msg.data = False
            self.spray_pub.publish(msg)
            success = True
            self.spray_status_var.set("喷雾状态: 关闭")
        
        elif self.serial_connected:
            # 使用串口发送关闭命令
            # 根据您的bridge.py，发送"off\n"命令
            success = self.send_serial_command("off\n")
            if success:
                self.spray_status_var.set("喷雾状态: 关闭") 
        
        if success:
            messagebox.showinfo("成功", "已发送关闭喷雾命令")
        else:
            messagebox.showerror("错误", "发送关闭喷雾命令失败")      
    
    def save_serial_port(self):
        """保存串口号"""
        if not self.pkg_path:
            messagebox.showerror("错误", "未找到ROS包路径")
            return
        
        port = self.port_combo.get()
        if not port:
            messagebox.showwarning("警告", "请先选择串口")
            return
        
        # 确保串口路径格式正确
        if not port.startswith('/dev/'):
            port = f'/dev/{port}'
        
        try:
            # 创建config目录
            config_dir = os.path.join(self.pkg_path, 'config')
            os.makedirs(config_dir, exist_ok=True)
            
            # 保存配置到YAML文件
            config = {
                'port': port,
                'baudrate': 115200,  # 根据您的bridge.py使用115200
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
                
                # 修复可能存在的路径问题
                if saved_port.startswith('//dev/'):
                    saved_port = saved_port.replace('//dev/', '/dev/')
                elif not saved_port.startswith('/dev/'):
                    saved_port = f'/dev/{saved_port}'
                
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
        """启动ROS launch文件"""
        try:
            # 获取当前环境变量
            env = os.environ.copy()
            
            # 设置串口参数（如果选择了串口）
            port = self.port_combo.get()
            if port:
                env['SPRAY_PORT'] = port
            
            # 启动ROS launch文件
            cmd = ['roslaunch', 'aihitplt_spray', 'aihitplt_spray.launch']
            
            print(f"启动命令: {' '.join(cmd)}")
            if port:
                print(f"使用的串口: {port}")
            
            self.ros_process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
                universal_newlines=True,
                env=env,
                preexec_fn=os.setsid
            )
            
            self.ros_running = True
            
            # 按照要求：启动ROS时，按钮变绿并改变文本
            self.launch_btn.config(
                text="关闭喷雾模块硬件系统launch文件",
                bg="green",
                fg="white"
            )
            
            # 按照要求：启动ROS时，串口连接按钮禁用
            self.connect_btn.config(state="disabled")
            self.refresh_btn.config(state="disabled")
            self.save_btn.config(state="disabled")
            
            # 启用喷雾控制按钮（通过ROS话题）
            self.start_spray_btn.config(state="normal")
            self.stop_spray_btn.config(state="normal")
            
            # 启动ROS输出监控线程
            self.ros_output_thread = threading.Thread(
                target=self.monitor_ros_output,
                daemon=True
            )
            self.ros_output_thread.start()
            
            # 延迟初始化ROS发布者
            self.root.after(3000, self.init_ros_publisher)
            
            self.update_status("ROS launch文件已启动")
            
        except Exception as e:
            messagebox.showerror("启动失败", f"无法启动ROS launch文件:\n{e}")
            print(f"启动失败: {e}")
            self.update_status(f"启动失败: {e}")
    
    def init_ros_publisher(self):
        """初始化ROS发布者"""
        if not self.ros_running:
            return
        
        try:
            # 初始化ROS节点
            rospy.init_node('spray_module_tester_gui', anonymous=True, disable_signals=True)
            self.ros_node_initialized = True
            
            # 修改：创建发布者 - 发布到'spray_control'话题（去掉斜杠，以匹配bridge的订阅）
            self.spray_pub = rospy.Publisher('spray_control', Bool, queue_size=10)
            
            # 等待连接
            time.sleep(1)
            
            print("ROS发布者初始化完成")
            self.update_status("ROS发布者已初始化，可通过话题控制喷雾")
            
        except Exception as e:
            print(f"ROS发布者初始化失败: {e}")
            self.update_status(f"ROS发布者初始化失败: {e}")
    
    def monitor_ros_output(self):
        """监控ROS进程输出"""
        if self.ros_process:
            try:
                for line in iter(self.ros_process.stdout.readline, ''):
                    if line:
                        line = line.strip()
                        print(f"[ROS输出] {line}")
                        
                        # 从ROS输出中解析状态信息
                        if "spray_status" in line:
                            # 解析状态消息
                            if "on" in line.lower() or "true" in line:
                                self.root.after(0, lambda: self.spray_status_var.set("喷雾状态: 开启"))
                            elif "off" in line.lower() or "false" in line:
                                self.root.after(0, lambda: self.spray_status_var.set("喷雾状态: 关闭"))
                        
                        if not self.ros_running:
                            break
            except Exception as e:
                print(f"ROS输出监控错误: {e}")
                pass
            
            # 进程结束后清理
            if self.ros_running:
                self.root.after(0, self.ros_process_terminated)
    
    def ros_process_terminated(self):
        """ROS进程终止后的处理"""
        self.ros_running = False
        self.ros_process = None
        self.ros_node_initialized = False
        self.spray_pub = None
        
        # 恢复按钮状态
        self.launch_btn.config(
            text="启动喷雾模块硬件系统launch文件",
            bg=self.default_button_bg,
            fg=self.default_button_fg
        )
        self.connect_btn.config(state="normal")
        self.refresh_btn.config(state="normal")
        self.save_btn.config(state="normal")
        
        # 如果串口未连接，禁用喷雾控制按钮
        if not self.serial_connected:
            self.start_spray_btn.config(state="disabled")
            self.stop_spray_btn.config(state="disabled")
        
        # 更新喷雾状态显示
        self.spray_status_var.set("喷雾状态: ROS已停止")
        
        self.update_status("ROS进程已停止")
    
    def stop_ros_launch(self):
        """停止ROS launch文件"""
        if self.ros_running and self.ros_process:
                
            # 发送SIGTERM信号给整个进程组
            try:
                os.killpg(os.getpgid(self.ros_process.pid), signal.SIGTERM)
            except (ProcessLookupError, AttributeError):
                pass
                
            # 等待进程结束
            try:
                self.ros_process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(os.getpgid(self.ros_process.pid), signal.SIGKILL)
                except:
                    pass
                self.ros_process.wait(timeout=1)
            
            finally:
                # 更新状态
                self.ros_running = False
                self.ros_process = None
                self.ros_node_initialized = False
                self.spray_pub = None
                
                # 恢复按钮状态
                self.launch_btn.config(
                    text="启动喷雾模块硬件系统launch文件",
                    bg=self.default_button_bg,
                    fg=self.default_button_fg
                )
                self.connect_btn.config(state="normal")
                self.refresh_btn.config(state="normal")
                self.save_btn.config(state="normal")
                
                # 如果串口未连接，禁用喷雾控制按钮
                if not self.serial_connected:
                    self.start_spray_btn.config(state="disabled")
                    self.stop_spray_btn.config(state="disabled")
                
                # 更新喷雾状态显示
                self.spray_status_var.set("喷雾状态: ROS已停止")
    
    def cleanup_resources(self):
        """清理所有资源"""
        
        # 停止串口连接
        if self.serial_connected:
            self.disconnect_serial()
        
        # 停止ROS进程
        if self.ros_running:
            self.stop_ros_launch()
        
        # 关闭ROS节点
        if self.ros_node_initialized:
            try:
                rospy.signal_shutdown("程序退出")
            except:
                pass
    
    def update_status(self, message):
        """更新状态栏"""
        timestamp = datetime.now().strftime("%H:%M:%S")
        self.status_var.set(f"[{timestamp}] {message}")
        print(f"[状态] {message}")
    
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
    # 设置信号处理器
    def sigint_handler(signum, frame):
        print("\nCtrl+C received, shutting down...")
        import sys
        sys.exit(0)
    
    signal.signal(signal.SIGINT, sigint_handler)
    
    # 设置环境
    if 'DISPLAY' not in os.environ:
        os.environ['DISPLAY'] = ':0'
    os.environ['QT_X11_NO_MITSHM'] = '1'

    root = tk.Tk()
    
    # 窗口居中
    window_width = 500
    window_height = 500
    screen_width = root.winfo_screenwidth()
    screen_height = root.winfo_screenheight()
    x = (screen_width - window_width) // 2
    y = (screen_height - window_height) // 2
    root.geometry(f"{window_width}x{window_height}+{x}+{y}")
    
    app = SprayModuleTester(root)
    
    # 运行主循环
    root.mainloop()

if __name__ == "__main__":
    main()