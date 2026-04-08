#!/usr/bin/env python3
import tkinter as tk
from tkinter import ttk
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib
import matplotlib.font_manager as fm
from matplotlib.patches import Ellipse
import rospy
import numpy as np
from collections import deque
import threading
import time
import signal
import sys
from aihitplt_hardware_test.msg import supersonic
from std_msgs.msg import Float32, Bool, UInt32

matplotlib.rcParams['font.sans-serif'] = ['WenQuanYi Micro Hei', 'WenQuanYi Zen Hei', 'Noto Sans CJK SC', 'Noto Serif CJK SC', 'AR PL UMing CN', 'AR PL UKai CN', 'Droid Sans Fallback', 'DejaVu Sans', 'Arial']
matplotlib.rcParams['axes.unicode_minus'] = False

class UltrasonicVisualization:
    def __init__(self, root):
        self.root = root
        self.root.title("传感器监控界面")
        self.root.geometry("700x500")
        self.root.configure(bg='white')
        
        # 初始化数据存储
        self.max_distance = 3.0
        self.time_window = 50
        self.sensor_names = ['A', 'B', 'C', 'D', 'E', 'F']
        self.sensor_labels = {'A': '左前方', 'B': '右前方', 'C': '左边', 'D': '右边', 'E': '左后方', 'F': '右后方'}
        
        # 数据队列
        self.distances = {name: deque([0.0] * self.time_window) for name in self.sensor_names}
        self.ir_distance = deque([0.0] * self.time_window)
        self.latest_ir_data = 0.0
        
        # 指示灯状态
        self.collision_state = False
        self.emergency_stop_state = False
        self.user_button_state = False
        
        # 最新数据
        self.latest_data = {name: 0.0 for name in self.sensor_names}
        self.data_count = 0
        self.ir_data_count = 0
        self.running = True
        
        # 创建主框架
        self.main_frame = ttk.Frame(root)
        self.main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        style = ttk.Style()
        style.configure('White.TFrame', background='white')
        self.main_frame.configure(style='White.TFrame')

        # 创建波形图和指示灯的组合布局
        self.setup_combined_plots()
        
        # 在主线程中初始化ROS
        self.setup_ros()
        
        # 设置Ctrl+C处理
        self.setup_signal_handlers()
        
        # 启动Tkinter定时器更新图形
        self.root.after(100, self.update_plots_timer)
        
    def setup_signal_handlers(self):
        def signal_handler(sig, frame):
            self.cleanup()
            sys.exit(0)
        signal.signal(signal.SIGINT, signal_handler)
        
    def setup_combined_plots(self):
        plot_frame = ttk.Frame(self.main_frame)
        plot_frame.pack(fill=tk.BOTH, expand=True, pady=10)
        
        self.fig = plt.figure(figsize=(6, 6), facecolor='white')
        gs = plt.GridSpec(3, 6, figure=self.fig)
        
        self.axes = []
        self.lines = []
        self.distance_texts = []
        
        # 设置6个超声波传感器图表
        positions = [(0, 0), (0, 1), (0, 2), (1, 0), (1, 1), (1, 2)]
        
        for i, (row, col) in enumerate(positions):
            ax = self.fig.add_subplot(gs[row, col*2:col*2+2])
            line, = ax.plot([], [], 'b-', linewidth=1.5)
            self.lines.append(line)
            
            sensor_name = self.sensor_names[i]
            sensor_label = self.sensor_labels[sensor_name]
            ax.set_title(f'超声波-{sensor_label}', fontweight='bold', fontsize=10, y=1.1)
            ax.set_ylabel('距离 (米)', fontsize=9)
            ax.set_ylim(0, self.max_distance)
            ax.set_xlim(0, self.time_window)
            ax.grid(True, alpha=0.3)
            ax.set_facecolor('white')
            ax.tick_params(axis='both', which='major', labelsize=8)
            
            text = ax.text(0.02, 0.95, '当前: 0.00米', transform=ax.transAxes, 
                          fontweight='bold', fontsize=8,
                          bbox=dict(boxstyle="round,pad=0.3", facecolor="lightblue", alpha=0.7))
            self.distance_texts.append(text)
            self.axes.append(ax)
        
        # 设置防跌落传感器图表
        ax_ir = self.fig.add_subplot(gs[2, 0:2])
        self.ir_line, = ax_ir.plot([], [], 'r-', linewidth=1.5)
        ax_ir.set_title('防跌落传感器 (红外)', fontweight='bold', fontsize=10)
        ax_ir.set_ylabel('距离 (米)', fontsize=8)
        ax_ir.set_ylim(0, 1.0)
        ax_ir.set_xlim(0, self.time_window)
        ax_ir.grid(True, alpha=0.3)
        ax_ir.set_facecolor('white')
        ax_ir.tick_params(axis='both', which='major', labelsize=8)
        
        self.ir_text = ax_ir.text(0.02, 0.95, '当前: 0.00米', transform=ax_ir.transAxes, 
                                 fontweight='bold', fontsize=9,
                                 bbox=dict(boxstyle="round,pad=0.3", facecolor="lightcoral", alpha=0.7))
        self.axes.append(ax_ir)
        
        # 指示灯区域
        self.indicators_ax = self.fig.add_subplot(gs[2, 2:6])
        self.indicators_ax.set_facecolor('white')
        self.indicators_ax.set_xlim(0, 12)
        self.indicators_ax.set_ylim(0, 8)
        self.indicators_ax.axis('off')
        
        # 创建指示灯（上下拉伸的椭圆形）
        self.collision_circle = Ellipse((2, 4), width=1.2, height=3.2, color='gray', ec='black', lw=2)
        self.emergency_circle = Ellipse((6, 4), width=1.2, height=3.2, color='gray', ec='black', lw=2)
        self.user_button_circle = Ellipse((10, 4), width=1.2, height=3.2, color='gray', ec='black', lw=2)
        
        self.indicators_ax.add_patch(self.collision_circle)
        self.indicators_ax.add_patch(self.emergency_circle)
        self.indicators_ax.add_patch(self.user_button_circle)
        
        # 添加标签
        self.indicators_ax.text(2, 1.0, '防碰撞\n传感器', ha='center', va='center', fontweight='bold', fontsize=9)
        self.indicators_ax.text(6, 1.0, '紧急\n停止', ha='center', va='center', fontweight='bold', fontsize=9)
        self.indicators_ax.text(10, 1.0, '用户\n按钮', ha='center', va='center', fontweight='bold', fontsize=9)
        
        self.fig.tight_layout()
        
        self.canvas = FigureCanvasTkAgg(self.fig, master=plot_frame)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        
    def setup_ros(self):
        try:
            rospy.init_node('ultrasonic_visualization', anonymous=True)
            
            rospy.Subscriber('/Distance', supersonic, self.distance_callback, queue_size=10)
            rospy.Subscriber('/ir_distance', Float32, self.ir_distance_callback, queue_size=10)
            rospy.Subscriber('/collision_sensor', Bool, self.collision_callback, queue_size=10)
            rospy.Subscriber('/user_button', Bool, self.user_button_callback, queue_size=10)
            rospy.Subscriber('/self_check_data', UInt32, self.emergency_stop_callback, queue_size=10)  # 订阅急停话题
            
            def spin_ros():
                rospy.spin()
            
            self.ros_spinner_thread = threading.Thread(target=spin_ros, daemon=True)
            self.ros_spinner_thread.start()
            
        except Exception as e:
            pass
        
    def distance_callback(self, msg):
        sensor_data = {
            'A': msg.distanceA, 'B': msg.distanceB, 'C': msg.distanceC,
            'D': msg.distanceD, 'E': msg.distanceE, 'F': msg.distanceF
        }
        self.latest_data = sensor_data
        for sensor_name, distance in sensor_data.items():
            self.distances[sensor_name].append(distance)
        self.data_count += 1
    
    def ir_distance_callback(self, msg):
        self.latest_ir_data = msg.data
        self.ir_distance.append(msg.data)
        self.ir_data_count += 1
        
    def collision_callback(self, msg):
        self.collision_state = msg.data
        self.collision_circle.set_facecolor('red' if self.collision_state else 'green')
        self.canvas.draw_idle()
        
    def user_button_callback(self, msg):
        self.user_button_state = msg.data
        self.user_button_circle.set_facecolor('blue' if self.user_button_state else 'gray')
        self.canvas.draw_idle()
        
    def emergency_stop_callback(self, msg):
        """处理急停话题回调 - 检测第21位 (1<<21 = 2097152)"""
        self.emergency_stop_state = bool(msg.data & 2097152)  # 检测第21位
        # 根据您的数据：2359296(急停按下) & 2097152 = True, 262144(松开) & 2097152 = False
        
        # 更新指示灯颜色：红色=急停激活，绿色=正常
        self.emergency_circle.set_facecolor('red' if self.emergency_stop_state else 'green')
        self.canvas.draw_idle()
        
    def update_plots_timer(self):
        if not self.running:
            return
            
        # 更新6个超声波传感器波形
        for i, (sensor_name, line) in enumerate(zip(self.sensor_names, self.lines)):
            if len(self.distances[sensor_name]) > 0:
                data_len = len(self.distances[sensor_name])
                x_data = list(range(data_len))
                y_data = list(self.distances[sensor_name])
                
                ax = self.axes[i]
                if data_len <= self.time_window:
                    ax.set_xlim(0, self.time_window)
                    line.set_data(x_data, y_data)
                else:
                    start_idx = data_len - self.time_window
                    ax.set_xlim(start_idx, data_len)
                    line.set_data(x_data[start_idx:], y_data[start_idx:])
                
                current_distance = self.distances[sensor_name][-1] if self.distances[sensor_name] else 0.0
                self.distance_texts[i].set_text(f'当前: {current_distance:.2f}米')
        
        # 更新防跌落传感器波形
        if len(self.ir_distance) > 0:
            data_len = len(self.ir_distance)
            x_data_ir = list(range(data_len))
            y_data_ir = list(self.ir_distance)
            
            ax_ir = self.axes[-1]
            if data_len <= self.time_window:
                ax_ir.set_xlim(0, self.time_window)
                self.ir_line.set_data(x_data_ir, y_data_ir)
            else:
                start_idx = data_len - self.time_window
                ax_ir.set_xlim(start_idx, data_len)
                self.ir_line.set_data(x_data_ir[start_idx:], y_data_ir[start_idx:])
            
            current_ir_distance = self.ir_distance[-1] if self.ir_distance else 0.0
            self.ir_text.set_text(f'当前: {current_ir_distance:.2f}米')
        
        self.canvas.draw_idle()
        
        if self.running:
            self.root.after(100, self.update_plots_timer)
        
    def cleanup(self):
        self.running = False
        rospy.signal_shutdown("Program terminated by user")
        self.root.quit()
        self.root.destroy()

def main():
    root = tk.Tk()
    app = UltrasonicVisualization(root)
    root.configure(bg='white')
    root.protocol("WM_DELETE_WINDOW", app.cleanup)
    
    try:
        root.mainloop()
    except KeyboardInterrupt:
        app.cleanup()

if __name__ == "__main__":
    main()