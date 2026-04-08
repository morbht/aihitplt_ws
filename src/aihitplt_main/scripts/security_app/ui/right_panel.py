#!/usr/bin/env python3

import json
import os
import rospy
import signal
import subprocess
import time
import yaml
from PyQt5.QtCore import QObject, QTimer, Qt, pyqtSignal
from PyQt5.QtGui import QFont
from PyQt5.QtWidgets import (QCheckBox, QFrame, QGridLayout, QGroupBox, QHBoxLayout,
                             QLabel, QProgressBar, QPushButton, QTextEdit, QVBoxLayout,
                             QWidget)
from std_msgs.msg import Bool, Float32, String, UInt32
from nav_msgs.msg import Odometry
from core.emergency_handler import EmergencyHandler

class ROSSignalHub(QObject):
    """跨线程信号中转站"""
    log_signal = pyqtSignal(str)
    sensor_data_signal = pyqtSignal(dict)
    pan_tilt_status_signal = pyqtSignal(str)
    pan_tilt_state_signal = pyqtSignal(bool)
    self_check_signal = pyqtSignal(int)
    # 新增：安全的UI更新信号
    battery_signal = pyqtSignal(float)
    robot_status_signal = pyqtSignal(str)

class RightPanel(QFrame):
    """右侧面板（状态和信息）- 彻底移除海康SDK依赖的纯净稳定版"""
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.parent = parent
        
        self.setFrameStyle(QFrame.Box | QFrame.Raised)
        self.setStyleSheet("background-color: white;")
        
        # 加载配置文件
        self.config = self.load_config()
        self.hub = ROSSignalHub()
        
        # 初始化传感器值存储
        self.sensor_values = {}
        self.sensor_labels = {}
        self.battery_voltage = 0
        self.has_sensor_data = False
        
        # 机器人底盘状态
        self.last_odom_time = 0  
        self.chassis_online = False  
        
        # 急停状态
        self.upper_emergency_stop = False
        self.body_emergency_stop = False
        
        # 云台状态
        self.pan_tilt_online = False  
        self.preset_position = None
        self.pan_tilt_enabled = False  
        self.pan_tilt_process = None  
        self.preset_sent = False  
        self.detect_mode = False  
        self.is_processing = False  # 操作互斥锁

        # 先初始化UI
        self.init_ui()
        
        # 初始化异常处理器
        self.emergency_handler = EmergencyHandler(self)
        
        # 连接信号
        self.setup_internal_connections()
        
        # 初始化 ROS 订阅器、发布器和定时器
        self.init_ros()
        
        # 启动时自动将云台置为休眠
        QTimer.singleShot(1000, self.send_pan_tilt_sleep)

    def setup_internal_connections(self):
        """连接所有内部信号"""
        self.hub.log_signal.connect(self._safe_log_update)
        self.hub.sensor_data_signal.connect(self.update_sensor_display)
        self.hub.pan_tilt_status_signal.connect(self.update_pan_tilt_status)
        self.hub.pan_tilt_state_signal.connect(self.on_pan_tilt_state)
        self.hub.self_check_signal.connect(self.on_self_check_data)
        
        # 连接新增的安全更新信号
        self.hub.battery_signal.connect(self.update_battery)
        self.hub.robot_status_signal.connect(self.update_robot_status)
        
        self.emergency_handler.emergency_triggered.connect(self.on_emergency_triggered)
        self.emergency_handler.emergency_cleared.connect(self.on_emergency_cleared)
        self.emergency_handler.voice_played.connect(self.on_voice_played)

    def log(self, message):
        """线程安全的日志接口"""
        self.hub.log_signal.emit(str(message))
        print(message)

    def _safe_log_update(self, message):
        from datetime import datetime
        timestamp = datetime.now().strftime("%H:%M:%S")
        if hasattr(self, 'log_text') and self.log_text:
            self.log_text.append(f"{timestamp} {message}")

    # ==================== ROS 初始化与回调 ====================

    def init_ros(self):
        """初始化 ROS 订阅与发布"""
        try:
            # 确保 ROS 节点被正确初始化，禁用信号处理防冲突
            try:
                rospy.init_node('gui_sensor_subscriber', anonymous=True, disable_signals=True)
            except rospy.exceptions.ROSException:
                pass 

            # 传感器订阅
            rospy.Subscriber('/security_sensors', String, lambda m: self.hub.sensor_data_signal.emit(json.loads(m.data.replace('data: "', '').replace('"\n---', ''))))
            # 电池电量订阅
            rospy.Subscriber('/PowerVoltage', Float32, self.battery_callback)
            # 底盘里程订阅
            rospy.Subscriber('/odom', Odometry, self.odom_callback)
            # 云台状态订阅
            rospy.Subscriber('/pan_tilt_camera/preset_control', String, lambda m: self.hub.pan_tilt_status_signal.emit(m.data))
            # 云台状态话题订阅
            rospy.Subscriber('/pan_tilt_camera/state', Bool, lambda m: self.hub.pan_tilt_state_signal.emit(m.data))
            # 自检数据订阅（用于车身急停检测）
            rospy.Subscriber('/self_check_data', UInt32, lambda m: self.hub.self_check_signal.emit(m.data))
            
            # 【关键新增】原生云台控制发布器，替代掉不稳定的 API
            self.preset_pub = rospy.Publisher('/pan_tilt_camera/preset_control', String, queue_size=10)
            
            # 环境等级发布器
            self.grade_pub = rospy.Publisher('/environment_grade', String, queue_size=10)
            
            # 创建定时器检查底盘状态
            self.chassis_timer = QTimer()
            self.chassis_timer.timeout.connect(self.check_chassis_status)
            self.chassis_timer.start(100)
            
            print("ROS节点初始化成功")
            
        except Exception as e:
            print(f"ROS 初始化失败: {e}")

    # ==================== 云台控制 (纯ROS话题驱动) ====================

    def send_pan_tilt_sleep(self):
        """发送云台休眠指令"""
        self.send_preset_point_close()
        self.pan_tilt_enabled = False
        self.update_pan_tilt_display("休眠")
        self.update_pan_tilt_button_style()
        self.log("云台已初始化为休眠状态")

    def toggle_pan_tilt(self):
        """切换云台启动/休眠状态"""
        if self.is_processing: return
        self.is_processing = True
        self.pan_tilt_btn.setEnabled(False)

        try:
            self.pan_tilt_enabled = not self.pan_tilt_enabled
            if self.pan_tilt_enabled:
                self.update_pan_tilt_display("初始化...")
                self.start_pan_tilt_process()
            else:
                self.stop_pan_tilt_process()
            
            self.update_pan_tilt_button_style()
            
        except Exception as e:
            self.log(f"云台控制失败: {e}")
            
        QTimer.singleShot(2000, lambda: self.pan_tilt_btn.setEnabled(True))
        QTimer.singleShot(2000, lambda: setattr(self, 'is_processing', False))

    def start_pan_tilt_process(self):
        """启动云台相机进程"""
        try:
            self.preset_sent = False
            self.detect_mode = self.detect_check.isChecked()
            
            if self.detect_mode:
                cmd = "roslaunch aihitplt_yolo pan_cam_detect_gui.launch"
                self.log("以检测模式启动云台相机进程")
            else:
                cmd = "roslaunch aihitplt_security aihitplt_pan_tilt_camera.launch gui:=true"
                self.log("以普通模式启动云台相机进程")
            
            self.pan_tilt_process = subprocess.Popen(
                cmd, shell=True, preexec_fn=os.setsid
            )
            self.log("云台相机进程已启动")
            
        except Exception as e:
            self.log(f"启动云台进程失败: {e}")
            self.pan_tilt_enabled = False

    def stop_pan_tilt_process(self):
        """停止云台相机进程"""
        try:
            self.send_preset_point_close()
            QTimer.singleShot(500, self.kill_pan_tilt_process)
        except Exception as e:
            self.log(f"停止云台进程失败: {e}")

    def kill_pan_tilt_process(self):
        """终止云台进程"""
        try:
            if self.pan_tilt_process:
                pgid = os.getpgid(self.pan_tilt_process.pid)
                os.killpg(pgid, signal.SIGINT)
                time.sleep(0.3)
                os.killpg(pgid, signal.SIGKILL)
                self.pan_tilt_process = None
                self.log("云台相机进程已关闭")
                self.pan_tilt_online = False
                self.update_pan_tilt_display("休眠")
        except Exception as e:
            self.log(f"终止进程失败: {e}")

    def send_preset_point_start(self):
        """发送预制点1指令 (使用原生发布器)"""
        try:
            if hasattr(self, 'preset_pub'):
                self.preset_pub.publish(String("go,1"))
                self.pan_tilt_online = True
                self.update_pan_tilt_display("运行")
                self.log("云台已移动到预置点1")
        except Exception as e:
            self.log(f"发送预置点1指令失败: {e}")

    def send_preset_point_close(self):
        """发送预制点40指令 (使用原生发布器)"""
        try:
            if hasattr(self, 'preset_pub'):
                self.preset_pub.publish(String("go,40"))
                self.log("云台已下发休眠指令")
        except Exception as e:
            self.log(f"发送休眠指令失败: {e}")

    def on_pan_tilt_state(self, state):
        """云台Ready信号接收"""
        if state and self.pan_tilt_enabled and not self.preset_sent:
            self.preset_sent = True
            QTimer.singleShot(800, self.send_preset_point_start)

    def update_pan_tilt_status(self, command):
        """更新云台状态显示"""
        try:
            if ',' in command:
                _, preset_str = command.split(',', 1)
                preset_num = int(preset_str.strip())
                
                if preset_num == 1:
                    self.pan_tilt_online = True
                    self.pan_tilt_enabled = True
                    self.update_pan_tilt_display("运行")
                    self.update_pan_tilt_button_style()
                elif preset_num == 40:
                    self.pan_tilt_online = False
                    self.pan_tilt_enabled = False
                    self.update_pan_tilt_display("休眠")
                    self.update_pan_tilt_button_style()
                    self.preset_sent = False
        except Exception as e:
            pass

    # ==================== UI 样式与布局完全还原 ====================

    def update_pan_tilt_button_style(self):
        """更新云台按钮样式（使用原始样式表）"""
        if self.pan_tilt_enabled:
            self.pan_tilt_btn.setText("关闭云台")
            self.pan_tilt_btn.setStyleSheet("""
                QPushButton {
                    background-color: #ffc107;
                    color: #333;
                    font-weight: bold;
                    border: 1px solid #e0a800;
                    border-radius: 3px;
                    padding: 2px 4px;
                    font-size: 10px;
                }
                QPushButton:hover {
                    background-color: #ffb300;
                }
            """)
        else:
            self.pan_tilt_btn.setText("启动云台")
            self.pan_tilt_btn.setStyleSheet("""
                QPushButton {
                    background-color: #f5f5f5;
                    color: #333;
                    border: 1px solid #e0e0e0;
                    border-radius: 3px;
                    padding: 2px 4px;
                    font-size: 10px;
                }
                QPushButton:hover {
                    background-color: #e3f2fd;
                    border-color: #1976d2;
                }
            """)

    def update_battery(self, percentage):
        """更新电池显示（使用原始样式表）"""
        percentage = max(0, min(100, percentage))
        self.battery_bar.setValue(percentage)
        self.battery_label.setText(f"{int(percentage)}%")
        
        if percentage <= 0:
            color = "#cccccc"
        elif percentage < 20:
            color = "#f44336"
        elif percentage < 50:
            color = "#ff9800"
        else:
            color = "#4caf50"
            
        self.battery_bar.setStyleSheet(f"""
            QProgressBar {{
                border: 1px solid #e0e0e0;
                border-radius: 4px;
                text-align: center;
                background-color: #f5f5f5;
                font-size: 10px;
                height: 16px;
            }}
            QProgressBar::chunk {{
                background-color: {color};
                border-radius: 3px;
            }}
        """)

    def init_ui(self):
        """初始化界面"""
        layout = QVBoxLayout(self)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(6)
        
        env_group = self.create_environment_group()
        layout.addWidget(env_group)
        
        sys_group = self.create_system_group()
        layout.addWidget(sys_group)
        
        log_group = self.create_log_group()
        layout.addWidget(log_group, 1)
        
        self.update_robot_status("离线")
        self.update_pan_tilt_display("休眠")
        self.update_battery(0)

    def create_environment_group(self):
        """创建环境监控组"""
        env_group = QGroupBox("环境监控")
        env_group.setStyleSheet("QGroupBox { font-size: 12px; }")
        env_layout = QVBoxLayout()
        env_layout.setSpacing(4)
        
        sensor_widget = QWidget()
        sensor_layout = QGridLayout(sensor_widget)
        sensor_layout.setVerticalSpacing(4)
        sensor_layout.setHorizontalSpacing(8)
        
        left_sensors = [
            ("alcohol", "酒精传感器:", "", ""),
            ("smoke", "烟雾传感器:", "", ""),
            ("eCO2", "CO₂浓度:", "", "ppm"),
            ("eCH2O", "甲醛浓度:", "", "mg/m³"),
            ("TVOC", "TVOC浓度:", "", "mg/m³"),
            ("PM25", "PM2.5:", "", "μg/m³")
        ]
        
        right_sensors = [
            ("PM10", "PM10:", "", "μg/m³"),
            ("temperature", "温度:", "", "°C"),
            ("humidity", "湿度:", "", "%"),
            ("light", "光照强度:", "", ""),
            ("sound", "声音强度:", "", ""),
            ("assessment", "综合评估:", "--", "")
        ]
        
        for i, (key, name, default, unit) in enumerate(left_sensors):
            row_layout = QHBoxLayout()
            row_layout.setSpacing(4)
            name_label = QLabel(name)
            name_label.setStyleSheet("color: #333333;")
            row_layout.addWidget(name_label)
            value_text = f"<b>{default}</b> {unit}" if default else unit
            value_label = QLabel(value_text)
            value_label.setStyleSheet("color: #1976d2; font-weight: bold;")
            row_layout.addWidget(value_label)
            row_layout.addStretch()
            sensor_layout.addLayout(row_layout, i, 0)
            self.sensor_labels[key] = value_label
            
        for i, (key, name, default, unit) in enumerate(right_sensors):
            row_layout = QHBoxLayout()
            row_layout.setSpacing(4)
            name_label = QLabel(name)
            name_label.setStyleSheet("color: #333333;")
            row_layout.addWidget(name_label)
            value_text = f"<b>{default}</b> {unit}" if default else unit
            value_label = QLabel(value_text)
            if key == "assessment":
                value_label.setStyleSheet("color: #999999; font-weight: bold;")
            else:
                value_label.setStyleSheet("color: #1976d2; font-weight: bold;")
            row_layout.addWidget(value_label)
            row_layout.addStretch()
            sensor_layout.addLayout(row_layout, i, 1)
            self.sensor_labels[key] = value_label
            
        env_layout.addWidget(sensor_widget)
        env_group.setLayout(env_layout)
        return env_group

    def create_system_group(self):
        """创建系统状态组"""
        sys_group = QGroupBox("系统状态")
        sys_group.setStyleSheet("""
            QGroupBox { 
                font-size: 12px; 
                padding-top: 8px;
                margin-top: 4px;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 5px 0 5px;
            }
        """)
        
        sys_group.setMinimumHeight(180)  
        
        sys_layout = QVBoxLayout()
        sys_layout.setSpacing(8)  
        sys_layout.setContentsMargins(8, 12, 8, 12)  
        
        # 第一行：急停状态
        emergency_widget = QWidget()
        emergency_widget.setFixedHeight(28)  
        emergency_layout = QHBoxLayout(emergency_widget)
        emergency_layout.setContentsMargins(0, 0, 0, 0)
        emergency_layout.setSpacing(4)
        
        emergency_label = QLabel("急停状态:")
        emergency_layout.addWidget(emergency_label)
        
        self.emergency_indicator = QLabel("●")
        self.emergency_indicator.setStyleSheet("color: #4CAF50; font-size: 14px;")  
        emergency_layout.addWidget(self.emergency_indicator)
        
        self.emergency_status = QLabel("正常")
        self.emergency_status.setStyleSheet("color: #4CAF50; font-weight: bold; font-size: 11px;") 
        emergency_layout.addWidget(self.emergency_status)
        emergency_layout.addStretch()
        sys_layout.addWidget(emergency_widget)
        
        # 第二行：云台状态、启动云台按钮、开启检测
        pan_tilt_row = QWidget()
        pan_tilt_row.setFixedHeight(28)  
        pan_tilt_layout = QHBoxLayout(pan_tilt_row)
        pan_tilt_layout.setContentsMargins(0, 0, 0, 0)
        pan_tilt_layout.setSpacing(20)
        
        # 云台状态指示
        pan_tilt_status_widget = QWidget()
        pan_tilt_status_layout = QHBoxLayout(pan_tilt_status_widget)
        pan_tilt_status_layout.setContentsMargins(0, 0, 0, 0)
        pan_tilt_status_layout.setSpacing(2)
        
        pan_tilt_label = QLabel("云台状态:")
        pan_tilt_label.setFixedWidth(60)
        pan_tilt_status_layout.addWidget(pan_tilt_label)
        
        self.pan_tilt_indicator = QLabel("●")
        self.pan_tilt_indicator.setStyleSheet("color: #f44336; font-size: 16px;") 
        pan_tilt_status_layout.addWidget(self.pan_tilt_indicator)
        
        self.pan_tilt_status_label = QLabel("休眠")
        self.pan_tilt_status_label.setStyleSheet("color: #f44336; font-weight: bold; font-size: 11px;")  
        pan_tilt_status_layout.addWidget(self.pan_tilt_status_label)
        
        pan_tilt_layout.addWidget(pan_tilt_status_widget, 35)
        
        # 启动云台按钮
        self.pan_tilt_btn = QPushButton("启动云台")
        self.pan_tilt_btn.setFixedHeight(24) 
        self.pan_tilt_btn.setFixedWidth(65) 
        self.pan_tilt_btn.setStyleSheet("""
            QPushButton {
                background-color: #f5f5f5;
                color: #333;
                border: 1px solid #e0e0e0;
                border-radius: 4px;
                padding: 3px 6px;
                font-size: 10px;
            }
            QPushButton:hover {
                background-color: #e3f2fd;
                border-color: #1976d2;
            }
        """)
        self.pan_tilt_btn.clicked.connect(self.toggle_pan_tilt)
        pan_tilt_layout.addWidget(self.pan_tilt_btn, 30)
        
        # 开启检测勾选框
        self.detect_check = QCheckBox("开启检测")
        self.detect_check.setFixedHeight(20)  
        self.detect_check.setStyleSheet("""
            QCheckBox {
                font-size: 11px;
                spacing: 4px;
            }
            QCheckBox::indicator {
                width: 14px;
                height: 14px;
            }
        """)
        pan_tilt_layout.addWidget(self.detect_check, 35)
        
        sys_layout.addWidget(pan_tilt_row)
        
        # 第三行：电池电量
        battery_widget = QWidget()
        battery_widget.setFixedHeight(32)
        battery_layout = QHBoxLayout(battery_widget)
        battery_layout.setContentsMargins(0, 0, 0, 0)
        battery_layout.setSpacing(4)
        
        battery_label = QLabel("电池电量:")
        battery_layout.addWidget(battery_label)
        
        self.battery_bar = QProgressBar()
        self.battery_bar.setRange(0, 100)
        self.battery_bar.setValue(0)  
        self.battery_bar.setTextVisible(False)  
        self.battery_bar.setStyleSheet("""
            QProgressBar {
                border: 1px solid #e0e0e0;
                border-radius: 5px;
                text-align: center;
                background-color: #f5f5f5;
                font-size: 11px;
                height: 20px;
            }
            QProgressBar::chunk {
                background-color: #4caf50;
                border-radius: 4px;
            }
        """)
        battery_layout.addWidget(self.battery_bar)
        
        self.battery_label = QLabel("0%")
        self.battery_label.setStyleSheet("font-weight: bold; font-size: 11px;")
        battery_layout.addWidget(self.battery_label)
        sys_layout.addWidget(battery_widget)
        
        # 第四行：机器人状态
        robot_widget = QWidget()
        robot_widget.setFixedHeight(28)
        robot_layout = QHBoxLayout(robot_widget)
        robot_layout.setContentsMargins(0, 0, 0, 0)
        robot_layout.setSpacing(4)
        
        robot_label = QLabel("机器人状态:")
        robot_layout.addWidget(robot_label)
        
        self.robot_indicator = QLabel("●")
        self.robot_indicator.setStyleSheet("color: #f44336; font-size: 16px;")
        robot_layout.addWidget(self.robot_indicator)
        
        self.robot_status = QLabel("离线")
        self.robot_status.setStyleSheet("color: #f44336; font-weight: bold; font-size: 11px;")
        robot_layout.addWidget(self.robot_status)
        
        robot_layout.addStretch()
        sys_layout.addWidget(robot_widget)
        
        sys_group.setLayout(sys_layout)
        return sys_group

    def create_log_group(self):
        """创建日志组"""
        log_group = QGroupBox("事件日志")
        log_group.setStyleSheet("QGroupBox { font-size: 12px; }")
        log_layout = QVBoxLayout()
        
        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        self.log_text.setFont(QFont("Consolas", 9))
        
        log_layout.addWidget(self.log_text)
        log_group.setLayout(log_layout)
        
        return log_group

    # ==================== 其他底层支撑逻辑 ====================

    def load_config(self):
        default_config = {'sensors': {},'scoring': {'weights': {},'grade_thresholds': {'A': 80, 'B': 60, 'C': 0}}}
        current_dir = os.path.dirname(os.path.abspath(__file__))
        config_path = os.path.join(os.path.dirname(current_dir), 'config', 'sensor_params', 'alarm_thresholds.yaml')
        try:
            if os.path.exists(config_path):
                with open(config_path, 'r', encoding='utf-8') as f: return yaml.safe_load(f)
        except Exception as e: pass
        return default_config

    def get_weight(self, sensor_key):
        return self.config.get('scoring', {}).get('weights', {}).get(sensor_key, 0)
        
    def get_grade_threshold(self):
        return self.config.get('scoring', {}).get('grade_thresholds', {'A': 80, 'B': 60, 'C': 0})

    def get_sensor_color(self, sensor_key, value):
        normal_color = "#1976d2"
        sensors = self.config.get('sensors', {})
        if sensor_key in sensors:
            sensor_config = sensors[sensor_key]
            is_reverse = sensor_config.get('reverse', False)
            if 'warning_min' in sensor_config and 'warning_max' in sensor_config:
                if value < sensor_config.get('alarm_min') or value > sensor_config.get('alarm_max'): return "#ff5252"
                elif value < sensor_config.get('warning_min') or value > sensor_config.get('warning_max'): return "#ff9800"
            elif 'warning' in sensor_config and 'alarm' in sensor_config:
                if is_reverse:
                    if value < sensor_config.get('alarm'): return "#ff5252"
                    elif value < sensor_config.get('warning'): return "#ff9800"
                else:
                    if value > sensor_config.get('alarm'): return "#ff5252"
                    elif value > sensor_config.get('warning'): return "#ff9800"
        return normal_color

    def update_robot_status(self, status):
        self.robot_status.setText(status)
        if status == "在线":
            self.robot_indicator.setStyleSheet("color: #4CAF50; font-size: 14px;")
            self.robot_status.setStyleSheet("color: #4CAF50; font-weight: bold;")
        else:
            self.robot_indicator.setStyleSheet("color: #f44336; font-size: 14px;")
            self.robot_status.setStyleSheet("color: #f44336; font-weight: bold;")

    def update_pan_tilt_display(self, status):
        self.pan_tilt_status_label.setText(status)
        if status == "运行":
            self.pan_tilt_indicator.setStyleSheet("color: #4CAF50; font-size: 14px;")
            self.pan_tilt_status_label.setStyleSheet("color: #4CAF50; font-weight: bold;")
        elif status == "初始化...":
            self.pan_tilt_indicator.setStyleSheet("color: #ff9800; font-size: 14px;")
            self.pan_tilt_status_label.setStyleSheet("color: #ff9800; font-weight: bold;")
        else:
            self.pan_tilt_indicator.setStyleSheet("color: #f44336; font-size: 14px;")
            self.pan_tilt_status_label.setStyleSheet("color: #f44336; font-weight: bold;")

    def update_emergency_display(self):
        emergency_triggered = self.upper_emergency_stop or self.body_emergency_stop
        if emergency_triggered:
            self.emergency_indicator.setStyleSheet("color: #f44336; font-size: 14px;")
            self.emergency_status.setText("触发")
            self.emergency_status.setStyleSheet("color: #f44336; font-weight: bold;")
        else:
            self.emergency_indicator.setStyleSheet("color: #4CAF50; font-size: 14px;")
            self.emergency_status.setText("正常")
            self.emergency_status.setStyleSheet("color: #4CAF50; font-weight: bold;")

    def battery_callback(self, msg):
        if self.chassis_online:
            self.battery_voltage = msg.data
            self.hub.battery_signal.emit(self.calculate_battery_percentage(self.battery_voltage))
        else:
            self.hub.battery_signal.emit(0)

    def calculate_battery_percentage(self, voltage):
        if voltage >= 25.4: return 100
        elif voltage <= 19.0: return 0
        else: return max(0, min(100, int(((voltage - 19.0) / (25.4 - 19.0)) * 100)))

    def odom_callback(self, msg):
        self.last_odom_time = time.time()
        if not self.chassis_online:
            self.chassis_online = True
            self.hub.robot_status_signal.emit("在线")

    def check_chassis_status(self):
        if self.last_odom_time != 0 and (time.time() - self.last_odom_time > 1.0):
            if self.chassis_online:
                self.chassis_online = False
                self.hub.robot_status_signal.emit("离线")
                self.hub.battery_signal.emit(0)

    def on_self_check_data(self, data):
        self.body_emergency_stop = (data == 2359296)
        self.update_emergency_display()

    def update_sensor_display(self, sensor_data):
        self.sensor_values = sensor_data
        self.has_sensor_data = True
        current_time = time.time()
        last_c_grade_log_time = getattr(self, '_last_c_grade_log_time', 0)
        
        mappings = {'alcohol': ('',''),'smoke': ('',''),'eCO2': ('','ppm'),'eCH2O': ('','mg/m³'),'TVOC': ('','mg/m³'),'PM25': ('','μg/m³'),'PM10': ('','μg/m³'),'temperature': ('','°C'),'humidity': ('','%'),'light': ('',''),'sound': ('','')}
        for key, (_, unit) in mappings.items():
            if key in sensor_data:
                value = sensor_data[key]
                if key in ['alcohol', 'smoke', 'light', 'sound']: text = f"<b>{int(value)}</b> {unit}"
                elif key in ['temperature', 'humidity']: text = f"<b>{value:.1f}</b> {unit}"
                elif key in ['eCH2O', 'TVOC']: text = f"<b>{value:.2f}</b> {unit}"
                else: text = f"<b>{value}</b> {unit}"
                
                if key in self.sensor_labels:
                    color = self.get_sensor_color(key, value)
                    self.sensor_labels[key].setStyleSheet(f"color: {color}; font-weight: bold;")
                    self.sensor_labels[key].setText(text)
        
        if 'emergency_stop' in sensor_data:
            self.upper_emergency_stop = (sensor_data['emergency_stop'] == 0)
            self.update_emergency_display()
        
        assessment = self.calculate_assessment(sensor_data)
        if 'assessment' in self.sensor_labels:
            grade_thresholds = self.get_grade_threshold()
            if assessment >= grade_thresholds.get('A', 80): grade = "A"; color = "#4CAF50"
            elif assessment >= grade_thresholds.get('B', 60): grade = "B"; color = "#ff9800"
            else:
                grade = "C"; color = "#f44336"
                if current_time - last_c_grade_log_time > 1:
                    if hasattr(self.parent, 'left_panel') and hasattr(self.parent.left_panel, 'inspection_table'):
                        if self.parent.left_panel.inspection_table.is_inspecting:
                            self.log("【警告】环境指数降至C级")
                            self._last_c_grade_log_time = current_time
            try:
                if hasattr(self, 'grade_pub'): self.grade_pub.publish(String(f"{grade},{int(assessment)}"))
            except Exception as e: pass
            
            self.sensor_labels['assessment'].setText(f"<b>{grade}</b>")
            self.sensor_labels['assessment'].setStyleSheet(f"color: {color}; font-weight: bold;")

    def calculate_assessment(self, sensor_data):
        score = 100
        for sensor_key, value in sensor_data.items():
            if sensor_key == 'emergency_stop': continue
            weight = self.get_weight(sensor_key)
            if weight == 0: continue
            sensors = self.config.get('sensors', {})
            if sensor_key not in sensors: continue
            sensor_config = sensors[sensor_key]
            is_reverse = sensor_config.get('reverse', False)
            if 'warning_min' in sensor_config and 'warning_max' in sensor_config:
                if value < sensor_config.get('alarm_min') or value > sensor_config.get('alarm_max'): score -= weight
                elif value < sensor_config.get('warning_min') or value > sensor_config.get('warning_max'): score -= weight // 2
            elif 'warning' in sensor_config and 'alarm' in sensor_config:
                if is_reverse:
                    if value < sensor_config.get('alarm'): score -= weight
                    elif value < sensor_config.get('warning'): score -= weight // 2
                else:
                    if value > sensor_config.get('alarm'): score -= weight
                    elif value > sensor_config.get('warning'): score -= weight // 2
        return max(0, min(100, score))

    def on_emergency_triggered(self):
        self.log("【紧急】环境异常！进入应急模式")
        if hasattr(self.parent, 'left_panel') and hasattr(self.parent.left_panel, 'inspection_table'):
            inspection_table = self.parent.left_panel.inspection_table
            if inspection_table.is_inspecting and not inspection_table.is_paused:
                inspection_table.pause_inspection()
                self.log("巡检任务已暂停")
        self.emergency_indicator.setStyleSheet("color: #f44336; font-size: 14px; font-weight: bold;")
        self.emergency_indicator.setText("⚠")
        
    def on_emergency_cleared(self):
        self.log("环境恢复正常，退出应急模式")
        self.emergency_indicator.setStyleSheet("color: #4CAF50; font-size: 14px;")
        self.emergency_indicator.setText("●")
        if hasattr(self.parent, 'left_panel') and hasattr(self.parent.left_panel, 'inspection_table'):
            inspection_table = self.parent.left_panel.inspection_table
            if inspection_table.is_paused:
                QTimer.singleShot(1000, inspection_table.resume_inspection)
                self.log("准备恢复巡检任务")
        
    def on_voice_played(self, success):
        if success: self.log("异常语音播报完成")
        else: self.log("异常语音播报失败，请检查语音文件")