#!/usr/bin/env python3
import sys
import subprocess
import os
import time
import signal
from PyQt5.QtWidgets import *
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QFont

class SidebarApp(QMainWindow):
    pack_path = "aihitplt_dev"
    
    def __init__(self):
        super().__init__()
        
        # 状态变量 - 统一管理
        self.processes_state = {
            'agv_hardware': {'running': False, 'process': None, 'mag_terminal': None},
            'slam': {'running': False, 'process': None},
            'rviz': {'running': False, 'process': None},
            'voice': {'running': False, 'process': None, 'terminal_process': None},
            'keyboard': {'running': False, 'process': None, 'terminal_process': None},
        }
        
        # 上装模块进程状态
        self.upper_processes = {}
        
        # 建图导航模块进程状态
        self.mapping_nav_processes = {}
        
        # 激光雷达模块进程状态
        self.lidar_processes = {}
        
        # 视觉应用模块进程状态
        self.vision_processes = {}
        
        # 模块选择状态 - 多选
        self.modules_selected = {
            'depth_camera': False,
            'knob_screen': False,
            'lidar': False,
            'mag_nav': False,  
            'ar_marker': False
        }
        
        # SLAM建图方法选择状态 - 单选
        self.slam_selected = {
            'Gmapping': False,
            'Hector': False,
            'Karto': False,
            'Cartographer': False,
            'Web': False
        }
        
        self.setup_ui()
        
        # 自动刷新ROS状态
        self.ros_timer = QTimer()
        self.ros_timer.timeout.connect(self.check_ros_status)
        self.ros_timer.start(2000)  # 每2秒刷新一次
        
        # 立即检查一次
        QTimer.singleShot(500, self.check_ros_status)
    
    def setup_ui(self):
        self.setWindowTitle("AIHIT 机器人控制平台")
        self.setFixedSize(300, 450)
        
        # 创建主窗口部件
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        
        # 主布局
        main_layout = QHBoxLayout()
        main_layout.setContentsMargins(0, 0, 0, 0)
        main_layout.setSpacing(0)
        
        # 左侧边栏
        self.sidebar = self.create_sidebar()
        main_layout.addWidget(self.sidebar)
        
        # 右侧内容区域
        self.content_widget = QWidget()
        self.content_layout = QVBoxLayout(self.content_widget)
        self.content_layout.setContentsMargins(0, 0, 0, 0)
        self.content_layout.setSpacing(0)
        self.create_content_pages()
        
        main_layout.addWidget(self.content_widget, 1)
        main_widget.setLayout(main_layout)
    
    def create_sidebar(self):
        """创建左侧边栏"""
        sidebar = QWidget()
        sidebar.setFixedWidth(80)
        
        sidebar.setStyleSheet("""
            QWidget {
                background-color: #2c3e50;
            }
            QPushButton {
                background-color: transparent;
                color: white;
                border: none;
                text-align: center;
                padding: 5px;
                font-size: 10px;
            }
            QPushButton:hover {
                background-color: #34495e;
            }
            QPushButton:checked {
                background-color: #3498db;
                border-left: 2px solid #2980b9;
            }
        """)
        
        layout = QVBoxLayout()
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)
        layout.setAlignment(Qt.AlignTop)
        
        # 标题
        title = QLabel("AIHIT\n控制台")
        title.setStyleSheet("""
            QLabel {
                color: white; 
                font-size: 11px; 
                font-weight: bold; 
                padding: 10px 22px;
                text-align: center;
            }
        """)
        title.setWordWrap(True)
        layout.addWidget(title)
        
        # 导航按钮
        nav_items = [
            ("🖥️\n开发主界面", 0),
            ("⚙️\nAGV上装", 1),
            ("🗺️\n建图导航", 2),
            ("📡\n激光雷达", 3),
            ("👁️\n视觉应用", 4),
        ]
        
        self.button_group = QButtonGroup(self)
        for text, page_id in nav_items:
            btn = QPushButton(text)
            btn.setCheckable(True)
            btn.setFixedHeight(45)
            btn.clicked.connect(lambda checked, pid=page_id: self.switch_page(pid))
            self.button_group.addButton(btn, page_id)
            layout.addWidget(btn)
        
        layout.addStretch()
        
        # 底部状态
        self.sidebar_status_label = QLabel("就绪")
        self.sidebar_status_label.setStyleSheet("""
            QLabel {
                color: #95a5a6; 
                font-size: 10px; 
                padding: 3px;
            }
        """)
        self.sidebar_status_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.sidebar_status_label)
        
        sidebar.setLayout(layout)
        return sidebar
    
    def create_content_pages(self):
        """创建各个内容页面"""
        # 开发主界面
        dashboard = self.create_dashboard()
        self.content_layout.addWidget(dashboard)
        
        # AGV上装页面
        agv_upper_page = self.create_agv_upper_page()
        self.content_layout.addWidget(agv_upper_page)
        
        # 建图导航页面
        mapping_nav_page = self.create_mapping_nav_page()
        self.content_layout.addWidget(mapping_nav_page)
        
        # 激光雷达页面
        lidar_page = self.create_lidar_page()
        self.content_layout.addWidget(lidar_page)
        
        # 视觉应用页面
        vision_page = self.create_vision_page()
        self.content_layout.addWidget(vision_page)
        

        
        # 默认显示开发主界面，隐藏其他页面
        for i in range(1, 5):
            self.content_layout.itemAt(i).widget().hide()
    
    def create_dashboard(self):
        """创建开发主界面"""
        widget = QWidget()
        layout = QVBoxLayout()
        layout.setContentsMargins(5, 5, 5, 5)
        layout.setSpacing(4)
        
        # 欢迎信息
        welcome = QLabel("AIHIT机器人平台")
        welcome.setStyleSheet("""
            QLabel {
                font-size: 13px; 
                font-weight: bold; 
                margin: 2px 0;
            }
        """)
        welcome.setAlignment(Qt.AlignCenter)
        layout.addWidget(welcome)
        
        # 快速启动区域
        quick_start = QGroupBox("快速启动")
        quick_start.setStyleSheet("""
            QGroupBox {
                font-weight: bold;
                font-size: 10px;
                border: 1px solid #3498db;
                border-radius: 2px;
                margin-top: 3px;
                padding-top: 5px;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 3px;
                padding: 0 2px 0 2px;
            }
        """)
        quick_layout = QVBoxLayout()
        quick_layout.setSpacing(3)
        
        # Main功能包VScode
        vscode_btn = QPushButton("Main功能包VScode")
        vscode_btn.setMinimumHeight(28)
        vscode_btn.setStyleSheet("""
            QPushButton {
                background-color: #3498db;
                color: white;
                border-radius: 2px;
                padding: 4px;
                font-size: 11px;
            }
            QPushButton:hover {
                background-color: #2980b9;
            }
        """)
        vscode_btn.clicked.connect(lambda: self.execute_command("code ~/aihitplt_ws/src/aihitplt_main/scripts"))
        quick_layout.addWidget(vscode_btn)
        
        # AGV硬件按钮
        self.agv_btn = QPushButton("启动AGV硬件")
        self.agv_btn.setMinimumHeight(28)
        self.agv_btn.clicked.connect(self.toggle_agv_hardware)
        self.update_button_style('agv_hardware')
        quick_layout.addWidget(self.agv_btn)
        
        # 模块选择行1：深度相机、旋钮屏幕、激光雷达
        self.module_row1 = QWidget()
        module_layout1 = QHBoxLayout()
        module_layout1.setContentsMargins(0, 0, 0, 0)
        module_layout1.setSpacing(2)
        
        # 深度相机按钮
        self.depth_camera_btn = QPushButton("深度相机")
        self.depth_camera_btn.setCheckable(True)
        self.depth_camera_btn.setMinimumHeight(25)
        self.depth_camera_btn.clicked.connect(
            lambda checked: self.toggle_module('depth_camera', self.depth_camera_btn, checked)
        )
        self.update_module_button_style(self.depth_camera_btn, False)
        module_layout1.addWidget(self.depth_camera_btn)
        
        # 旋钮屏幕按钮
        self.knob_screen_btn = QPushButton("旋钮屏幕")
        self.knob_screen_btn.setCheckable(True)
        self.knob_screen_btn.setMinimumHeight(25)
        self.knob_screen_btn.clicked.connect(
            lambda checked: self.toggle_module('knob_screen', self.knob_screen_btn, checked)
        )
        self.update_module_button_style(self.knob_screen_btn, False)
        module_layout1.addWidget(self.knob_screen_btn)
        
        # 激光雷达按钮
        self.lidar_btn = QPushButton("激光雷达")
        self.lidar_btn.setCheckable(True)
        self.lidar_btn.setMinimumHeight(25)
        self.lidar_btn.clicked.connect(
            lambda checked: self.toggle_module('lidar', self.lidar_btn, checked)
        )
        self.update_module_button_style(self.lidar_btn, False)
        module_layout1.addWidget(self.lidar_btn)
        
        self.module_row1.setLayout(module_layout1)
        quick_layout.addWidget(self.module_row1)
        
        # 模块选择行2：磁导航、AR码识别
        self.module_row2 = QWidget()
        module_layout2 = QHBoxLayout()
        module_layout2.setContentsMargins(0, 0, 0, 0)
        module_layout2.setSpacing(2)
        
        # 磁导航按钮 
        self.mag_nav_btn = QPushButton("磁导航")
        self.mag_nav_btn.setCheckable(True)
        self.mag_nav_btn.setMinimumHeight(25)
        self.mag_nav_btn.clicked.connect(
            lambda checked: self.toggle_module('mag_nav', self.mag_nav_btn, checked)
        )
        self.update_module_button_style(self.mag_nav_btn, False)
        module_layout2.addWidget(self.mag_nav_btn)
        
        # AR码识别按钮
        self.ar_marker_btn = QPushButton("AR码识别")
        self.ar_marker_btn.setCheckable(True)
        self.ar_marker_btn.setMinimumHeight(25)
        self.ar_marker_btn.clicked.connect(
            lambda checked: self.toggle_module('ar_marker', self.ar_marker_btn, checked)
        )
        self.update_module_button_style(self.ar_marker_btn, False)
        module_layout2.addWidget(self.ar_marker_btn)
        
        self.module_row2.setLayout(module_layout2)
        quick_layout.addWidget(self.module_row2)
        
        # 可视化界面按钮
        self.rviz_btn = QPushButton("显示可视化界面")
        self.rviz_btn.setMinimumHeight(28)
        self.rviz_btn.clicked.connect(self.toggle_rviz)
        self.update_button_style('rviz')
        quick_layout.addWidget(self.rviz_btn)
        
        # 离线语音交互按钮
        self.voice_btn = QPushButton("离线语音交互")
        self.voice_btn.setMinimumHeight(28)
        self.voice_btn.clicked.connect(self.toggle_voice)
        self.update_button_style('voice')
        quick_layout.addWidget(self.voice_btn)
        
        # SLAM导航按钮（可切换的）
        self.slam_btn = QPushButton("启动SLAM建图")
        self.slam_btn.setMinimumHeight(28)
        self.slam_btn.clicked.connect(self.toggle_slam)
        self.update_button_style('slam')
        quick_layout.addWidget(self.slam_btn)
        
        # 第一行SLAM建图方法选择按钮
        self.maptype_row1 = QWidget()
        maptype_layout1 = QHBoxLayout()
        maptype_layout1.setContentsMargins(0, 0, 0, 0)
        maptype_layout1.setSpacing(2)
        
        # Gmapping按钮
        self.Gmapping_btn = QPushButton("Gmapping")
        self.Gmapping_btn.setCheckable(True)
        self.Gmapping_btn.setMinimumHeight(25)
        self.Gmapping_btn.clicked.connect(
            lambda checked: self.toggle_slam_module('Gmapping', self.Gmapping_btn, checked)
        )
        self.update_maptype_button_style(self.Gmapping_btn, False)
        maptype_layout1.addWidget(self.Gmapping_btn)
        
        # Hector按钮
        self.Hector_btn = QPushButton("Hector")
        self.Hector_btn.setCheckable(True)
        self.Hector_btn.setMinimumHeight(25)
        self.Hector_btn.clicked.connect(
            lambda checked: self.toggle_slam_module('Hector', self.Hector_btn, checked)
        )
        self.update_maptype_button_style(self.Hector_btn, False)
        maptype_layout1.addWidget(self.Hector_btn)
        
        # Karto按钮
        self.Karto_btn = QPushButton("Karto")
        self.Karto_btn.setCheckable(True)
        self.Karto_btn.setMinimumHeight(25)
        self.Karto_btn.clicked.connect(
            lambda checked: self.toggle_slam_module('Karto', self.Karto_btn, checked)
        )
        self.update_maptype_button_style(self.Karto_btn, False)
        maptype_layout1.addWidget(self.Karto_btn)
        
        self.maptype_row1.setLayout(maptype_layout1)
        quick_layout.addWidget(self.maptype_row1)
        
        # 第二行SLAM建图方法选择按钮
        self.maptype_row2 = QWidget()
        maptype_layout2 = QHBoxLayout()
        maptype_layout2.setContentsMargins(0, 0, 0, 0)
        maptype_layout2.setSpacing(2)
        
        # Cartographer按钮
        self.Cartographer_btn = QPushButton("Cartographer")
        self.Cartographer_btn.setCheckable(True)
        self.Cartographer_btn.setMinimumHeight(25)
        self.Cartographer_btn.clicked.connect(
            lambda checked: self.toggle_slam_module('Cartographer', self.Cartographer_btn, checked)
        )
        self.update_maptype_button_style(self.Cartographer_btn, False)
        maptype_layout2.addWidget(self.Cartographer_btn)
        
        # Web按钮
        self.Web_btn = QPushButton("Web")
        self.Web_btn.setCheckable(True)
        self.Web_btn.setMinimumHeight(25)
        self.Web_btn.clicked.connect(
            lambda checked: self.toggle_slam_module('Web', self.Web_btn, checked)
        )
        self.update_maptype_button_style(self.Web_btn, False)
        maptype_layout2.addWidget(self.Web_btn)
        
        self.maptype_row2.setLayout(maptype_layout2)
        quick_layout.addWidget(self.maptype_row2)
        
        # 键盘控制按钮
        self.keyboard_btn = QPushButton("启动键盘控制")
        self.keyboard_btn.setMinimumHeight(25)
        self.keyboard_btn.clicked.connect(self.toggle_keyboard)
        self.update_button_style('keyboard')
        quick_layout.addWidget(self.keyboard_btn)
        
        quick_start.setLayout(quick_layout)
        layout.addWidget(quick_start)
        
        status_group = QGroupBox("系统状态")
        status_group.setStyleSheet("""
            QGroupBox {
                font-weight: bold;
                font-size: 10px;
                border: 1px solid #7f8c8d;
                border-radius: 2px;
                margin-top: 3px;
                padding-top: 5px;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 3px;
                padding: 0 2px 0 2px;
            }
        """)
        status_layout = QVBoxLayout()
        status_layout.setSpacing(3)
        
        # ROS Master状态（简化布局）
        self.ros_status_label = QLabel("检测中...")
        self.ros_status_label.setStyleSheet("""
            QLabel {
                font-weight: bold; 
                font-size: 10px; 
                margin: 2px 0;
            }
        """)
        self.ros_status_label.setAlignment(Qt.AlignCenter)
        status_layout.addWidget(self.ros_status_label)
        
        status_group.setLayout(status_layout)
        layout.addWidget(status_group)
        
        widget.setLayout(layout)
        return widget
    
    def create_mapping_nav_page(self):
        """创建建图导航页面"""
        widget = QWidget()
        layout = QVBoxLayout()
        layout.setContentsMargins(5, 5, 5, 5)
        layout.setSpacing(4)
        
        # 标题
        title = QLabel("建图导航算法控制")
        title.setStyleSheet("""
            QLabel {
                font-size: 13px; 
                font-weight: bold; 
                margin: 2px 0;
            }
        """)
        title.setAlignment(Qt.AlignCenter)
        layout.addWidget(title)
        
        # 创建滚动区域
        scroll_area = QScrollArea()
        scroll_area.setWidgetResizable(True)
        scroll_area.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        scroll_area.setVerticalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        
        # 滚动内容部件
        scroll_content = QWidget()
        scroll_layout = QVBoxLayout(scroll_content)
        scroll_layout.setContentsMargins(5, 5, 5, 5)
        scroll_layout.setSpacing(8)
        
        # Gmapping算法
        gmapping_group = self.create_mapping_group(
            "Gmapping算法",
            [
                ("Gmapping算法建图", f"roslaunch {self.pack_path} qt_slam_mapping.launch map_type:=gmapping", False),
                ("Gmapping算法导航", f"roslaunch {self.pack_path} qt_navigation.launch map_name:=gmapping_map", False),
                ("Gmapping算法保存地图", "save_gmapping_map", True)  # 保存地图按钮
            ]
        )
        scroll_layout.addWidget(gmapping_group)
        
        # Hector算法
        hector_group = self.create_mapping_group(
            "Hector算法",
            [
                ("Hector算法建图", f"roslaunch {self.pack_path} qt_slam_mapping.launch map_type:=hector", False),
                ("Hector算法导航", f"roslaunch {self.pack_path} qt_navigation.launch map_name:=hector_map", False),
                ("Hector算法保存地图", "save_hector_map", True)  # 保存地图按钮
            ]
        )
        scroll_layout.addWidget(hector_group)
        
        # Karto算法
        karto_group = self.create_mapping_group(
            "Karto算法",
            [
                ("Karto算法建图", f"roslaunch {self.pack_path} qt_slam_mapping.launch map_type:=karto", False),
                ("Karto算法导航", f"roslaunch {self.pack_path} qt_navigation.launch map_name:=karto_map", False),
                ("Karto算法保存地图", "save_karto_map", True)  # 保存地图按钮
            ]
        )
        scroll_layout.addWidget(karto_group)
        
        # Cartographer算法
        cartographer_group = self.create_mapping_group(
            "Cartographer算法",
            [
                ("Cartographer算法建图", f"roslaunch {self.pack_path} qt_slam_mapping.launch map_type:=cartographer", False),
                ("Cartographer算法导航", f"roslaunch {self.pack_path} qt_navigation.launch map_name:=carto_map", False),  # 注意这里改为carto_map
                ("Cartographer算法保存地图", "save_carto_map", True)  # 保存地图按钮，注意这里改为carto_map
            ]
        )
        scroll_layout.addWidget(cartographer_group)
        
        # 视觉算法
        vision_group = self.create_mapping_group(
            "视觉算法",
            [
                ("视觉2D建图", f"roslaunch {self.pack_path} qt_vision_mapping.launch", False),
                ("视觉2D导航", f"roslaunch {self.pack_path} qt_vision_nav.launch map_name:=2d_vision_map", False),  # 使用对应的地图名称
                ("视觉2D保存地图", "save_vision_2d_map", True)  # 保存地图按钮
            ]
        )
        scroll_layout.addWidget(vision_group)
        
        # RTAB-Map算法
        rtabmap_group = self.create_mapping_group(
            "RTAB-Map算法",
            [
                ("RTAB-Map算法建图", f"roslaunch {self.pack_path} qt_rtabmap.launch", False),
                ("RTAB-Map算法导航", f"roslaunch {self.pack_path} qt_rtabmap_nav.launch", False)
            ]
        )
        scroll_layout.addWidget(rtabmap_group)
        
        # 添加伸缩空间
        scroll_layout.addStretch()
        
        # 设置滚动区域内容
        scroll_area.setWidget(scroll_content)
        layout.addWidget(scroll_area)
        
        widget.setLayout(layout)
        return widget
    
    def create_mapping_group(self, title, buttons_info):
        """创建建图导航模块组"""
        group = QGroupBox(title)
        group.setStyleSheet("""
            QGroupBox {
                font-weight: bold;
                font-size: 10px;
                border: 1px solid #3498db;
                border-radius: 2px;
                margin-top: 3px;
                padding-top: 8px;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 3px;
                padding: 0 2px 0 2px;
            }
        """)
        
        layout = QVBoxLayout()
        layout.setSpacing(3)
        
        for btn_info in buttons_info:
            # 按钮信息包含3个元素：文本、命令、是否保存地图按钮
            btn_text, cmd, is_save_map = btn_info
            
            # 创建按钮名称，用于状态管理
            btn_key = f"{title}_{btn_text}".replace(" ", "_").replace("-", "_").replace("!", "").lower()
            
            # 初始化状态（仅对非保存地图按钮进行状态管理）
            if not is_save_map and btn_key not in self.mapping_nav_processes:
                self.mapping_nav_processes[btn_key] = {
                    'running': False,
                    'process': None,
                    'terminal_process': None
                }
            
            btn = QPushButton(btn_text)
            btn.setMinimumHeight(28)
            btn.setProperty('command', cmd)
            btn.setProperty('btn_key', btn_key)
            btn.setProperty('is_save_map', is_save_map)  # 标记是否为保存地图按钮
            
            # 设置初始样式
            if is_save_map:
                # 保存地图按钮特殊样式（始终为蓝色，不切换）
                btn.setStyleSheet("""
                    QPushButton {
                        background-color: #3498db;
                        color: white;
                        border-radius: 2px;
                        padding: 4px;
                        font-size: 11px;
                    }
                    QPushButton:hover {
                        background-color: #2980b9;
                    }
                """)
            else:
                # 普通按钮样式
                state = self.mapping_nav_processes.get(btn_key, {'running': False})
                if state['running']:
                    btn.setText(f"关闭{btn_text}")
                    btn.setStyleSheet("""
                        QPushButton {
                            background-color: #f1c40f;
                            color: black;
                            border-radius: 2px;
                            padding: 4px;
                            font-size: 11px;
                        }
                        QPushButton:hover {
                            background-color: #f39c12;
                        }
                    """)
                else:
                    btn.setText(btn_text)
                    btn.setStyleSheet("""
                        QPushButton {
                            background-color: #3498db;
                            color: white;
                            border-radius: 2px;
                            padding: 4px;
                            font-size: 11px;
                        }
                        QPushButton:hover {
                            background-color: #2980b9;
                        }
                    """)
            
            # 连接点击事件
            btn.clicked.connect(lambda checked, b=btn: self.toggle_mapping_module(b))
            layout.addWidget(btn)
        
        group.setLayout(layout)
        return group
    
    def toggle_mapping_module(self, button):
        """切换建图导航模块"""
        cmd = button.property('command')
        btn_key = button.property('btn_key')
        is_save_map = button.property('is_save_map')
        
        if is_save_map:
            # 处理保存地图按钮
            self.handle_save_map(button, cmd)
            return
        
        # 普通按钮处理
        state = self.mapping_nav_processes[btn_key]
        
        if not state['running']:
            # 启动模块
            print(f"启动建图导航模块: {cmd}")
            
            try:
                process = subprocess.Popen(
                    cmd,
                    shell=True,
                    preexec_fn=os.setsid
                )
                state['process'] = process
                state['running'] = True
                
                # 更新按钮文本和样式
                btn_text = button.text()
                button.setText(f"关闭{btn_text}")
                button.setStyleSheet("""
                    QPushButton {
                        background-color: #f1c40f;
                        color: black;
                        border-radius: 2px;
                        padding: 4px;
                        font-size: 11px;
                    }
                    QPushButton:hover {
                        background-color: #f39c12;
                    }
                """)
                
            except Exception as e:
                QMessageBox.critical(self, "错误", f"启动模块失败: {str(e)}")
                state['running'] = False
        
        else:
            # 关闭模块
            print(f"关闭建图导航模块: {cmd}")
            
            # 1. 首先关闭键盘控制（因为launch文件中自动启动了键盘控制）
            print("正在关闭建图功能，同时关闭关联的键盘控制...")
            self._close_mapping_keyboard(cmd)
            
            # 2. 关闭主进程组
            if state['process'] and state['process'].pid:
                try:
                    os.killpg(os.getpgid(state['process'].pid), signal.SIGTERM)
                    print(f"已关闭建图导航模块进程组 (PID: {state['process'].pid})")
                except Exception as e:
                    print(f"关闭建图导航模块进程组失败: {e}")
            
            # 3. 清理相关进程
            try:
                launch_file = cmd.split(" ")[-1]
                # 清理主建图进程
                subprocess.run(f"pkill -f '{launch_file}'", 
                            shell=True, timeout=1,
                            stdout=subprocess.DEVNULL,
                            stderr=subprocess.DEVNULL)
                
                # 清理可能的建图相关进程
                mapping_patterns = [
                    "slam_mapping.launch",  # 你的launch文件
                    "aihitplt_map.launch",  # 引用的导航launch
                    "qt_slam_mapping.launch",  # 你代码中提到的launch
                    "gmapping",
                    "hector_mapping",
                    "karto",
                    "cartographer",
                    "amcl",
                    "move_base",
                    "map_server",
                    "rviz",
                    "joint_state_publisher",
                    "robot_state_publisher"
                ]
                
                for pattern in mapping_patterns:
                    try:
                        subprocess.run(f"pkill -f '{pattern}'", 
                                    shell=True, timeout=0.5,
                                    stdout=subprocess.DEVNULL,
                                    stderr=subprocess.DEVNULL)
                    except:
                        pass
                    
            except:
                pass
            
            # 4. 重置状态
            state['process'] = None
            state['running'] = False
            
            # 5. 更新按钮文本和样式
            btn_text = button.text().replace("关闭", "")
            button.setText(btn_text)
            button.setStyleSheet("""
                QPushButton {
                    background-color: #3498db;
                    color: white;
                    border-radius: 2px;
                    padding: 4px;
                    font-size: 11px;
                }
                QPushButton:hover {
                    background-color: #2980b9;
                }
            """)

    def _close_mapping_keyboard(self, mapping_cmd):
        """专门关闭建图功能关联的键盘控制"""
        # 提取launch文件名，用于精确识别
        launch_file = mapping_cmd.split(" ")[-1]
        
        print(f"清理与 {launch_file} 关联的键盘控制进程...")
        
        # 1. 查找并关闭通过launch文件启动的键盘控制终端
        try:
            # 查找包含 "gnome-terminal.*qt_keyboard_teleop" 的进程
            result = subprocess.run(
                "ps aux | grep 'gnome-terminal.*qt_keyboard_teleop' | grep -v grep",
                shell=True,
                capture_output=True,
                text=True
            )
            
            if result.stdout:
                print(f"找到键盘控制终端进程: {result.stdout}")
                # 关闭这些终端进程
                subprocess.run("pkill -f 'gnome-terminal.*qt_keyboard_teleop'", 
                            shell=True, timeout=1,
                            stdout=subprocess.DEVNULL,
                            stderr=subprocess.DEVNULL)
        except:
            pass
        
        # 2. 关闭键盘控制相关进程
        keyboard_patterns = [
            "keyboard_teleop_launcher",  # 你的launch文件中的节点名
            "keyboard_teleop",
            "teleop_twist_keyboard",
            "qt_keyboard_teleop.launch",
            "roslaunch.*keyboard_teleop"
        ]
        
        for pattern in keyboard_patterns:
            try:
                subprocess.run(f"pkill -f '{pattern}'", 
                            shell=True, timeout=0.5,
                            stdout=subprocess.DEVNULL,
                            stderr=subprocess.DEVNULL)
            except:
                pass
        
        # 3. 清理主界面可能启动的键盘控制
        main_keyboard_state = self.processes_state['keyboard']
        if main_keyboard_state['running']:
            print("发现主界面的键盘控制正在运行，一并关闭...")
            # 关闭主界面的键盘控制
            if main_keyboard_state['terminal_process']:
                try:
                    subprocess.run("pkill -f 'gnome-terminal.*qt_keyboard_teleop'", 
                                shell=True, timeout=1,
                                stdout=subprocess.DEVNULL,
                                stderr=subprocess.DEVNULL)
                except:
                    pass
            
            # 重置主界面键盘控制状态
            main_keyboard_state['terminal_process'] = None
            main_keyboard_state['process'] = None
            main_keyboard_state['running'] = False
            
            # 更新主界面按钮样式
            self.update_button_style('keyboard')
    
    def handle_save_map(self, button, cmd):
        """处理保存地图按钮"""
        btn_text = button.text()
        
        try:
            # 定义算法名称到地图名称的映射
            algorithm_map_names = {
                "save_gmapping_map": "gmapping_map",
                "save_hector_map": "hector_map",
                "save_karto_map": "karto_map",
                "save_carto_map": "carto_map",  # 注意这里是carto_map不是cartographer_map
                "save_vision_2d_map": "2d_vision_map",
            }
            
            # 根据cmd获取对应的地图名称
            if cmd in algorithm_map_names:
                map_name = algorithm_map_names[cmd]
                save_cmd = f"rosrun map_server map_saver -f ~/aihitplt_ws/src/aihitplt_nav/maps/{map_name}"
                print(f"保存{algorithm_map_names[cmd].replace('_map', '').replace('_', ' ')}地图: {save_cmd}")
            
            print(f"执行保存地图命令: {save_cmd}")
            
            result = subprocess.run(
                save_cmd,
                shell=True,
                capture_output=True,
                text=True,
                timeout=2  # 增加超时时间
            )
            
            if result.returncode == 0:
                QMessageBox.information(self, "保存成功", f"{btn_text}成功！\n地图已保存为: {map_name if cmd in algorithm_map_names else 'aihitplt'}\n保存路径: ~/aihitplt_ws/src/aihitplt_nav/maps/")
                print(f"{btn_text}成功")
            else:
                QMessageBox.warning(self, "保存失败", f"{btn_text}失败！\n错误信息: {result.stderr}")
                print(f"{btn_text}失败: {result.stderr}")
                
        except subprocess.TimeoutExpired:
            QMessageBox.warning(self, "保存超时", f"{btn_text}超时，请检查ROS状态和地图服务")
            print(f"{btn_text}超时")
        except Exception as e:
            QMessageBox.critical(self, "错误", f"{btn_text}发生错误: {str(e)}")
            print(f"{btn_text}错误: {e}")
    
    def create_agv_upper_page(self):
        """创建AGV上装页面"""
        widget = QWidget()
        layout = QVBoxLayout()
        layout.setContentsMargins(5, 5, 5, 5)
        layout.setSpacing(4)
        
        # 标题
        title = QLabel("AGV上装模块控制")
        title.setStyleSheet("""
            QLabel {
                font-size: 13px; 
                font-weight: bold; 
                margin: 2px 0;
            }
        """)
        title.setAlignment(Qt.AlignCenter)
        layout.addWidget(title)
        
        # 创建滚动区域
        scroll_area = QScrollArea()
        scroll_area.setWidgetResizable(True)
        scroll_area.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        scroll_area.setVerticalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        
        # 滚动内容部件
        scroll_content = QWidget()
        scroll_layout = QVBoxLayout(scroll_content)
        scroll_layout.setContentsMargins(5, 5, 5, 5)
        scroll_layout.setSpacing(8)
        
        # 迎宾模块
        welcome_group = self.create_upper_module_group(
            "迎宾模块",
            [
                ("急停状态", f"roslaunch {self.pack_path} qt_guide_estop.launch", True),
                ("USB摄像头", f"roslaunch {self.pack_path} qt_usbcam_greet.launch", False),
                ("迎宾导览界面", f"roslaunch {self.pack_path} welcome_guide_interface.launch", False)
            ]
        )
        scroll_layout.addWidget(welcome_group)
        
        # 多功能AI开发套件
        ai_kit_group = self.create_upper_module_group(
            "多功能AI开发套件",
            [
                ("云台与传感器节点", f"roslaunch {self.pack_path} qt_multi_ai_node.launch",False),
                ("双深度相机", f"roslaunch {self.pack_path} multi_camera.launch", False),
                ("rqt图像工具", f"rosrun rqt_image_view rqt_image_view", False),
                ("麦克风阵列测试工具", f"roslaunch {self.pack_path} qt_mic_gui.launch", False),
                ("AI套件传感器测试工具", f"roslaunch {self.pack_path} qt_multi_ai_gui.launch", False)
            ]
        )
        scroll_layout.addWidget(ai_kit_group)
        
        # 机械臂
        arm_group = self.create_upper_module_group(
            "机械臂",
            [
                ("MoveIt!工具", f"roslaunch {self.pack_path} qt_arm_moveit.launch", False),
                ("USB摄像头", f"roslaunch {self.pack_path} qt_usbcam_arm.launch", False),
                ("机械臂测试工具", f"roslaunch {self.pack_path} qt_arm_gui.launch", False)
            ]
        )
        scroll_layout.addWidget(arm_group)
        
        # 送餐模块
        delivery_group = self.create_upper_module_group(
            "送餐模块",
            [
                ("急停状态", f"roslaunch {self.pack_path} qt_deli_estop.launch", True),
                ("送餐界面", f"roslaunch {self.pack_path} delivery_interface.launch", False)
            ]
        )
        scroll_layout.addWidget(delivery_group)
        
        # 送物模块
        transport_group = self.create_upper_module_group(
            "送物模块",
            [
                ("送物界面", f"roslaunch {self.pack_path} transport_interface.launch", False),
                ("送物模块测试工具", f"roslaunch {self.pack_path} qt_delivery_gui.launch", False)
            ]
        )
        scroll_layout.addWidget(transport_group)
        
        # 工业物流模块
        logistics_group = self.create_upper_module_group(
            "工业物流模块",
            [
                ("工业物流界面", f"roslaunch {self.pack_path} logistics_interface.launch", False),
                ("工业物流传感器测试工具", f"roslaunch {self.pack_path} qt_deli_sensor_gui.launch", False)
            ]
        )
        scroll_layout.addWidget(logistics_group)
        
        # 安防模块
        security_group = self.create_upper_module_group(
            "安防模块",
            [
                ("工业云台相机", f"roslaunch {self.pack_path} qt_pan_tilt_camera.launch", False),
                ("安防模块传感器测试工具", f"roslaunch {self.pack_path} qt_security_sensors_gui.launch", False)
            ]
        )
        scroll_layout.addWidget(security_group)
        
        # 喷雾消杀模块
        spray_group = self.create_upper_module_group(
            "喷雾消杀模块",
            [
                ("喷雾模块硬件系统", f"roslaunch {self.pack_path} qt_spray_gui.launch", False),
            ]
        )
        scroll_layout.addWidget(spray_group)
        
        # 添加伸缩空间
        scroll_layout.addStretch()
        
        # 设置滚动区域内容
        scroll_area.setWidget(scroll_content)
        layout.addWidget(scroll_area)
        
        widget.setLayout(layout)
        return widget
    
    def create_upper_module_group(self, title, buttons_info):
        """创建上装模块组"""
        group = QGroupBox(title)
        group.setStyleSheet("""
            QGroupBox {
                font-weight: bold;
                font-size: 10px;
                border: 1px solid #3498db;
                border-radius: 2px;
                margin-top: 3px;
                padding-top: 8px;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 3px;
                padding: 0 2px 0 2px;
            }
        """)
        
        layout = QVBoxLayout()
        layout.setSpacing(3)
        
        for btn_info in buttons_info:
            # 按钮信息现在包含3个元素：文本、命令、是否在新终端中运行
            btn_text, cmd, use_terminal = btn_info
            
            # 创建按钮名称，用于状态管理
            btn_key = f"{title}_{btn_text}".replace(" ", "_").replace("!", "").lower()
            
            # 初始化状态
            if btn_key not in self.upper_processes:
                self.upper_processes[btn_key] = {
                    'running': False,
                    'process': None,
                    'use_terminal': use_terminal,  # 添加是否使用终端的标志
                    'terminal_process': None  # 添加终端进程记录
                }
            else:
                # 更新现有状态的use_terminal标志
                self.upper_processes[btn_key]['use_terminal'] = use_terminal
            
            btn = QPushButton(btn_text)
            btn.setMinimumHeight(28)
            btn.setProperty('command', cmd)
            btn.setProperty('btn_key', btn_key)
            btn.setProperty('use_terminal', use_terminal)  # 将是否使用终端的信息存储在按钮属性中
            
            # 设置初始样式
            state = self.upper_processes[btn_key]
            if state['running']:
                btn.setText(f"关闭{btn_text}")
                btn.setStyleSheet("""
                    QPushButton {
                        background-color: #f1c40f;
                        color: black;
                        border-radius: 2px;
                        padding: 4px;
                        font-size: 11px;
                    }
                    QPushButton:hover {
                        background-color: #f39c12;
                    }
                """)
            else:
                btn.setText(btn_text)
                btn.setStyleSheet("""
                    QPushButton {
                        background-color: #3498db;
                        color: white;
                        border-radius: 2px;
                        padding: 4px;
                        font-size: 11px;
                    }
                    QPushButton:hover {
                        background-color: #2980b9;
                    }
                """)
            
            # 连接点击事件
            btn.clicked.connect(lambda checked, b=btn: self.toggle_upper_module(b))
            layout.addWidget(btn)
        
        group.setLayout(layout)
        return group
    
    def create_lidar_page(self):
        """创建激光雷达页面"""
        widget = QWidget()
        layout = QVBoxLayout()
        layout.setContentsMargins(5, 5, 5, 5)
        layout.setSpacing(4)
        
        # 标题
        title = QLabel("激光雷达应用控制")
        title.setStyleSheet("""
            QLabel {
                font-size: 13px; 
                font-weight: bold; 
                margin: 2px 0;
            }
        """)
        title.setAlignment(Qt.AlignCenter)
        layout.addWidget(title)
        
        # 创建滚动区域
        scroll_area = QScrollArea()
        scroll_area.setWidgetResizable(True)
        scroll_area.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        scroll_area.setVerticalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        
        # 滚动内容部件
        scroll_content = QWidget()
        scroll_layout = QVBoxLayout(scroll_content)
        scroll_layout.setContentsMargins(5, 5, 5, 5)
        scroll_layout.setSpacing(8)
        
        # 基础应用组
        basic_group = self.create_lidar_group(
            "基础应用",
            [
                ("激光雷达Rivz界面", f"roslaunch {self.pack_path} lidar_display.launch", False),
                ("rqt_reconfigure调试工具", f"rosrun rqt_reconfigure rqt_reconfigure", False)
            ]
        )
        scroll_layout.addWidget(basic_group)
        
        # 智能应用组
        smart_group = self.create_lidar_group(
            "智能应用",
            [
                ("激光雷达避障", f"roslaunch {self.pack_path} laser_Avoidance.launch", False),
                ("激光雷达警卫", f"roslaunch {self.pack_path} laser_Warning.launch", False),
                ("激光雷达跟随", f"roslaunch {self.pack_path} laser_Tracker.launch", False)
            ]
        )
        scroll_layout.addWidget(smart_group)
        
        # 添加伸缩空间
        scroll_layout.addStretch()
        
        # 设置滚动区域内容
        scroll_area.setWidget(scroll_content)
        layout.addWidget(scroll_area)
        
        widget.setLayout(layout)
        return widget
    
    def create_lidar_group(self, title, buttons_info):
        """创建激光雷达模块组"""
        group = QGroupBox(title)
        group.setStyleSheet("""
            QGroupBox {
                font-weight: bold;
                font-size: 10px;
                border: 1px solid #3498db;
                border-radius: 2px;
                margin-top: 3px;
                padding-top: 8px;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 3px;
                padding: 0 2px 0 2px;
            }
        """)
        
        layout = QVBoxLayout()
        layout.setSpacing(3)
        
        for btn_info in buttons_info:
            # 按钮信息包含3个元素：文本、命令、是否保存地图按钮（激光雷达页面均为False）
            btn_text, cmd, is_special = btn_info
            
            # 创建按钮名称，用于状态管理
            btn_key = f"{title}_{btn_text}".replace(" ", "_").replace("-", "_").replace("!", "").lower()
            
            # 初始化状态
            if btn_key not in self.lidar_processes:
                self.lidar_processes[btn_key] = {
                    'running': False,
                    'process': None,
                    'terminal_process': None
                }
            
            btn = QPushButton(btn_text)
            btn.setMinimumHeight(28)
            btn.setProperty('command', cmd)
            btn.setProperty('btn_key', btn_key)
            
            # 设置初始样式
            state = self.lidar_processes[btn_key]
            if state['running']:
                btn.setText(f"关闭{btn_text}")
                btn.setStyleSheet("""
                    QPushButton {
                        background-color: #f1c40f;
                        color: black;
                        border-radius: 2px;
                        padding: 4px;
                        font-size: 11px;
                    }
                    QPushButton:hover {
                        background-color: #f39c12;
                    }
                """)
            else:
                btn.setText(btn_text)
                btn.setStyleSheet("""
                    QPushButton {
                        background-color: #3498db;
                        color: white;
                        border-radius: 2px;
                        padding: 4px;
                        font-size: 11px;
                    }
                    QPushButton:hover {
                        background-color: #2980b9;
                    }
                """)
            
            # 连接点击事件
            btn.clicked.connect(lambda checked, b=btn: self.toggle_lidar_module(b))
            layout.addWidget(btn)
        
        group.setLayout(layout)
        return group
    
    def create_vision_page(self):
        """创建视觉应用页面"""
        widget = QWidget()
        layout = QVBoxLayout()
        layout.setContentsMargins(5, 5, 5, 5)
        layout.setSpacing(4)
        
        # 标题
        title = QLabel("视觉应用模块控制")
        title.setStyleSheet("""
            QLabel {
                font-size: 13px; 
                font-weight: bold; 
                margin: 2px 0;
            }
        """)
        title.setAlignment(Qt.AlignCenter)
        layout.addWidget(title)
        
        # 创建滚动区域
        scroll_area = QScrollArea()
        scroll_area.setWidgetResizable(True)
        scroll_area.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        scroll_area.setVerticalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        
        # 滚动内容部件
        scroll_content = QWidget()
        scroll_layout = QVBoxLayout(scroll_content)
        scroll_layout.setContentsMargins(5, 5, 5, 5)
        scroll_layout.setSpacing(8)
        
        # 基础视觉功能组
        basic_vision_group = self.create_vision_group(
            "基础视觉功能",
            [
                ("启动深度相机", f"roslaunch {self.pack_path} qt_astrapro.launch", False),
                ("二维码识别", f"roslaunch {self.pack_path} qt_QRcode_Parsing_deepcam.launch", False),
                ("人体姿态估计和目标检测", f"roslaunch {self.pack_path} qt_target_detection_deepcam.launch", False),
                ("HLS颜色过滤", f"roslaunch {self.pack_path} qt_hls_color_filter_deepcam.launch", False),
                ("AR视觉", f"roslaunch {self.pack_path} qt_simple_AR_deepcam.launch", False),
                ("AR二维码", f"roslaunch {self.pack_path} qt_ar_track_deepcam.launch", False),
            ]
        )
        scroll_layout.addWidget(basic_vision_group)
        
        # 检测识别组
        detection_group = self.create_vision_group(
            "检测与识别",
            [
                ("手部检测", f"roslaunch {self.pack_path} qt_HandDetector_deepcam.launch", False),
                ("整体检测（手+脸+姿态）", f"roslaunch {self.pack_path} qt_Holistic_deepcam.launch", False),
                ("人脸识别", f"roslaunch {self.pack_path} qt_FaceDetection_deepcam.launch", False),
                ("人脸特效", f"roslaunch {self.pack_path} qt_FaceLandmarks_deepcam.launch", False),
                ("三维物体识别", f"roslaunch {self.pack_path} qt_Objectron_deepcam.launch", False),
            ]
        )
        scroll_layout.addWidget(detection_group)
        
        # 交互应用组
        interaction_group = self.create_vision_group(
            "交互应用",
            [
                ("三维画笔", f"roslaunch {self.pack_path} qt_VirtualPaint_deepcam.launch", False),
                ("手指控制", f"roslaunch {self.pack_path} qt_HandCtrl_deepcam.launch", False),
                ("手势识别", f"roslaunch {self.pack_path} qt_GestureRecognition_deepcam.launch", False),
            ]
        )
        scroll_layout.addWidget(interaction_group)
        
        # 追踪应用组
        tracking_group = self.create_vision_group(
            "追踪应用",
            [
                ("颜色追踪", f"roslaunch {self.pack_path} qt_colorTracker.launch", False),
                ("物体追踪", f"roslaunch {self.pack_path} qt_KCFTracker.launch", False),
                ("车道线检测", f"roslaunch {self.pack_path} qt_follow_line.launch", False),
            ]
        )
        scroll_layout.addWidget(tracking_group)
        
        # 调试工具组
        debug_group = self.create_vision_group(
            "调试工具",
            [
                ("rqt_reconfigure调试工具", f"rosrun rqt_reconfigure rqt_reconfigure", False),
            ]
        )
        scroll_layout.addWidget(debug_group)
        
        # 添加伸缩空间
        scroll_layout.addStretch()
        
        # 设置滚动区域内容
        scroll_area.setWidget(scroll_content)
        layout.addWidget(scroll_area)
        
        widget.setLayout(layout)
        return widget
    
    def create_vision_group(self, title, buttons_info):
        """创建视觉应用模块组"""
        group = QGroupBox(title)
        group.setStyleSheet("""
            QGroupBox {
                font-weight: bold;
                font-size: 10px;
                border: 1px solid #3498db;
                border-radius: 2px;
                margin-top: 3px;
                padding-top: 8px;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 3px;
                padding: 0 2px 0 2px;
            }
        """)
        
        layout = QVBoxLayout()
        layout.setSpacing(3)
        
        for btn_info in buttons_info:
            # 按钮信息包含3个元素：文本、命令、是否特殊处理
            btn_text, cmd, is_special = btn_info
            
            # 创建按钮名称，用于状态管理
            btn_key = f"{title}_{btn_text}".replace(" ", "_").replace("-", "_").replace("(", "").replace(")", "").replace("+", "plus").lower()
            
            # 初始化状态
            if btn_key not in self.vision_processes:
                self.vision_processes[btn_key] = {
                    'running': False,
                    'process': None,
                    'terminal_process': None
                }
            
            btn = QPushButton(btn_text)
            btn.setMinimumHeight(28)
            btn.setProperty('command', cmd)
            btn.setProperty('btn_key', btn_key)
            btn.setProperty('is_rqt', 'rqt_reconfigure' in cmd)  # 标记是否为rqt工具
            
            # 设置初始样式
            state = self.vision_processes[btn_key]
            if state['running']:
                btn.setText(f"关闭{btn_text}")
                btn.setStyleSheet("""
                    QPushButton {
                        background-color: #f1c40f;
                        color: black;
                        border-radius: 2px;
                        padding: 4px;
                        font-size: 11px;
                    }
                    QPushButton:hover {
                        background-color: #f39c12;
                    }
                """)
            else:
                btn.setText(btn_text)
                btn.setStyleSheet("""
                    QPushButton {
                        background-color: #3498db;
                        color: white;
                        border-radius: 2px;
                        padding: 4px;
                        font-size: 11px;
                    }
                    QPushButton:hover {
                        background-color: #2980b9;
                    }
                """)
            
            # 连接点击事件
            btn.clicked.connect(lambda checked, b=btn: self.toggle_vision_module(b))
            layout.addWidget(btn)
        
        group.setLayout(layout)
        return group
    
    def toggle_upper_module(self, button):
        """切换上装模块"""
        cmd = button.property('command')
        btn_key = button.property('btn_key')
        use_terminal = button.property('use_terminal')
        
        state = self.upper_processes[btn_key]
        
        if not state['running']:
            # 启动模块
            print(f"启动上装模块: {cmd}")
            
            try:
                if use_terminal:
                    # 在新终端中运行
                    ros_master_uri = "ROS_MASTER_URI=http://localhost:11311"
                    full_cmd = f"{ros_master_uri} {cmd}"
                    terminal_cmd = f"gnome-terminal -- bash -c '{full_cmd}; exec bash'"
                    
                    terminal_process = subprocess.Popen(terminal_cmd, shell=True)
                    state['terminal_process'] = terminal_process
                    state['process'] = None  # 终端模式下没有直接的后台进程
                else:
                    # 在后台运行
                    process = subprocess.Popen(
                        cmd,
                        shell=True,
                        preexec_fn=os.setsid
                    )
                    state['process'] = process
                    state['terminal_process'] = None
                
                state['running'] = True
                
                # 更新按钮文本和样式
                btn_text = button.text()
                button.setText(f"关闭{btn_text}")
                button.setStyleSheet("""
                    QPushButton {
                        background-color: #f1c40f;
                        color: black;
                        border-radius: 2px;
                        padding: 4px;
                        font-size: 11px;
                    }
                    QPushButton:hover {
                        background-color: #f39c12;
                    }
                """)
                
            except Exception as e:
                QMessageBox.critical(self, "错误", f"启动模块失败: {str(e)}")
                state['running'] = False
        
        else:
            # 关闭模块
            print(f"关闭上装模块: {cmd}")
            
            if use_terminal and state['terminal_process']:
                # 关闭终端进程
                try:
                    # 从命令中提取launch文件名，用于查找进程
                    launch_file = cmd.split(" ")[-1]
                    pattern_name = launch_file.replace(".launch", "")
                    subprocess.run(f"pkill -f 'gnome-terminal.*{pattern_name}'", 
                                 shell=True, timeout=1,
                                 stdout=subprocess.DEVNULL,
                                 stderr=subprocess.DEVNULL)
                except:
                    pass
            elif state['process'] and state['process'].pid:
                # 关闭后台进程
                try:
                    os.killpg(os.getpgid(state['process'].pid), signal.SIGTERM)
                except:
                    pass
            
            # 清理相关进程
            try:
                launch_file = cmd.split(" ")[-1]
                subprocess.run(f"pkill -f '{launch_file}'", 
                             shell=True, timeout=1,
                             stdout=subprocess.DEVNULL,
                             stderr=subprocess.DEVNULL)
            except:
                pass
            
            # 重置状态
            state['process'] = None
            state['terminal_process'] = None
            state['running'] = False
            
            # 更新按钮文本和样式
            btn_text = button.text().replace("关闭", "")
            button.setText(btn_text)
            button.setStyleSheet("""
                QPushButton {
                    background-color: #3498db;
                    color: white;
                    border-radius: 2px;
                    padding: 4px;
                    font-size: 11px;
                }
                QPushButton:hover {
                    background-color: #2980b9;
                }
            """)
    
    def toggle_lidar_module(self, button):
        """切换激光雷达模块"""
        cmd = button.property('command')
        btn_key = button.property('btn_key')
        
        state = self.lidar_processes[btn_key]
        
        if not state['running']:
            # 启动模块
            print(f"启动激光雷达模块: {cmd}")
            
            try:
                # 检查是否为rqt_reconfigure调试工具（可能需要特殊处理）
                if "rqt_reconfigure" in cmd:
                    # rqt_reconfigure可能需要在终端中运行
                    process = subprocess.Popen(
                        cmd,
                        shell=True,
                        preexec_fn=os.setsid
                    )
                else:
                    # 其他激光雷达模块
                    process = subprocess.Popen(
                        cmd,
                        shell=True,
                        preexec_fn=os.setsid
                    )
                
                state['process'] = process
                state['running'] = True
                
                # 更新按钮文本和样式
                btn_text = button.text()
                button.setText(f"关闭{btn_text}")
                button.setStyleSheet("""
                    QPushButton {
                        background-color: #f1c40f;
                        color: black;
                        border-radius: 2px;
                        padding: 4px;
                        font-size: 11px;
                    }
                    QPushButton:hover {
                        background-color: #f39c12;
                    }
                """)
                
            except Exception as e:
                QMessageBox.critical(self, "错误", f"启动激光雷达模块失败: {str(e)}")
                state['running'] = False
        
        else:
            # 关闭模块
            print(f"关闭激光雷达模块: {cmd}")
            
            # 关闭进程组
            if state['process'] and state['process'].pid:
                try:
                    os.killpg(os.getpgid(state['process'].pid), signal.SIGTERM)
                    print(f"已关闭激光雷达模块进程组 (PID: {state['process'].pid})")
                except Exception as e:
                    print(f"关闭激光雷达模块进程组失败: {e}")
            
            # 对于rqt_reconfigure可能需要特殊处理
            if "rqt_reconfigure" in cmd:
                try:
                    subprocess.run("pkill -f 'rqt_reconfigure'", 
                                 shell=True, timeout=1,
                                 stdout=subprocess.DEVNULL,
                                 stderr=subprocess.DEVNULL)
                except:
                    pass
            else:
                # 清理相关进程
                try:
                    launch_file = cmd.split(" ")[-1]
                    subprocess.run(f"pkill -f '{launch_file}'", 
                                 shell=True, timeout=1,
                                 stdout=subprocess.DEVNULL,
                                 stderr=subprocess.DEVNULL)
                except:
                    pass
            
            # 重置状态
            state['process'] = None
            state['running'] = False
            
            # 更新按钮文本和样式
            btn_text = button.text().replace("关闭", "")
            button.setText(btn_text)
            button.setStyleSheet("""
                QPushButton {
                    background-color: #3498db;
                    color: white;
                    border-radius: 2px;
                    padding: 4px;
                    font-size: 11px;
                }
                QPushButton:hover {
                    background-color: #2980b9;
                }
            """)
    
    def toggle_vision_module(self, button):
        """切换视觉应用模块"""
        cmd = button.property('command')
        btn_key = button.property('btn_key')
        is_rqt = button.property('is_rqt')
        
        state = self.vision_processes[btn_key]
        
        if not state['running']:
            # 启动模块
            print(f"启动视觉应用模块: {cmd}")
            
            try:
                # 检查是否为rqt_reconfigure调试工具（可能需要特殊处理）
                if is_rqt:
                    # rqt_reconfigure可能需要在终端中运行
                    process = subprocess.Popen(
                        cmd,
                        shell=True,
                        preexec_fn=os.setsid
                    )
                else:
                    # 其他视觉模块
                    process = subprocess.Popen(
                        cmd,
                        shell=True,
                        preexec_fn=os.setsid
                    )
                
                state['process'] = process
                state['running'] = True
                
                # 更新按钮文本和样式
                btn_text = button.text()
                button.setText(f"关闭{btn_text}")
                button.setStyleSheet("""
                    QPushButton {
                        background-color: #f1c40f;
                        color: black;
                        border-radius: 2px;
                        padding: 4px;
                        font-size: 11px;
                    }
                    QPushButton:hover {
                        background-color: #f39c12;
                    }
                """)
                
            except Exception as e:
                QMessageBox.critical(self, "错误", f"启动视觉应用模块失败: {str(e)}")
                state['running'] = False
        
        else:
            # 关闭模块
            print(f"关闭视觉应用模块: {cmd}")
            
            # 关闭进程组
            if state['process'] and state['process'].pid:
                try:
                    os.killpg(os.getpgid(state['process'].pid), signal.SIGTERM)
                    print(f"已关闭视觉应用模块进程组 (PID: {state['process'].pid})")
                except Exception as e:
                    print(f"关闭视觉应用模块进程组失败: {e}")
            
            # 对于rqt_reconfigure可能需要特殊处理
            if is_rqt:
                try:
                    subprocess.run("pkill -f 'rqt_reconfigure'", 
                                 shell=True, timeout=1,
                                 stdout=subprocess.DEVNULL,
                                 stderr=subprocess.DEVNULL)
                except:
                    pass
            else:
                # 清理相关进程
                try:
                    launch_file = cmd.split(" ")[-1]
                    subprocess.run(f"pkill -f '{launch_file}'", 
                                 shell=True, timeout=1,
                                 stdout=subprocess.DEVNULL,
                                 stderr=subprocess.DEVNULL)
                except:
                    pass
            
            # 重置状态
            state['process'] = None
            state['running'] = False
            
            # 更新按钮文本和样式
            btn_text = button.text().replace("关闭", "")
            button.setText(btn_text)
            button.setStyleSheet("""
                QPushButton {
                    background-color: #3498db;
                    color: white;
                    border-radius: 2px;
                    padding: 4px;
                    font-size: 11px;
                }
                QPushButton:hover {
                    background-color: #2980b9;
                }
            """)
    
    
    # ============== 按钮样式更新函数 ==============
    
    def update_button_style(self, button_type):
        """统一更新按钮样式"""
        state = self.processes_state[button_type]
        
        if button_type == 'agv_hardware':
            if state['running']:
                self.agv_btn.setText("关闭AGV硬件")
                self.agv_btn.setStyleSheet("""
                    QPushButton {
                        background-color: #f1c40f;
                        color: black;
                        border-radius: 2px;
                        padding: 4px;
                        font-size: 11px;
                    }
                    QPushButton:hover {
                        background-color: #f39c12;
                    }
                """)
            else:
                self.agv_btn.setText("启动AGV硬件")
                self.agv_btn.setStyleSheet("""
                    QPushButton {
                        background-color: #3498db;
                        color: white;
                        border-radius: 2px;
                        padding: 4px;
                        font-size: 11px;
                    }
                    QPushButton:hover {
                        background-color: #2980b9;
                    }
                """)
                
        elif button_type == 'rviz':
            if state['running']:
                self.rviz_btn.setText("关闭可视化界面")
                self.rviz_btn.setStyleSheet("""
                    QPushButton {
                        background-color: #f1c40f;
                        color: black;
                        border-radius: 2px;
                        padding: 4px;
                        font-size: 11px;
                    }
                    QPushButton:hover {
                        background-color: #f39c12;
                    }
                """)
            else:
                self.rviz_btn.setText("显示可视化界面")
                self.rviz_btn.setStyleSheet("""
                    QPushButton {
                        background-color: #3498db;
                        color: white;
                        border-radius: 2px;
                        padding: 4px;
                        font-size: 11px;
                    }
                    QPushButton:hover {
                        background-color: #2980b9;
                    }
                """)
                
        elif button_type == 'voice':
            if state['running']:
                self.voice_btn.setText("关闭离线语音")
                self.voice_btn.setStyleSheet("""
                    QPushButton {
                        background-color: #f1c40f;
                        color: black;
                        border-radius: 2px;
                        padding: 4px;
                        font-size: 11px;
                    }
                    QPushButton:hover {
                        background-color: #f39c12;
                    }
                """)
            else:
                self.voice_btn.setText("离线语音交互")
                self.voice_btn.setStyleSheet("""
                    QPushButton {
                        background-color: #3498db;
                        color: white;
                        border-radius: 2px;
                        padding: 4px;
                        font-size: 11px;
                    }
                    QPushButton:hover {
                        background-color: #2980b9;
                    }
                """)
                
        elif button_type == 'keyboard':
            if state['running']:
                self.keyboard_btn.setText("关闭键盘控制")
                self.keyboard_btn.setStyleSheet("""
                    QPushButton {
                        background-color: #f1c40f;
                        color: black;
                        border-radius: 2px;
                        padding: 4px;
                        font-size: 11px;
                    }
                    QPushButton:hover {
                        background-color: #f39c12;
                    }
                """)
            else:
                self.keyboard_btn.setText("启动键盘控制")
                self.keyboard_btn.setStyleSheet("""
                    QPushButton {
                        background-color: #3498db;
                        color: white;
                        border-radius: 2px;
                        padding: 4px;
                        font-size: 11px;
                    }
                    QPushButton:hover {
                        background-color: #2980b9;
                    }
                """)
                
        elif button_type == 'slam':
            if state['running']:
                self.slam_btn.setText("关闭SLAM建图")
                self.slam_btn.setStyleSheet("""
                    QPushButton {
                        background-color: #f1c40f;
                        color: black;
                        border-radius: 2px;
                        padding: 4px;
                        font-size: 11px;
                    }
                    QPushButton:hover {
                        background-color: #f39c12;
                    }
                """)
            else:
                self.slam_btn.setText("启动SLAM建图")
                self.slam_btn.setStyleSheet("""
                    QPushButton {
                        background-color: #3498db;
                        color: white;
                        border-radius: 2px;
                        padding: 4px;
                        font-size: 11px;
                    }
                    QPushButton:hover {
                        background-color: #2980b9;
                    }
                """)
    
    def update_module_button_style(self, button, is_selected):
        """更新AGV模块按钮样式"""
        if is_selected:
            button.setStyleSheet("""
                QPushButton {
                    background-color: #27ae60;
                    color: white;
                    border-radius: 2px;
                    padding: 2px;
                    font-size: 10px;
                }
                QPushButton:hover {
                    background-color: #229954;
                }
            """)
        else:
            button.setStyleSheet("""
                QPushButton {
                    background-color: #95a5a6;
                    color: white;
                    border-radius: 2px;
                    padding: 2px;
                    font-size: 10px;
                }
                QPushButton:hover {
                    background-color: #7f8c8d;
                }
            """)
    
    def update_maptype_button_style(self, button, is_selected):
        """更新SLAM建图方法按钮样式"""
        if is_selected:
            button.setStyleSheet("""
                QPushButton {
                    background-color: #9b59b6;
                    color: white;
                    border-radius: 2px;
                    padding: 2px;
                    font-size: 10px;
                }
                QPushButton:hover {
                    background-color: #8e44ad;
                }
            """)
        else:
            button.setStyleSheet("""
                QPushButton {
                    background-color: #95a5a6;
                    color: white;
                    border-radius: 2px;
                    padding: 2px;
                    font-size: 10px;
                }
                QPushButton:hover {
                    background-color: #7f8c8d;
                }
            """)
    
    # ============== 按钮功能函数 ==============
    
    def toggle_module(self, module_name, button, checked):
        """切换AGV模块选择状态（多选）"""
        # 如果AGV硬件正在运行，不允许更改模块选择
        if self.processes_state['agv_hardware']['running']:
            button.setChecked(not checked)
            return
            
        # AGV模块是多选模式，不需要取消其他按钮的选择状态
        self.modules_selected[module_name] = checked
        self.update_module_button_style(button, checked)
    
    def toggle_slam_module(self, maptype, button, checked):
        """切换SLAM建图方法选择状态（单选）"""
        # 如果SLAM正在运行，不允许更改选择
        if self.processes_state['slam']['running']:
            button.setChecked(not checked)
            QMessageBox.warning(self, "提示", "请先关闭SLAM建图以更改建图方法！")
            return
        
        # 取消其他按钮的选择状态（单选模式）
        if checked:
            for name in self.slam_selected:
                if name != maptype and self.slam_selected[name]:
                    self.slam_selected[name] = False
                    # 找到对应的按钮并更新样式
                    if name == 'Gmapping':
                        self.Gmapping_btn.setChecked(False)
                        self.update_maptype_button_style(self.Gmapping_btn, False)
                    elif name == 'Hector':
                        self.Hector_btn.setChecked(False)
                        self.update_maptype_button_style(self.Hector_btn, False)
                    elif name == 'Karto':
                        self.Karto_btn.setChecked(False)
                        self.update_maptype_button_style(self.Karto_btn, False)
                    elif name == 'Cartographer':
                        self.Cartographer_btn.setChecked(False)
                        self.update_maptype_button_style(self.Cartographer_btn, False)
                    elif name == 'Web':
                        self.Web_btn.setChecked(False)
                        self.update_maptype_button_style(self.Web_btn, False)
        
        self.slam_selected[maptype] = checked
        self.update_maptype_button_style(button, checked)
    
    def reset_all_modules(self):
        """重置所有AGV模块选择状态"""
        for key in self.modules_selected.keys():
            self.modules_selected[key] = False
        
        module_buttons = [
            (self.depth_camera_btn, 'depth_camera'),
            (self.knob_screen_btn, 'knob_screen'),
            (self.lidar_btn, 'lidar'),
            (self.mag_nav_btn, 'mag_nav'),
            (self.ar_marker_btn, 'ar_marker')
        ]
        
        for button, module_name in module_buttons:
            button.setChecked(False)
            self.update_module_button_style(button, False)
    
    def reset_slam_modules(self):
        """重置所有SLAM方法选择状态"""
        for key in self.slam_selected.keys():
            self.slam_selected[key] = False
        
        slam_buttons = [
            (self.Gmapping_btn, 'Gmapping'),
            (self.Hector_btn, 'Hector'),
            (self.Karto_btn, 'Karto'),
            (self.Cartographer_btn, 'Cartographer'),
            (self.Web_btn, 'Web')
        ]
        
        for button, slam_name in slam_buttons:
            button.setChecked(False)
            self.update_maptype_button_style(button, False)
    
    def toggle_agv_hardware(self):
        """切换AGV硬件"""
        state = self.processes_state['agv_hardware']
        
        if not state['running']:
            # 启动AGV硬件主程序
            base_cmd = "roslaunch aihitplt_dev qt_bringup.launch"
            args = []
            
            # 添加模块参数（磁导航除外）
            if self.modules_selected['depth_camera']:
                args.append("depth_camera:=true")
            if self.modules_selected['knob_screen']:
                args.append("round_panel:=true")
            if self.modules_selected['lidar']:
                args.append("lidar:=true")
            if self.modules_selected['ar_marker']:
                args.append("AR_rec:=true")
            
            full_cmd = f"{base_cmd} {' '.join(args)}" if args else base_cmd
            
            print(f"启动AGV硬件: {full_cmd}")
            
            process = subprocess.Popen(
                full_cmd,
                shell=True,
                preexec_fn=os.setsid
            )
            state['process'] = process
            

            time.sleep(0.5)
            
            if self.modules_selected['mag_nav']:

                ros_master_uri = "ROS_MASTER_URI=http://localhost:11311"
                mag_cmd = f"{ros_master_uri} roslaunch aihitplt_dev qt_mag_follower.launch"
                terminal_cmd = f"gnome-terminal -- bash -c '{mag_cmd}; exec bash'"
                mag_process = subprocess.Popen(terminal_cmd, shell=True)
                state['mag_terminal'] = mag_process
                print(f"在新终端中启动磁导航: {mag_cmd}")
            
            state['running'] = True
            self.sidebar_status_label.setText("AGV运行中")
            self.sidebar_status_label.setStyleSheet("""
                QLabel {
                    color: #27ae60; 
                    font-size: 9px; 
                    padding: 3px;
                }
            """)
        
            
        else:
            # 关闭AGV相关进程
            print("关闭AGV硬件")
            
            # 1. 关闭主进程
            if state['process'] and state['process'].pid:
                try:
                    os.killpg(os.getpgid(state['process'].pid), signal.SIGTERM)
                except:
                    pass
            
            # 2. 清理所有相关进程
            kill_patterns = [
                "qt_bringup",
                "aihitplt_robot_node",
                "self_check_node",
                "qt_mag_follower",
                "aihitplt_mag",
            ]
            
            for pattern in kill_patterns:
                try:
                    subprocess.run(f"pkill -f '{pattern}'", 
                                shell=True, timeout=1,
                                stdout=subprocess.DEVNULL,
                                stderr=subprocess.DEVNULL)
                except:
                    pass
            
            # 3. 重置状态
            state['process'] = None
            state['running'] = False
            
            self.sidebar_status_label.setText("就绪")
            self.sidebar_status_label.setStyleSheet("""
                QLabel {
                    color: #95a5a6; 
                    font-size: 9px; 
                    padding: 3px;
                }
            """)
            
            # 重置模块选择
            self.reset_all_modules()
        
        self.update_button_style('agv_hardware')
    
    def toggle_slam(self):
        """切换SLAM建图"""
        state = self.processes_state['slam']
        
        if not state['running']:
            # 检查是否选择了SLAM方法
            selected_slam = [name for name, selected in self.slam_selected.items() if selected]
            if not selected_slam:
                QMessageBox.warning(self, "警告", "请先选择SLAM建图方法！")
                return
            
            # 根据选择的SLAM方法构建命令
            slam_method = selected_slam[0]
            
            cmd = f"roslaunch aihitplt_dev qt_slam_mapping.launch map_type:={slam_method.lower()}"
            
            print(f"启动SLAM建图 ({slam_method}): {cmd}")
            
            try:
                process = subprocess.Popen(
                    cmd,
                    shell=True,
                    preexec_fn=os.setsid
                )
                state['process'] = process
                state['running'] = True
                
            except Exception as e:
                QMessageBox.critical(self, "错误", f"启动SLAM失败: {str(e)}")
                state['running'] = False
            
        else:
            print("关闭SLAM建图")
            
            # 关闭SLAM进程
            if state['process'] and state['process'].pid:
                try:
                    os.killpg(os.getpgid(state['process'].pid), signal.SIGTERM)
                    print(f"已关闭SLAM进程组 (PID: {state['process'].pid})")
                except Exception as e:
                    print(f"关闭SLAM进程组失败: {e}")
            
            # 清理SLAM相关进程（确保所有相关进程都被终止）
            kill_patterns = [
                "qt_keyboard_teleop.launch",
                "slam_mapping.launch",
                "gmapping",
                "hector_mapping",
                "karto",
                "cartographer",
                "roslaunch.*slam",
            ]
            
            for pattern in kill_patterns:
                try:
                    subprocess.run(f"pkill -f '{pattern}'", 
                                shell=True, timeout=1,
                                stdout=subprocess.DEVNULL,
                                stderr=subprocess.DEVNULL)
                except:
                    pass
            
            state['process'] = None
            state['running'] = False
            
            # 重置SLAM方法选择按钮（类似AGV硬件关闭逻辑）
            self.reset_slam_modules()
        
        self.update_button_style('slam')
    
    def toggle_rviz(self):
        """切换可视化界面"""
        state = self.processes_state['rviz']
        
        if not state['running']:
            cmd = "roslaunch aihitplt_dev qt_rviz_display.launch"
            print(f"启动RViz: {cmd}")
            
            process = subprocess.Popen(
                cmd,
                shell=True,
                preexec_fn=os.setsid
            )
            state['process'] = process
            state['running'] = True
            
        else:
            print("关闭RViz")
            
            if state['process'] and state['process'].pid:
                try:
                    os.killpg(os.getpgid(state['process'].pid), signal.SIGTERM)
                except:
                    pass
            
            # 清理RViz进程
            try:
                subprocess.run("pkill -f 'roslaunch.*qt_rviz_display'", 
                             shell=True, timeout=1,
                             stdout=subprocess.DEVNULL,
                             stderr=subprocess.DEVNULL)
            except:
                pass
            
            state['process'] = None
            state['running'] = False
        
        self.update_button_style('rviz')
    
    def toggle_voice(self):
        """切换离线语音"""
        state = self.processes_state['voice']
        
        if not state['running']:
            # 在新终端中启动离线语音
            ros_master_uri = "ROS_MASTER_URI=http://localhost:11311"
            voice_cmd = f"{ros_master_uri} roslaunch aihitplt_dev qt_voice_off_line.launch"
            terminal_cmd = f"gnome-terminal -- bash -c '{voice_cmd}; exec bash'"
            print(f"启动离线语音: {voice_cmd}")
            
            terminal_process = subprocess.Popen(terminal_cmd, shell=True)
            state['terminal_process'] = terminal_process
            state['running'] = True
            
        else:
            print("关闭离线语音")
            
            # 1. 关闭终端进程
            if state['terminal_process'] and state['terminal_process'].pid:
                try:
                    # 使用pkill查找并关闭gnome-terminal进程
                    subprocess.run("pkill -f 'gnome-terminal.*qt_voice_off_line'", 
                                 shell=True, timeout=1,
                                 stdout=subprocess.DEVNULL,
                                 stderr=subprocess.DEVNULL)
                except:
                    pass
            
            # 2. 清理语音相关进程
            kill_patterns = [
                "qt_voice_off_line",
                "voice_interaction",
            ]
            
            for pattern in kill_patterns:
                try:
                    subprocess.run(f"pkill -f '{pattern}'", 
                                 shell=True, timeout=1,
                                 stdout=subprocess.DEVNULL,
                                 stderr=subprocess.DEVNULL)
                except:
                    pass
            
            # 3. 重置状态
            state['terminal_process'] = None
            state['process'] = None
            state['running'] = False
        
        self.update_button_style('voice')
    
    def toggle_keyboard(self):
        """切换键盘控制"""
        state = self.processes_state['keyboard']
        
        if not state['running']:
            # 在新终端中启动键盘控制
            ros_master_uri = "ROS_MASTER_URI=http://localhost:11311"
            keyboard_cmd = f"{ros_master_uri} roslaunch aihitplt_dev qt_keyboard_teleop.launch"
            terminal_cmd = f"gnome-terminal -- bash -c '{keyboard_cmd}; exec bash'"
            print(f"启动键盘控制: {keyboard_cmd}")
            
            terminal_process = subprocess.Popen(terminal_cmd, shell=True)
            state['terminal_process'] = terminal_process
            state['running'] = True
            
        else:
            print("关闭键盘控制")
            
            # 1. 关闭终端进程
            if state['terminal_process'] and state['terminal_process'].pid:
                try:
                    # 使用pkill查找并关闭gnome-terminal进程
                    subprocess.run("pkill -f 'gnome-terminal.*qt_keyboard_teleop'", 
                                 shell=True, timeout=1,
                                 stdout=subprocess.DEVNULL,
                                 stderr=subprocess.DEVNULL)
                except:
                    pass
            
            # 2. 清理键盘控制相关进程
            kill_patterns = [
                "qt_keyboard_teleop",
                "keyboard_teleop",
                "roslaunch.*keyboard_teleop",
            ]
            
            for pattern in kill_patterns:
                try:
                    subprocess.run(f"pkill -f '{pattern}'", 
                                 shell=True, timeout=1,
                                 stdout=subprocess.DEVNULL,
                                 stderr=subprocess.DEVNULL)
                except:
                    pass
            
            # 3. 重置状态
            state['terminal_process'] = None
            state['process'] = None
            state['running'] = False
        
        self.update_button_style('keyboard')
    
    # ============== 其他功能函数 ==============
    
    def check_ros_status(self):
        """自动检查ROS状态"""
        try:
            result = subprocess.run(
                "rosnode list",
                shell=True,
                capture_output=True,
                text=True,
                timeout=2
            )
            if result.returncode == 0 and result.stdout.strip():
                self.ros_status_label.setText("✅ ROS Master 运行中")
                self.ros_status_label.setStyleSheet("""
                    QLabel {
                        color: #27ae60; 
                        font-weight: bold; 
                        font-size: 10px;
                    }
                """)
            else:
                self.ros_status_label.setText("❌ ROS Master 未运行")
                self.ros_status_label.setStyleSheet("""
                    QLabel {
                        color: #e74c3c; 
                        font-weight: bold; 
                        font-size: 10px;
                    }
                """)
        except:
            self.ros_status_label.setText("❌ ROS Master 未运行")
            self.ros_status_label.setStyleSheet("""
                QLabel {
                    color: #e74c3c; 
                    font-weight: bold; 
                    font-size: 10px;
                }
            """)
    
    def switch_page(self, page_id):
        """切换页面"""
        for i in range(self.content_layout.count()):
            widget = self.content_layout.itemAt(i).widget()
            if widget:
                widget.hide()
        
        selected_widget = self.content_layout.itemAt(page_id).widget()
        if selected_widget:
            selected_widget.show()
        
        self.button_group.button(page_id).setChecked(True)
    
    def execute_command(self, command):
        """执行一次性命令"""
        print(f"执行: {command}")
        subprocess.Popen(command, shell=True)
    
    def closeEvent(self, event):
        """窗口关闭时清理所有进程"""
        print("正在关闭程序，清理所有进程...")
        
        # 停止自动刷新定时器
        self.ros_timer.stop()
        
        # 清理所有进程
        for name, state in self.processes_state.items():
            if state['running']:
                # 关闭进程组
                if state['process'] and state['process'].pid:
                    try:
                        os.killpg(os.getpgid(state['process'].pid), signal.SIGTERM)
                        print(f"已关闭{name}进程")
                    except:
                        pass
                
                # 特殊处理AGV的磁导航终端
                if name == 'agv_hardware' and state['mag_terminal']:
                    try:
                        subprocess.run("pkill -f 'gnome-terminal.*qt_mag_follower'", 
                                     shell=True, timeout=1,
                                     stdout=subprocess.DEVNULL,
                                     stderr=subprocess.DEVNULL)
                    except:
                        pass
                
                # 特殊处理离线语音的终端
                if name == 'voice' and state['terminal_process']:
                    try:
                        subprocess.run("pkill -f 'gnome-terminal.*qt_mic_init'", 
                                     shell=True, timeout=1,
                                     stdout=subprocess.DEVNULL,
                                     stderr=subprocess.DEVNULL)
                    except:
                        pass
                
                # 特殊处理键盘控制的终端
                if name == 'keyboard' and state['terminal_process']:
                    try:
                        subprocess.run("pkill -f 'gnome-terminal.*qt_keyboard_teleop'", 
                                     shell=True, timeout=1,
                                     stdout=subprocess.DEVNULL,
                                     stderr=subprocess.DEVNULL)
                    except:
                        pass
        
        # 清理上装模块进程
        for btn_key, state in self.upper_processes.items():
            if state['running']:
                if state['use_terminal'] and state['terminal_process']:
                    # 关闭终端进程
                    try:
                        # 从状态中获取命令信息
                        for btn_info in self._get_all_button_info():
                            if btn_info[0].lower().replace(" ", "_") in btn_key:
                                pattern_name = btn_info[1].split(" ")[-1].replace(".launch", "")
                                subprocess.run(f"pkill -f 'gnome-terminal.*{pattern_name}'", 
                                             shell=True, timeout=1,
                                             stdout=subprocess.DEVNULL,
                                             stderr=subprocess.DEVNULL)
                                break
                    except:
                        pass
                elif state['process'] and state['process'].pid:
                    # 关闭后台进程
                    try:
                        os.killpg(os.getpgid(state['process'].pid), signal.SIGTERM)
                        print(f"已关闭上装模块 {btn_key} 进程")
                    except:
                        pass
        
        # 清理建图导航模块进程
        for btn_key, state in self.mapping_nav_processes.items():
            if state['running'] and state['process'] and state['process'].pid:
                try:
                    os.killpg(os.getpgid(state['process'].pid), signal.SIGTERM)
                    print(f"已关闭建图导航模块 {btn_key} 进程")
                except:
                    pass
        
        # 清理激光雷达模块进程
        for btn_key, state in self.lidar_processes.items():
            if state['running'] and state['process'] and state['process'].pid:
                try:
                    os.killpg(os.getpgid(state['process'].pid), signal.SIGTERM)
                    print(f"已关闭激光雷达模块 {btn_key} 进程")
                except:
                    pass
        
        # 清理视觉应用模块进程
        for btn_key, state in self.vision_processes.items():
            if state['running'] and state['process'] and state['process'].pid:
                try:
                    os.killpg(os.getpgid(state['process'].pid), signal.SIGTERM)
                    print(f"已关闭视觉应用模块 {btn_key} 进程")
                except:
                    pass
        
        # 额外清理
        cleanup_patterns = [
            "roslaunch.*aihitplt",
            "aihitplt_robot_node",
            "rviz",
            "qt_mag_follower",
            "roslaunch.*slam",
            "qt_keyboard_teleop",
            "roslaunch.*aihitplt_upper",
            "roslaunch.*gmapping",
            "roslaunch.*hector",
            "roslaunch.*karto",
            "roslaunch.*cartographer",
            "roslaunch.*vision",
            "roslaunch.*rtabmap",
            "roslaunch.*lidar",  
            "rqt_reconfigure",   
            "roslaunch.*depth_camera",
            "roslaunch.*qrcode",
            "roslaunch.*human_pose",
            "roslaunch.*hls_color",
            "roslaunch.*ar_vision",
            "roslaunch.*hand_detection",
            "roslaunch.*holistic_detection",
            "roslaunch.*face_recognition",
            "roslaunch.*face_effects",
            "roslaunch.*3d_object_recognition",
            "roslaunch.*3d_brush",
            "roslaunch.*finger_control",
            "roslaunch.*gesture_recognition",
            "roslaunch.*color_tracking",
            "roslaunch.*object_tracking",
            "roslaunch.*lane_detection",
        ]
        
        for pattern in cleanup_patterns:
            try:
                subprocess.run(f"pkill -f '{pattern}'", shell=True, timeout=1,
                             stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            except:
                pass
        
        print("程序关闭完成")
        event.accept()
    
    def _get_all_button_info(self):
        """获取所有按钮信息（用于清理进程）"""
        # 这里返回所有按钮信息的列表，格式为[(按钮文本, 命令, 是否使用终端), ...]
        return [
            ("急停状态", "roslaunch aihitplt_dev qt_guide_estop.launch", True),
            ("启动USB摄像头", "roslaunch aihitplt_dev qt_usbcam_greet.launch", False),
            ("启动迎宾导览界面", "roslaunch aihitplt_dev welcome_guide_interface.launch", False),
        ]

if __name__ == '__main__':
    app = QApplication(sys.argv)
    app.setStyle('Fusion')
    window = SidebarApp()
    window.show()
    sys.exit(app.exec_())