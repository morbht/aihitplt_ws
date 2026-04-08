#!/usr/bin/env python3

from PyQt5.QtWidgets import *
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QFont

from ui.inspection_table import InspectionTable

class LeftPanel(QFrame):
    """左侧面板（地图和巡检控制）"""
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.parent = parent
        
        self.setFrameStyle(QFrame.Box | QFrame.Raised)
        self.setStyleSheet("background-color: white;")
        
        self.init_ui()
        
    def init_ui(self):
        """初始化界面"""
        layout = QVBoxLayout(self)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(6)
        
        # 地图显示区域
        map_group = self.create_map_group()
        layout.addWidget(map_group)
        
        # 坐标信息
        coord_widget = self.create_coordinate_widget()
        layout.addWidget(coord_widget)
        
        # 巡检控制面板
        inspect_group = self.create_inspection_group()
        layout.addWidget(inspect_group)
        
        # 添加弹簧，使内容顶部对齐
        layout.addStretch()
        

    def create_map_group(self):
        """创建地图显示组"""
        map_group = QGroupBox("地图显示")
        map_group.setStyleSheet("QGroupBox { font-size: 12px; }")
        map_layout = QVBoxLayout()
        
        # 地图工具栏
        toolbar_layout = QHBoxLayout()
        toolbar_layout.setSpacing(4)
        
        # 视图控制按钮 - 现在只创建按钮对象，不立即保存
        view_buttons = [
            ("重定位", self.relocate_robot),
            ("放大", self.zoom_in_map),
            ("缩小", self.zoom_out_map),
            ("打开地图", self.parent.open_map),
            ("保存地图", self.parent.save_map)
        ]
        
        buttons = []  # 临时存储按钮对象
        
        for text, callback in view_buttons:
            btn = QPushButton(text)
            btn.setFixedHeight(24)
            btn.setStyleSheet("""
                QPushButton {
                    background-color: #f5f5f5;
                    color: #333;
                    border: 1px solid #e0e0e0;
                    border-radius: 3px;
                    padding: 2px 6px;
                    font-size: 10px;
                }
                QPushButton:hover {
                    background-color: #e3f2fd;
                    border-color: #1976d2;
                }
            """)
            btn.clicked.connect(callback)
            toolbar_layout.addWidget(btn)
            buttons.append((text, btn))
        
        # 特殊处理：编辑地图按钮需要保存为实例属性
        self.edit_map_btn = QPushButton("编辑地图")
        self.edit_map_btn.setFixedHeight(24)
        self.edit_map_btn.setStyleSheet("""
            QPushButton {
                background-color: #f5f5f5;
                color: #333;
                border: 1px solid #e0e0e0;
                border-radius: 3px;
                padding: 2px 6px;
                font-size: 10px;
            }
            QPushButton:hover {
                background-color: #e3f2fd;
                border-color: #1976d2;
            }
            QPushButton[is_edit_mode="true"] {
                background-color: #ffc107;
                color: #333;
                border: 1px solid #e0e0e0;
                border-radius: 3px;
                font-weight: bold;
                font-size: 10px;
            }
        """)
        self.edit_map_btn.clicked.connect(self.toggle_edit_map)
        toolbar_layout.addWidget(self.edit_map_btn)
        
        toolbar_layout.addStretch()
        map_layout.addLayout(toolbar_layout)
        
        # 地图画布
        if hasattr(self.parent, 'map_canvas'):
            map_layout.addWidget(self.parent.map_canvas)
            
            # 连接编辑模式改变信号
            if hasattr(self.parent.map_canvas, 'edit_mode_changed'):
                self.parent.map_canvas.edit_mode_changed.connect(self.on_edit_mode_changed)
        
        map_group.setLayout(map_layout)
        
        return map_group

    def toggle_edit_map(self):
        """切换地图编辑模式"""
        if hasattr(self.parent.map_canvas, 'set_edit_mode'):
            is_edit_mode = not self.parent.map_canvas.is_edit_mode
            self.parent.map_canvas.set_edit_mode(is_edit_mode)
            
            # 更新按钮文本和样式
            if is_edit_mode:
                self.edit_map_btn.setText("结束编辑")
                self.edit_map_btn.setProperty("is_edit_mode", True)
                self.edit_map_btn.style().polish(self.edit_map_btn)  # 刷新样式
            else:
                self.edit_map_btn.setText("编辑地图")
                self.edit_map_btn.setProperty("is_edit_mode", False)
                self.edit_map_btn.style().polish(self.edit_map_btn)  # 刷新样式
    
    def on_edit_mode_changed(self, enabled):
        """处理编辑模式改变"""
        if enabled:
            self.edit_map_btn.setText("结束编辑")
            self.edit_map_btn.setProperty("is_edit_mode", True)
        else:
            self.edit_map_btn.setText("编辑地图")
            self.edit_map_btn.setProperty("is_edit_mode", False)
        self.edit_map_btn.style().polish(self.edit_map_btn)

    def reset_map_view(self):
        """重置地图视图"""
        if hasattr(self.parent.map_canvas, 'reset_view'):
            self.parent.map_canvas.reset_view()

    def zoom_in_map(self):
        """放大地图"""
        if hasattr(self.parent.map_canvas, 'zoom_in'):
            self.parent.map_canvas.zoom_in()

    def zoom_out_map(self):
        """缩小地图"""
        if hasattr(self.parent.map_canvas, 'zoom_out'):
            self.parent.map_canvas.zoom_out()
            
    def create_coordinate_widget(self):
        """创建坐标信息部件"""
        coord_widget = QWidget()
        coord_layout = QHBoxLayout(coord_widget)
        coord_layout.setSpacing(6)
            
        # 坐标显示项
        coord_items = [
            ("map:", "label_map_pos", "(0.00,0.00)"),
            ("robot:", "label_robot_pos", "(0.00,0.00,0.00)")
        ]
            
        self.coord_labels = {}
            
        for label_text, obj_name, default_value in coord_items:
            hbox = QHBoxLayout()
            hbox.setSpacing(2)
                
            label = QLabel(label_text)
            label.setFont(QFont("Arial", 9))
            label.setStyleSheet("color: #666666;")
            hbox.addWidget(label)
                
            value_label = QLabel(default_value)
            value_label.setObjectName(obj_name)
            value_label.setStyleSheet("""
                QLabel {
                    background-color: #f8f9fa;
                    border: 1px solid #dee2e6;
                    border-radius: 3px;
                    padding: 2px 4px;
                    min-width: 70px;
                    font-family: Consolas;
                    font-size: 9px;
                }
            """)
            hbox.addWidget(value_label)
                
            self.coord_labels[obj_name] = value_label
            coord_layout.addLayout(hbox)
            
        coord_layout.addStretch()
        return coord_widget
        
    def create_inspection_group(self):
            """创建巡检控制组"""
            inspect_group = QGroupBox("巡检控制")
            inspect_group.setStyleSheet("QGroupBox { font-size: 12px; }")
            inspect_layout = QHBoxLayout()
            inspect_layout.setSpacing(8)
            
            # 左侧：按钮区域
            left_buttons = self.create_button_panel()
            inspect_layout.addWidget(left_buttons, 35)
            
            # 右侧：点位信息表格
            right_table = self.create_inspection_table()
            inspect_layout.addWidget(right_table, 65)
            
            inspect_group.setLayout(inspect_layout)
            return inspect_group
            
    def create_button_panel(self):
        widget = QWidget()
        layout = QVBoxLayout(widget)
        layout.setSpacing(6)
        layout.setContentsMargins(0, 0, 0, 0)

        # 开始巡检按钮 
        self.start_btn = QPushButton("开始巡检")
        self.start_btn.setFixedHeight(26)  
        self.start_btn.setStyleSheet("""
            QPushButton {
                background-color: #1976d2;
                color: white;
                font-size: 11px;
            }
        """)
        layout.addWidget(self.start_btn)
        
        # 添加点位按钮
        self.add_point_btn = QPushButton("添加点位")
        self.add_point_btn.setFixedHeight(26)
        self.add_point_btn.setStyleSheet("""
            QPushButton {
                background-color: #1976d2;
                color: white;
                font-size: 11px;
            }
        """)

        layout.addWidget(self.add_point_btn)

        load_points_btn = QPushButton("加载点位")
        load_points_btn.setFixedHeight(26)
        load_points_btn.setStyleSheet("""
            QPushButton {
                background-color: #1976d2;
                color: white;
                font-size: 11px;
            }
        """)
        load_points_btn.clicked.connect(self.parent.load_points)
        layout.addWidget(load_points_btn)

        save_points_btn = QPushButton("保存点位")
        save_points_btn.setFixedHeight(26)
        save_points_btn.setStyleSheet("""
            QPushButton {
                background-color: #1976d2;
                color: white;
                font-size: 11px;
            }
        """)
        save_points_btn.clicked.connect(self.parent.save_points)
        layout.addWidget(save_points_btn)

        self.loop_check = QCheckBox("循环任务")
        layout.addWidget(self.loop_check)

        layout.addStretch()
        return widget
        
    def create_inspection_table(self):
        """创建巡检表格"""
        self.inspection_table = InspectionTable(self.parent)
        return self.inspection_table
        
    def update_coordinates(self, robot_pos):
        """更新坐标显示"""
        if hasattr(self, 'coord_labels'):
            self.coord_labels["label_robot_pos"].setText(
                f"({robot_pos.x():.1f},{robot_pos.y():.1f},0.0)")
            self.coord_labels["label_map_pos"].setText(
                f"({robot_pos.x()/2:.1f},{robot_pos.y()/2:.1f})")
                
    def add_inspection_point(self, status="等待"):
        """添加巡检点"""
        return self.inspection_table.add_point_row(None, status)
        
    def get_point_count(self):
        """获取点位数量"""
        return self.inspection_table.get_point_count()
    
    def relocate_robot(self):
        """重定位机器人"""
        if hasattr(self.parent.map_canvas, 'show_relocate_dialog'):
            self.parent.map_canvas.show_relocate_dialog()
