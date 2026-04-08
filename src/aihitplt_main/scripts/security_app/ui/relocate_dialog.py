#!/usr/bin/env python3

from PyQt5.QtWidgets import *
from PyQt5.QtCore import Qt, pyqtSignal
from PyQt5.QtGui import QPainter, QBrush, QPen, QColor

class RelocateDialog(QWidget):
    """重定位对话框"""
    
    relocate_confirmed = pyqtSignal(float, float, float)  # x, y, theta
    cancelled = pyqtSignal()  # 新增：取消信号
    
    def __init__(self, robot_x, robot_y, robot_theta, parent=None):
        super().__init__(parent)
        
        # 设置窗口标志
        self.setWindowFlags(Qt.Tool | Qt.FramelessWindowHint)
        
        # 设置背景
        self.setAutoFillBackground(True)
        self.setStyleSheet("""
            QWidget {
                background-color: white;
                border: 2px solid #4a90e2;
                border-radius: 8px;
            }
        """)
        
        # 初始化UI
        self.init_ui(robot_x, robot_y, robot_theta)
    
    def paintEvent(self, event):
        """绘制圆角矩形背景"""
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        
        # 绘制白色背景和蓝色边框
        rect = self.rect()
        painter.setBrush(QBrush(QColor(255, 255, 255)))
        painter.drawRoundedRect(rect.adjusted(1, 1, -1, -1), 8, 8)
        
        super().paintEvent(event)
    
    def init_ui(self, robot_x, robot_y, robot_theta):
        """初始化界面"""
        # 主布局
        main_layout = QVBoxLayout(self)
        main_layout.setContentsMargins(12, 12, 12, 12)
        main_layout.setSpacing(8)
        
        # X坐标输入
        x_layout = QHBoxLayout()
        x_label = QLabel("X:")
        x_label.setFixedWidth(40)
        x_label.setStyleSheet("border: none; color: #333333; font-size: 12px;")
        
        self.x_spin = QDoubleSpinBox()
        self.x_spin.setRange(-1000, 1000)
        self.x_spin.setSingleStep(0.01)
        self.x_spin.setDecimals(3)
        self.x_spin.setValue(robot_x)
        self.x_spin.setStyleSheet("""
            QDoubleSpinBox {
                border: 1px solid #d0d0d0;
                border-radius: 4px;
                padding: 4px 6px;
                background-color: #fafafa;
                color: #333333;
                font-size: 12px;
            }
        """)
        x_layout.addWidget(x_label)
        x_layout.addWidget(self.x_spin)
        main_layout.addLayout(x_layout)
        
        # Y坐标输入
        y_layout = QHBoxLayout()
        y_label = QLabel("Y:")
        y_label.setFixedWidth(40)
        y_label.setStyleSheet("border: none; color: #333333; font-size: 12px;")
        
        self.y_spin = QDoubleSpinBox()
        self.y_spin.setRange(-1000, 1000)
        self.y_spin.setSingleStep(0.01)
        self.y_spin.setDecimals(3)
        self.y_spin.setValue(robot_y)
        self.y_spin.setStyleSheet("""
            QDoubleSpinBox {
                border: 1px solid #d0d0d0;
                border-radius: 4px;
                padding: 4px 6px;
                background-color: #fafafa;
                color: #333333;
                font-size: 12px;
            }
        """)
        y_layout.addWidget(y_label)
        y_layout.addWidget(self.y_spin)
        main_layout.addLayout(y_layout)
        
        # 角度输入
        theta_layout = QHBoxLayout()
        theta_label = QLabel("角度:")
        theta_label.setFixedWidth(40)
        theta_label.setStyleSheet("border: none; color: #333333; font-size: 12px;")
        
        self.theta_spin = QDoubleSpinBox()
        self.theta_spin.setRange(-180, 180)
        self.theta_spin.setSingleStep(5)
        self.theta_spin.setDecimals(1)
        self.theta_spin.setValue(robot_theta)
        self.theta_spin.setSuffix("°")
        self.theta_spin.setStyleSheet("""
            QDoubleSpinBox {
                border: 1px solid #d0d0d0;
                border-radius: 4px;
                padding: 4px 6px;
                background-color: #fafafa;
                color: #333333;
                font-size: 12px;
            }
        """)
        theta_layout.addWidget(theta_label)
        theta_layout.addWidget(self.theta_spin)
        main_layout.addLayout(theta_layout)
        
        
        # 按钮布局
        button_layout = QHBoxLayout()
        button_layout.setSpacing(8)
        
        # 确定按钮（绿色）
        self.confirm_btn = QPushButton("确定")
        self.confirm_btn.setStyleSheet("""
            QPushButton {
                background-color: #4CAF50;
                color: white;
                border: none;
                border-radius: 4px;
                padding: 6px 12px;
                font-weight: bold;
                font-size: 12px;
            }
            QPushButton:hover {
                background-color: #45a049;
            }
        """)
        self.confirm_btn.clicked.connect(self.on_confirm)
        
        # 取消按钮（灰色）
        self.cancel_btn = QPushButton("取消")
        self.cancel_btn.setStyleSheet("""
            QPushButton {
                background-color: #9e9e9e;
                color: white;
                border: none;
                border-radius: 4px;
                padding: 6px 12px;
                font-weight: bold;
                font-size: 12px;
            }
            QPushButton:hover {
                background-color: #757575;
            }
        """)
        self.cancel_btn.clicked.connect(self.on_cancel)
        
        button_layout.addWidget(self.confirm_btn)
        button_layout.addWidget(self.cancel_btn)
        main_layout.addLayout(button_layout)
        
        # 调整窗口大小
        self.setFixedSize(250, 230)
    
    def on_confirm(self):
        """确定按钮点击"""
        x, y, theta = self.x_spin.value(), self.y_spin.value(), self.theta_spin.value()
        self.relocate_confirmed.emit(x, y, theta)
        self.close()
    
    def on_cancel(self):
        """取消按钮点击"""
        self.cancelled.emit()
        self.close()