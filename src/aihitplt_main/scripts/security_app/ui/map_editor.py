#!/usr/bin/env python3

from PyQt5.QtWidgets import *
from PyQt5.QtCore import Qt, pyqtSignal, QPoint, QObject
from PyQt5.QtGui import QFont, QPainter, QBrush, QPen, QColor
import uuid

class TargetPoint:
    """目标点类"""
    def __init__(self, world_x, world_y, screen_x, screen_y, name=None, world_theta=0.0):
        self.world_x = world_x
        self.world_y = world_y
        self.world_theta = world_theta
        self.screen_x = screen_x
        self.screen_y = screen_y
        if name is None:
            self.name = f"目标点_{uuid.uuid4().hex[:4]}"  # 生成简短唯一名称
        else:
            self.name = name
    
    def update_screen_pos(self, screen_x, screen_y):
        """更新屏幕坐标"""
        self.screen_x = screen_x
        self.screen_y = screen_y
    
    def update_world_pos(self, world_x, world_y):
        """更新世界坐标"""
        self.world_x = world_x
        self.world_y = world_y

class PointEditorWidget(QWidget):
    """目标点编辑小部件"""
    point_updated = pyqtSignal(str, float, float, float)  # 名称, x, y, theta
    point_deleted = pyqtSignal(str)  # 名称
    point_renamed = pyqtSignal(str, str)  # 旧名称, 新名称
    
    def __init__(self, target_point, parent=None):
        super().__init__(parent)
        self.target_point = target_point
        
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
        self.init_ui()
        
        # 移动到目标点右下方
        self.move_to_target()
    
    def paintEvent(self, event):
        """绘制圆角矩形背景"""
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        
        # 绘制白色背景和蓝色边框
        rect = self.rect()
        painter.setBrush(QBrush(QColor(255, 255, 255)))
        painter.drawRoundedRect(rect.adjusted(1, 1, -1, -1), 8, 8)
        
        super().paintEvent(event)
    
    def init_ui(self):
        """初始化界面"""
        # 主布局
        main_layout = QVBoxLayout(self)
        main_layout.setContentsMargins(12, 12, 12, 12)
        main_layout.setSpacing(8)
        
        # 名称编辑
        name_layout = QHBoxLayout()
        name_label = QLabel("点位名:")
        name_label.setFixedWidth(40)
        
        name_label.setStyleSheet("""
            QLabel {
                background-color: transparent;
                border: none;
                padding: 0px;
                margin: 0px;
                color: #333333;
                font-size: 12px;
            }
        """)
        
        self.name_edit = QLineEdit(self.target_point.name)
        self.name_edit.setStyleSheet("""
            QLineEdit {
                border: 1px solid #d0d0d0;
                border-radius: 4px;
                padding: 4px 6px;
                background-color: #fafafa;
                color: #333333;
                font-size: 12px;
                min-height: 24px;
            }
        """)
        name_layout.addWidget(name_label)
        name_layout.addWidget(self.name_edit)
        
        # X坐标编辑
        x_layout = QHBoxLayout()
        x_label = QLabel("X坐标:")
        x_label.setFixedWidth(40)
        x_label.setStyleSheet("QLabel { background-color: transparent; border: none; padding: 0px; margin: 0px; color: #333333; font-size: 12px; }")
        
        self.x_spin = QDoubleSpinBox()
        self.x_spin.setRange(-1000, 1000)
        self.x_spin.setSingleStep(0.01)
        self.x_spin.setDecimals(3)
        self.x_spin.setValue(self.target_point.world_x)
        self.x_spin.setStyleSheet("""
            QDoubleSpinBox { border: 1px solid #d0d0d0; border-radius: 4px; padding: 4px 6px; background-color: #fafafa; color: #333333; font-size: 12px; min-height: 24px; }
            QDoubleSpinBox:focus { border: 2px solid #4a90e2; background-color: #ffffff; }
        """)
        x_layout.addWidget(x_label)
        x_layout.addWidget(self.x_spin)
        
        # Y坐标编辑
        y_layout = QHBoxLayout()
        y_label = QLabel("Y坐标:")
        y_label.setFixedWidth(40)
        y_label.setStyleSheet("QLabel { background-color: transparent; border: none; padding: 0px; margin: 0px; color: #333333; font-size: 12px; }")
        
        self.y_spin = QDoubleSpinBox()
        self.y_spin.setRange(-1000, 1000)
        self.y_spin.setSingleStep(0.01)
        self.y_spin.setDecimals(3)
        self.y_spin.setValue(self.target_point.world_y)
        self.y_spin.setStyleSheet("""
            QDoubleSpinBox { border: 1px solid #d0d0d0; border-radius: 4px; padding: 4px 6px; background-color: #fafafa; color: #333333; font-size: 12px; min-height: 24px; }
            QDoubleSpinBox:focus { border: 2px solid #4a90e2; background-color: #ffffff; }
        """)
        y_layout.addWidget(y_label)
        y_layout.addWidget(self.y_spin)
        
        # 方向(角度)编辑
        theta_layout = QHBoxLayout()
        theta_label = QLabel("方向(°):")
        theta_label.setFixedWidth(40)
        theta_label.setStyleSheet("QLabel { background-color: transparent; border: none; padding: 0px; margin: 0px; color: #333333; font-size: 12px; }")
        
        self.theta_spin = QDoubleSpinBox()
        self.theta_spin.setRange(-180, 180)
        self.theta_spin.setSingleStep(5)
        self.theta_spin.setDecimals(1)
        self.theta_spin.setValue(self.target_point.world_theta)
        self.theta_spin.setStyleSheet("""
            QDoubleSpinBox { border: 1px solid #d0d0d0; border-radius: 4px; padding: 4px 6px; background-color: #fafafa; color: #333333; font-size: 12px; min-height: 24px; }
            QDoubleSpinBox:focus { border: 2px solid #4a90e2; background-color: #ffffff; }
        """)
        theta_layout.addWidget(theta_label)
        theta_layout.addWidget(self.theta_spin)
        
        # 按钮布局
        button_layout = QHBoxLayout()
        button_layout.setSpacing(6)
        
        # 重命名按钮
        self.rename_btn = QPushButton("重命名")
        self.rename_btn.setStyleSheet("QPushButton { background-color: #4a90e2; color: white; border: none; border-radius: 4px; padding: 6px 12px; font-weight: bold; font-size: 12px; min-height: 28px; } QPushButton:hover { background-color: #357abd; }")
        
        # 删除按钮
        self.delete_btn = QPushButton("删除点位")
        self.delete_btn.setStyleSheet("QPushButton { background-color: #e74c3c; color: white; border: none; border-radius: 4px; padding: 6px 12px; font-weight: bold; font-size: 12px; min-height: 28px; } QPushButton:hover { background-color: #c0392b; }")
        
        # 关闭按钮
        self.close_btn = QPushButton("关闭")
        self.close_btn.setStyleSheet("QPushButton { background-color: #95a5a6; color: white; border: none; border-radius: 4px; padding: 6px 12px; font-weight: bold; font-size: 12px; min-height: 28px; } QPushButton:hover { background-color: #7f8c8d; }")
        
        button_layout.addWidget(self.rename_btn)
        button_layout.addWidget(self.delete_btn)
        button_layout.addWidget(self.close_btn)
        
        # 添加到主布局
        main_layout.addLayout(name_layout)
        main_layout.addLayout(x_layout)
        main_layout.addLayout(y_layout)
        main_layout.addLayout(theta_layout)
        main_layout.addSpacing(8)
        main_layout.addLayout(button_layout)
        
        # 连接信号
        self.rename_btn.clicked.connect(self.on_rename)
        self.delete_btn.clicked.connect(self.on_delete)
        self.close_btn.clicked.connect(self.close)
        
        # 连接坐标变化信号
        self.x_spin.valueChanged.connect(self.on_coordinate_changed)
        self.y_spin.valueChanged.connect(self.on_coordinate_changed)
        self.theta_spin.valueChanged.connect(self.on_coordinate_changed)
    
    def move_to_target(self):
        parent = self.parent()
        if parent:
            target_pos = QPoint(self.target_point.screen_x, self.target_point.screen_y)
            global_pos = parent.mapToGlobal(target_pos)
            self.move(global_pos.x() + 30, global_pos.y() + 30)
    
    def on_rename(self):
        new_name = self.name_edit.text().strip()
        if new_name and new_name != self.target_point.name:
            old_name = self.target_point.name
            self.target_point.name = new_name
            self.point_renamed.emit(old_name, new_name)
            self.name_edit.clearFocus()
    
    def on_delete(self):
        self.point_deleted.emit(self.target_point.name)
        self.close()
    
    def on_coordinate_changed(self):
        new_x = self.x_spin.value()
        new_y = self.y_spin.value()
        new_theta = self.theta_spin.value()
        
        self.target_point.update_world_pos(new_x, new_y)
        self.target_point.world_theta = new_theta
        
        self.point_updated.emit(self.target_point.name, new_x, new_y, new_theta)

class MapEditor(QObject):
    """地图编辑器"""
    point_deleted = pyqtSignal(str)
    point_renamed = pyqtSignal(str, str)
    point_added = pyqtSignal(str, float, float, float)  # 名称, x, y, theta
    points_updated = pyqtSignal()
    
    def __init__(self, map_canvas):
        super().__init__()
        self.map_canvas = map_canvas
        self.target_points = []
        self.editor_widgets = {}
        self.click_tolerance = 15
    
    def add_target_point(self, world_x, world_y, screen_x, screen_y, name=None, world_theta=0.0):
        point = TargetPoint(world_x, world_y, screen_x, screen_y, name, world_theta)
        self.target_points.append(point)
        self.point_added.emit(point.name, world_x, world_y, world_theta)
        
        if self.map_canvas.is_edit_mode:
            self.show_editor_for_point(point)
        self.map_canvas.update()
    
    def show_editor_for_point(self, point):
        if point.name in self.editor_widgets:
            self.editor_widgets[point.name].close()
        
        editor = PointEditorWidget(point, self.map_canvas)
        editor.point_updated.connect(self.on_point_updated)
        editor.point_deleted.connect(self.on_point_deleted)
        editor.point_renamed.connect(self.on_point_renamed)
        
        self.editor_widgets[point.name] = editor
        editor.show()
    
    def check_point_click(self, click_x, click_y):
        for point in self.target_points:
            distance = ((click_x - point.screen_x) ** 2 + (click_y - point.screen_y) ** 2) ** 0.5
            tolerance = self.click_tolerance * (20.0 / self.map_canvas.view_scale)
            if distance <= tolerance:
                self.show_editor_for_point(point)
                return True
        self.close_all_editors()
        return False
    
    def on_point_updated(self, point_name, x, y, theta):
        for point in self.target_points:
            if point.name == point_name:
                point.update_world_pos(x, y)
                point.world_theta = theta
                break
        self.points_updated.emit()
        self.map_canvas.update()
    
    def on_point_deleted(self, point_name):
        self.target_points = [p for p in self.target_points if p.name != point_name]
        if point_name in self.editor_widgets:
            self.editor_widgets[point_name].close()
            del self.editor_widgets[point_name]
        self.point_deleted.emit(point_name)
        self.points_updated.emit()
    
    def on_point_renamed(self, old_name, new_name):
        for point in self.target_points:
            if point.name == old_name:
                point.name = new_name
                break
        if old_name in self.editor_widgets:
            self.editor_widgets[new_name] = self.editor_widgets[old_name]
            del self.editor_widgets[old_name]
        self.point_renamed.emit(old_name, new_name)
        self.points_updated.emit()
    
    def close_all_editors(self):
        for editor in self.editor_widgets.values():
            editor.close()
        self.editor_widgets.clear()
    
    def clear_all_points(self):
        self.close_all_editors()
        self.target_points.clear()
        self.points_updated.emit()
    
    def get_all_points(self):
        points = []
        for point in self.target_points:
            points.append((point.name, point.world_x, point.world_y, point.world_theta))
        return points