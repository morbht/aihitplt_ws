#!/usr/bin/env python3

from PyQt5.QtWidgets import *
from PyQt5.QtCore import Qt, pyqtSignal, QTimer
import rospy
import math
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib


class InspectionTable(QWidget):
    inspection_started = pyqtSignal()
    inspection_stopped = pyqtSignal()
    inspection_paused = pyqtSignal()
    inspection_resumed = pyqtSignal()
    point_selected = pyqtSignal(str, float, float, float)
    navigation_started = pyqtSignal(str, float, float, float)
    point_reached_signal = pyqtSignal()
    navigation_failed_signal = pyqtSignal(str)
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.parent = parent
        self.available_numbers = list(range(1, 9))
        self.used_numbers = set()
        self.map_points = []
        self.current_running_index = -1
        self.is_inspecting = False
        self.is_paused = False
        self.loop_enabled = False
        self.navigation_in_progress = False
        self.paused_point_index = -1
        self.paused_point_name = ""
        self.paused_x = 0.0
        self.paused_y = 0.0
        self.paused_theta = 0.0
        self.rows = []
        self.point_reached_signal.connect(self._on_point_reached)
        self.navigation_failed_signal.connect(self._on_navigation_failed)
        self.move_base_client = None
        self._init_ros_client()
        self._init_ui()
        
    def _init_ros_client(self):
        try:
            self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
            if not self.move_base_client.wait_for_server(timeout=rospy.Duration(1.0)):
                self._log_message("警告: move_base服务器未连接")
        except Exception as e:
            self._log_message(f"导航客户端初始化失败: {str(e)}")
    
    def _log_message(self, message):
        if self.parent and hasattr(self.parent, 'right_panel'):
            self.parent.right_panel.log(message)
    
    def _init_ui(self):
        layout = QVBoxLayout(self)
        layout.setSpacing(0)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(self._create_table_header())
        self._create_scrollable_table()
        layout.addWidget(self.table_scroll)
    
    def _create_table_header(self):
        header_widget = QWidget()
        header_widget.setFixedHeight(24)
        header_layout = QHBoxLayout(header_widget)
        header_layout.setContentsMargins(0, 0, 0, 0)
        for header, width in zip(["点位名", "任务状态", "删除", "运行"], [42, 20, 12, 12]):
            label = QLabel(header)
            label.setAlignment(Qt.AlignCenter)
            label.setFixedHeight(20)
            label.setStyleSheet("font-weight: bold; color: #333; font-size: 10px;")
            header_layout.addWidget(label, width)
        return header_widget
    
    def _create_scrollable_table(self):
        self.table_scroll = QScrollArea()
        self.table_scroll.setWidgetResizable(True)
        self.table_scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.table_scroll.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOn)
        self.table_scroll.setStyleSheet("QScrollArea { border: none; } QScrollBar:vertical { background-color: #f0f0f0; width: 5px; border: none; } QScrollBar::handle:vertical { background-color: #d0d0d0; border: 1px solid #b0b0b0; min-height: 20px; }")
        self.points_table_widget = QWidget()
        self.points_table_layout = QVBoxLayout(self.points_table_widget)
        self.points_table_layout.setSpacing(0)
        self.points_table_layout.setContentsMargins(0, 0, 0, 0)
        self.table_scroll.setWidget(self.points_table_widget)
    
    def add_point_row(self, point_name=None, x=None, y=None, status="等待"):
        if not self.available_numbers:
            QMessageBox.warning(self, "警告", "最多只能添加8个巡检点")
            return False
        
        number = self.available_numbers.pop(0)
        self.used_numbers.add(number)
        
        row_widget = QWidget()
        row_widget.setObjectName(f"row_{number}")
        row_widget.setFixedHeight(22)
        row_widget.number = number
        row_widget.world_x = x if x is not None else 0.0
        row_widget.world_y = y if y is not None else 0.0
        row_widget.world_theta = 0.0
        
        row_layout = QHBoxLayout(row_widget)
        row_layout.setSpacing(0)
        row_layout.setContentsMargins(0, 0, 0, 0)
        
        name_combo = QComboBox()
        name_combo.setEditable(True)
        name_combo.addItem("")
        name_combo.setCurrentIndex(0)
        name_combo.setFixedHeight(20)
        name_combo.setStyleSheet("QComboBox { font-size: 10px; border: 1px solid #e0e0e0; border-radius: 3px; padding: 2px; } QComboBox:hover { border-color: #1976d2; }")
        self._update_combo_with_map_points(name_combo)
        
        if point_name:
            index = name_combo.findText(point_name)
            if index >= 0: name_combo.setCurrentIndex(index)
            else: name_combo.setEditText(point_name)
        
        name_combo.currentTextChanged.connect(lambda text, r=row_widget: self._on_point_name_changed(r, text))
        name_combo.currentIndexChanged.connect(lambda idx, r=row_widget: self._on_point_selected(r, idx))
        row_layout.addWidget(name_combo, 50)
        row_widget.name_combo = name_combo
        
        status_label = QLabel(status)
        status_label.setAlignment(Qt.AlignCenter)
        status_label.setFixedHeight(20)
        status_label.setStyleSheet(self._get_status_style(status))
        row_layout.addWidget(status_label, 25)
        row_widget.status_label = status_label
        
        delete_btn = QPushButton("删除")
        delete_btn.setFixedHeight(20)
        delete_btn.setStyleSheet("QPushButton { background-color: #f5f5f5; color: #333; border: 1px solid #e0e0e0; border-radius: 3px; padding: 2px 6px; font-size: 10px; } QPushButton:hover { background-color: #ffebee; border-color: #f44336; }")
        delete_btn.clicked.connect(lambda: self._remove_point_row(row_widget))
        row_layout.addWidget(delete_btn, 12)
        row_widget.delete_btn = delete_btn
        
        run_btn = QPushButton("运行")
        run_btn.setFixedHeight(20)
        run_btn.setStyleSheet("QPushButton { background-color: #f5f5f5; color: #333; border: 1px solid #e0e0e0; border-radius: 3px; padding: 2px 6px; font-size: 10px; } QPushButton:hover { background-color: #e8f5e8; border-color: #4caf50; }")
        run_btn.clicked.connect(lambda: self._run_single_point(row_widget))
        row_layout.addWidget(run_btn, 13)
        row_widget.run_btn = run_btn
        
        self.points_table_layout.addWidget(row_widget)
        self.rows.append(row_widget)
        return True
    
    def _update_combo_with_map_points(self, combo):
        current_text = combo.currentText()
        combo.clear()
        combo.addItem("", None)
        for point_data in self.map_points:
            if len(point_data) == 4:
                name, x, y, theta = point_data
            else:
                name, x, y = point_data
                theta = 0.0
            combo.addItem(name, (x, y, theta))
        
        if current_text:
            index = combo.findText(current_text)
            if index >= 0: combo.setCurrentIndex(index)
            else: combo.setEditText(current_text)
    
    def set_map_points(self, points):
        self.map_points = points
        for row in self.rows:
            if hasattr(row, 'name_combo'):
                current_text = row.name_combo.currentText()
                self._update_combo_with_map_points(row.name_combo)
                if current_text:
                    index = row.name_combo.findText(current_text)
                    if index >= 0:
                        row.name_combo.setCurrentIndex(index)
                    else:
                        row.name_combo.setCurrentIndex(0)
                        row.world_x, row.world_y, row.world_theta = 0, 0, 0.0
    
    def _on_point_name_changed(self, row_widget, text): pass
    
    def _on_point_selected(self, row_widget, index):
        combo = row_widget.name_combo
        if index > 0 and combo.itemData(index):
            data = combo.itemData(index)
            if len(data) == 3: x, y, theta = data
            else: x, y, theta = data[0], data[1], 0.0
            point_name = combo.currentText()
            row_widget.world_x, row_widget.world_y, row_widget.world_theta = x, y, theta
            self.point_selected.emit(point_name, x, y, theta)
    
    def _get_status_style(self, status):
        styles = {
            "运行中": "QLabel { background-color: #4caf50; color: white; border-radius: 3px; font-size: 10px; font-weight: bold; margin: 0px 2px; }",
            "等待": "QLabel { background-color: #ffeb3b; color: #333; border-radius: 3px; font-size: 10px; font-weight: bold; margin: 0px 2px; }",
            "完成": "QLabel { background-color: #9e9e9e; color: white; border-radius: 3px; font-size: 10px; font-weight: bold; margin: 0px 2px; }",
            "暂停": "QLabel { background-color: #ff9800; color: white; border-radius: 3px; font-size: 10px; font-weight: bold; margin: 0px 2px; }"
        }
        return styles.get(status, "QLabel { background-color: #f44336; color: white; border-radius: 3px; font-size: 10px; font-weight: bold; margin: 0px 2px; }")
    
    def _remove_point_row(self, row_widget):
        if self.is_inspecting and row_widget in self.rows:
            if self.rows.index(row_widget) == self.current_running_index:
                self.stop_inspection()
        self.points_table_layout.removeWidget(row_widget)
        if row_widget in self.rows: self.rows.remove(row_widget)
        number = getattr(row_widget, 'number', None)
        row_widget.deleteLater()
        if number:
            self.used_numbers.discard(number)
            self.available_numbers.append(number)
            self.available_numbers.sort()
    
    def _run_single_point(self, row_widget):
        point_name = row_widget.name_combo.currentText()
        if not point_name:
            QMessageBox.warning(self, "警告", "请先选择点位名")
            return
        
        x, y, theta = row_widget.world_x, row_widget.world_y, getattr(row_widget, 'world_theta', 0.0)
        
        if x == 0 and y == 0 and row_widget.name_combo.currentData():
            data = row_widget.name_combo.currentData()
            if len(data) == 3: x, y, theta = data
            else: x, y, theta = data[0], data[1], 0.0
            row_widget.world_x, row_widget.world_y, row_widget.world_theta = x, y, theta
        
        self._reset_all_status()
        row_widget.status_label.setText("运行中")
        row_widget.status_label.setStyleSheet(self._get_status_style("运行中"))
        
        self.navigation_started.emit(point_name, x, y, theta)
        self._send_navigation_goal(x, y, theta, point_name)
    
    def _send_navigation_goal(self, x, y, theta, point_name):
        try:
            if not self.move_base_client: self._init_ros_client()
            if self.navigation_in_progress and self.move_base_client and self.move_base_client.gh:
                self.move_base_client.cancel_all_goals()
                rospy.sleep(0.1)
            
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = "map"
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose.position.x = x
            goal.target_pose.pose.position.y = y
            goal.target_pose.pose.position.z = 0.0
            
            theta_rad = math.radians(theta)
            goal.target_pose.pose.orientation.x = 0.0
            goal.target_pose.pose.orientation.y = 0.0
            goal.target_pose.pose.orientation.z = math.sin(theta_rad / 2.0)
            goal.target_pose.pose.orientation.w = math.cos(theta_rad / 2.0)
            
            if self.move_base_client and self.move_base_client.wait_for_server(timeout=rospy.Duration(0.5)):
                self.navigation_in_progress = True
                self.move_base_client.send_goal(goal, done_cb=lambda status, result: self._on_navigation_done(point_name, status, result))
                self._log_message(f"导航目标已发送: {point_name} ({x:.2f}, {y:.2f}, {theta:.1f}°)")
            else:
                self._log_message("错误: move_base服务器未连接")
                self.navigation_in_progress = False
                self.navigation_failed_signal.emit(point_name)
        except Exception as e:
            self._log_message(f"导航失败: {str(e)}")
            self.navigation_in_progress = False
            self.navigation_failed_signal.emit(point_name)
    
    def _on_navigation_done(self, point_name, status, result):
        """导航完成回调（在ROS线程中执行）"""
        self.navigation_in_progress = False
        if status == actionlib.GoalStatus.SUCCEEDED:
            self._log_message(f"到达点位: {point_name}")
            if self.is_inspecting and not self.is_paused:
                # 通过信号触发，信号会在Qt主线程中处理
                self.point_reached_signal.emit()
        else:
            self._log_message(f"导航到 {point_name} 失败，状态码: {status}")
            if self.is_inspecting and not self.is_paused:
                self.navigation_failed_signal.emit(point_name)
    
    def _on_point_reached(self):
        """点位到达处理（在Qt主线程中执行）"""
        if not self.is_inspecting or self.is_paused: 
            return
        
        if 0 <= self.current_running_index < len(self.rows):
            self.rows[self.current_running_index].status_label.setText("完成")
            self.rows[self.current_running_index].status_label.setStyleSheet(self._get_status_style("完成"))
        
        self.current_running_index += 1
        
        if self.current_running_index >= len(self.rows):
            if self.loop_enabled:
                self.current_running_index = 0
                self._log_message("循环巡检: 开始下一轮")
                # 延迟3秒后开始下一轮
                QTimer.singleShot(3000, self._run_next_point)
            else:
                self.stop_inspection()
                self._log_message("巡检任务完成")
        else:
            # 延迟3秒后前往下一个点
            QTimer.singleShot(3000, self._run_next_point)
    
    def _on_navigation_failed(self, point_name):
        if self.is_inspecting and not self.is_paused:
            if 0 <= self.current_running_index < len(self.rows):
                self.rows[self.current_running_index].status_label.setText("失败")
                self.rows[self.current_running_index].status_label.setStyleSheet(self._get_status_style("失败"))
            reply = QMessageBox.question(self, "导航失败", f"导航到 {point_name} 失败，是否继续下一个点？", QMessageBox.Yes | QMessageBox.No)
            if reply == QMessageBox.Yes:
                self.current_running_index += 1
                QTimer.singleShot(100, self._run_next_point)
            else:
                self.stop_inspection()
    
    def _reset_all_status(self):
        for row in self.rows:
            if hasattr(row, 'status_label'):
                row.status_label.setText("等待")
                row.status_label.setStyleSheet(self._get_status_style("等待"))
    
    def get_point_count(self): return len(self.rows)
    
    def clear_points(self):
        while self.rows:
            row = self.rows.pop(0)
            self.points_table_layout.removeWidget(row)
            if row.number in self.used_numbers:
                self.used_numbers.discard(row.number)
                self.available_numbers.append(row.number)
            row.deleteLater()
        self.available_numbers.sort()
        self.current_running_index, self.is_inspecting, self.is_paused = -1, False, False
    
    def start_inspection(self, loop_enabled=False):
        if len(self.rows) == 0:
            QMessageBox.warning(self, "警告", "请先添加巡检点位")
            return False
        for i, row in enumerate(self.rows):
            if not row.name_combo.currentText():
                QMessageBox.warning(self, "警告", f"第{i+1}个点位名称为空，请先设置点位名")
                return False
            if row.world_x == 0 and row.world_y == 0 and row.name_combo.currentData():
                data = row.name_combo.currentData()
                if len(data) == 3: row.world_x, row.world_y, row.world_theta = data
                else: row.world_x, row.world_y, row.world_theta = data[0], data[1], 0.0
        self.is_inspecting, self.is_paused, self.loop_enabled, self.current_running_index, self.navigation_in_progress = True, False, loop_enabled, 0, False
        self._reset_all_status()
        self.inspection_started.emit()
        self._run_next_point()
        return True
    
    def pause_inspection(self):
        if not self.is_inspecting or self.is_paused: return False
        if 0 <= self.current_running_index < len(self.rows):
            current_row = self.rows[self.current_running_index]
            self.paused_point_index = self.current_running_index
            self.paused_point_name = current_row.name_combo.currentText()
            self.paused_x, self.paused_y = current_row.world_x, current_row.world_y
            self.paused_theta = getattr(current_row, 'world_theta', 0.0)
            current_row.status_label.setText("暂停")
            current_row.status_label.setStyleSheet(self._get_status_style("暂停"))
        if self.move_base_client and self.move_base_client.gh: self.move_base_client.cancel_all_goals()
        self.navigation_in_progress, self.is_paused = False, True
        self._log_message(f"巡检已暂停，当前点位: {self.paused_point_name}")
        self.inspection_paused.emit()
        return True
    
    def resume_inspection(self):
        if not self.is_paused or self.paused_point_index < 0: return False
        self.is_inspecting, self.is_paused, self.current_running_index = True, False, self.paused_point_index
        QTimer.singleShot(500, self._execute_resume)
        self._log_message(f"恢复巡检，继续前往: {self.paused_point_name}")
        self.inspection_resumed.emit()
        return True
    
    def _execute_resume(self):
        if self.is_paused or self.current_running_index < 0: return
        if self.current_running_index >= len(self.rows):
            self.stop_inspection()
            return
        current_row = self.rows[self.current_running_index]
        point_name, x, y, theta = current_row.name_combo.currentText(), current_row.world_x, current_row.world_y, getattr(current_row, 'world_theta', 0.0)
        self._reset_all_status()
        current_row.status_label.setText("运行中")
        current_row.status_label.setStyleSheet(self._get_status_style("运行中"))
        self._send_navigation_goal(x, y, theta, point_name)
    
    def stop_inspection(self):
        self.is_inspecting, self.is_paused, self.current_running_index, self.paused_point_index = False, False, -1, -1
        if self.move_base_client and self.move_base_client.gh: self.move_base_client.cancel_all_goals()
        self.navigation_in_progress = False
        self._reset_all_status()
        self.inspection_stopped.emit()
        self._log_message("巡检已停止")
    
    def _run_next_point(self):
        if not self.is_inspecting or self.is_paused: return
        if len(self.rows) == 0:
            self.stop_inspection()
            return
        if self.current_running_index >= len(self.rows):
            if self.loop_enabled:
                self.current_running_index = 0
                self._log_message("循环巡检: 重新开始")
            else:
                self.stop_inspection()
                self._log_message("巡检任务完成")
                return
        if self.navigation_in_progress:
            QTimer.singleShot(100, self._run_next_point)
            return
        current_row = self.rows[self.current_running_index]
        point_name, x, y, theta = current_row.name_combo.currentText(), current_row.world_x, current_row.world_y, getattr(current_row, 'world_theta', 0.0)
        self._reset_all_status()
        current_row.status_label.setText("运行中")
        current_row.status_label.setStyleSheet(self._get_status_style("运行中"))
        self._log_message(f"巡检中 ({self.current_running_index + 1}/{len(self.rows)}): 前往 {point_name}")
        self._send_navigation_goal(x, y, theta, point_name)
    
    def get_all_points(self):
        points = []
        for row in self.rows:
            point_name = row.name_combo.currentText()
            if point_name:
                points.append({
                    'name': point_name,
                    'x': row.world_x,
                    'y': row.world_y,
                    'theta': getattr(row, 'world_theta', 0.0),
                    'status': row.status_label.text()
                })
        return points