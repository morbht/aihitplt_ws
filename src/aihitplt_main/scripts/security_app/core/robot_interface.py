#!/usr/bin/env python3

import os
import sys
import rospy
from PyQt5.QtWidgets import *
from PyQt5.QtCore import Qt, QTimer
from pathlib import Path
from PyQt5.QtWidgets import QFileDialog, QMessageBox
from core.map_canvas import MapCanvas
from ui.left_panel import LeftPanel
from ui.right_panel import RightPanel


class RobotInterface(QMainWindow):
    """ROS机器人巡检控制主界面"""
    
    def __init__(self):
        super().__init__()
        self.setWindowTitle("ROS机器人巡检控制界面")
        self.setGeometry(100, 100, 800, 600)
        
        # 加载样式表
        self.load_stylesheet()
        
        # 初始化组件
        self.init_components()
        
        # 初始化界面
        self.init_ui()
        
        # 巡检状态
        self.is_inspecting = False
        
    def load_stylesheet(self):
        """加载样式表"""
        try:
            from ui.styles import get_main_stylesheet
            self.setStyleSheet(get_main_stylesheet())
        except:
            pass
    
    def init_components(self):
        """初始化组件"""
        # 地图画布
        self.map_canvas = MapCanvas()

        # 左右面板
        self.left_panel = LeftPanel(self)
        self.right_panel = RightPanel(self)

        # 连接地图信号到坐标显示
        if hasattr(self.map_canvas, 'robot_pose_updated'):
            self.map_canvas.robot_pose_updated.connect(self.update_position_display)
        
        # 连接地图画布的信号
        if hasattr(self.map_canvas, 'editor'):
            self.map_canvas.editor.point_added.connect(self.on_map_point_added)
            self.map_canvas.editor.point_deleted.connect(self.on_map_point_deleted)
            self.map_canvas.editor.point_renamed.connect(self.on_map_point_renamed)
            self.map_canvas.editor.points_updated.connect(self.on_map_points_updated)

    def init_ui(self):
        """初始化界面"""
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        main_layout = QHBoxLayout(central_widget)
        main_layout.setContentsMargins(5, 5, 5, 5)
        main_layout.setSpacing(5)
        
        main_layout.addWidget(self.left_panel, 60)
        main_layout.addWidget(self.right_panel, 40)
        
        # 连接信号
        self.connect_signals()
        
    def connect_signals(self):
        """连接信号和槽"""
        if hasattr(self.left_panel, 'start_btn'):
            self.left_panel.start_btn.clicked.connect(self.toggle_inspection)
    
        if hasattr(self.left_panel, 'add_point_btn'):
            self.left_panel.add_point_btn.clicked.connect(self.add_inspection_point)
        
        # 连接地图点击信号
        if hasattr(self.map_canvas, 'map_position_clicked'):
            self.map_canvas.map_position_clicked.connect(self.update_map_position_display)
        
        # 连接地图移动信号
        if hasattr(self.map_canvas, 'map_position_moved'):
            self.map_canvas.map_position_moved.connect(self.update_map_position_mouse_move)
        
        # 连接巡检表格信号
        if hasattr(self.left_panel, 'inspection_table'):
            inspection_table = self.left_panel.inspection_table
            inspection_table.inspection_started.connect(self.on_inspection_started)
            inspection_table.inspection_stopped.connect(self.on_inspection_stopped)
            inspection_table.inspection_paused.connect(self.on_inspection_paused)
            inspection_table.inspection_resumed.connect(self.on_inspection_resumed)
            inspection_table.point_selected.connect(self.on_point_selected_from_table)
            inspection_table.navigation_started.connect(self.on_navigation_started)
        
        if hasattr(self.right_panel, 'emergency_handler'):
            self.right_panel.emergency_handler.emergency_cleared.connect(self.on_emergency_cleared)
    
    def on_emergency_cleared(self):
        """异常清除后的处理"""
        if self.right_panel:
            self.right_panel.log("收到异常解除信号")
    
    def on_inspection_paused(self):
        """巡检暂停的处理"""
        if self.right_panel:
            self.right_panel.log("巡检已暂停")
    
    def on_inspection_resumed(self):
        """巡检恢复的处理"""
        if self.right_panel:
            self.right_panel.log("巡检已恢复")
    
    def update_position_display(self, robot_x, robot_y, robot_theta):
        """更新坐标显示"""
        if hasattr(self.left_panel, 'coord_labels'):
            self.left_panel.coord_labels["label_robot_pos"].setText(
                f"({robot_x:.2f},{robot_y:.2f},{robot_theta:.2f})")
    
    def update_map_position_display(self, map_x, map_y):
        """更新地图点击位置显示"""
        if hasattr(self.left_panel, 'coord_labels'):
            self.left_panel.coord_labels["label_map_pos"].setText(
                f"({map_x:.2f},{map_y:.2f})")
    
    def update_map_position_mouse_move(self, map_x, map_y):
        """更新鼠标移动时的地图位置显示"""
        if hasattr(self.left_panel, 'coord_labels'):
            self.left_panel.coord_labels["label_map_pos"].setText(
                f"({map_x:.2f},{map_y:.2f})")
    
    def on_inspection_started(self):
        """巡检开始的处理"""
        self.is_inspecting = True
        if hasattr(self.left_panel, 'start_btn'):
            self.left_panel.start_btn.setText("暂停巡检")
            self.left_panel.start_btn.setStyleSheet("""
                QPushButton {
                    background-color: #f44336;
                    color: white;
                    font-weight: bold;
                    font-size: 12px;
                }
            """)
        
        if self.right_panel:
            self.right_panel.log("开始巡检任务")

    def on_inspection_stopped(self):
        """巡检停止的处理"""
        self.is_inspecting = False
        if hasattr(self.left_panel, 'start_btn'):
            self.left_panel.start_btn.setText("开始巡检")
            self.left_panel.start_btn.setStyleSheet("""
                QPushButton {
                    background-color: #1976d2;
                    color: white;
                    font-weight: bold;
                    font-size: 12px;
                }
            """)
        
        if self.right_panel:
            self.right_panel.log("巡检已停止")

    def on_point_selected_from_table(self, point_name, x, y, theta):
        """从表格选择点位的处理"""
        if self.right_panel:
            self.right_panel.log(f"选择点位: {point_name} ({x:.2f}, {y:.2f}, {theta:.1f}°)")

    def on_navigation_started(self, point_name, x, y, theta):
        """导航开始的处理"""
        pass

    def toggle_inspection(self):
        """切换巡检状态"""
        if not self.is_inspecting:
            self.start_inspection()
        else:
            self.stop_inspection()
            
    def start_inspection(self):
        """开始巡检"""
        if hasattr(self.left_panel, 'inspection_table'):
            loop_enabled = hasattr(self.left_panel, 'loop_check') and self.left_panel.loop_check.isChecked()
            self.left_panel.inspection_table.start_inspection(loop_enabled)
            
    def stop_inspection(self):
        """停止巡检"""
        if hasattr(self.left_panel, 'inspection_table'):
            self.left_panel.inspection_table.stop_inspection()
    
    def add_inspection_point(self):
        """添加巡检点"""
        if hasattr(self.left_panel, 'inspection_table'):
            success = self.left_panel.inspection_table.add_point_row()
            if success and self.right_panel:
                count = self.left_panel.inspection_table.get_point_count()
                self.right_panel.log(f"添加巡检点: 点位{count}")
                
    def on_map_point_deleted(self, point_name):
        """处理地图点位删除"""
        if hasattr(self.map_canvas, 'editor'):
            map_points = self.map_canvas.editor.get_all_points()
            if hasattr(self.left_panel, 'inspection_table'):
                self.left_panel.inspection_table.set_map_points(map_points)
                if self.right_panel:
                    self.right_panel.log(f"地图点位已删除: {point_name}")

    def on_map_point_renamed(self, old_name, new_name):
        """处理地图点位重命名"""
        if hasattr(self.map_canvas, 'editor'):
            map_points = self.map_canvas.editor.get_all_points()
            if hasattr(self.left_panel, 'inspection_table'):
                self.left_panel.inspection_table.set_map_points(map_points)
                if self.right_panel:
                    self.right_panel.log(f"地图点位已重命名: {old_name} -> {new_name}")

    def on_map_points_updated(self):
        """处理地图点位列表更新"""
        if hasattr(self.map_canvas, 'editor'):
            map_points = self.map_canvas.editor.get_all_points()
            if hasattr(self.left_panel, 'inspection_table'):
                self.left_panel.inspection_table.set_map_points(map_points)
                
    def on_map_point_added(self, point_name, x, y, theta):
        """处理地图上添加的点位"""
        if hasattr(self.map_canvas, 'editor'):
            map_points = self.map_canvas.editor.get_all_points()
            if hasattr(self.left_panel, 'inspection_table'):
                self.left_panel.inspection_table.set_map_points(map_points)
                if self.right_panel:
                    self.right_panel.log(f"地图点位已更新: {point_name} ({x:.2f}, {y:.2f}, {theta:.1f}°)")

    def load_map_yaml(self, yaml_path):
        """加载 YAML 地图文件"""
        try:
            import yaml
            
            with open(yaml_path, 'r', encoding='utf-8') as f:
                map_data = yaml.safe_load(f)
            
            # 获取地图文件名
            map_name = os.path.basename(yaml_path).rsplit('.', 1)[0]
            map_dir = os.path.dirname(yaml_path)
            
            # 查找对应的导航点文件
            nav_points_files = [
                f"{map_dir}/{map_name}_points.json",
                f"{map_dir}/navigation_points.json",
                f"{map_dir}/points.json"
            ]
            
            for nav_file in nav_points_files:
                if os.path.exists(nav_file):
                    self.load_navigation_points(nav_file)
                    break
            
            if self.right_panel:
                self.right_panel.log(f"成功打开地图文件: {yaml_path}")
            
        except Exception as e:
            if self.right_panel:
                self.right_panel.log(f"警告: 无法完全加载地图 - {str(e)}")

    def load_navigation_points(self, json_path):
        """加载导航点文件"""
        try:
            import json
            
            with open(json_path, 'r', encoding='utf-8') as f:
                points_data = json.load(f)
            
            if 'points' in points_data and points_data['points']:
                if hasattr(self.map_canvas, 'editor'):
                    self.map_canvas.editor.clear_all_points()
                
                map_points = []
                
                for point_info in points_data['points']:
                    if all(key in point_info for key in ['name', 'x', 'y']):
                        self.map_canvas.editor.add_target_point(
                            point_info['x'],
                            point_info['y'],
                            0, 0,
                            point_info['name'],
                            point_info.get('theta', 0.0)
                        )
                        map_points.append((point_info['name'], point_info['x'], point_info['y'], point_info.get('theta', 0.0)))
                
                if hasattr(self.left_panel, 'inspection_table'):
                    self.left_panel.inspection_table.set_map_points(map_points)
                
                if self.right_panel:
                    count = len(points_data['points'])
                    self.right_panel.log(f"成功加载 {count} 个导航点: {json_path}")
            
        except Exception as e:
            if self.right_panel:
                self.right_panel.log(f"警告: 无法加载导航点文件 - {str(e)}")

    def save_map_files(self, yaml_path):
        """保存地图相关文件"""
        try:
            import yaml
            import json
            from pathlib import Path
            
            yaml_path = Path(yaml_path)
            base_path = yaml_path.parent
            map_name = yaml_path.stem
            
            base_path.mkdir(parents=True, exist_ok=True)
            
            pgm_path = base_path / f'{map_name}.pgm'
            if hasattr(self.map_canvas, 'save_map_image'):
                if not self.map_canvas.save_map_image(str(pgm_path)):
                    QMessageBox.warning(self, "警告", "无法保存 PGM 地图图像")
                    if self.right_panel:
                        self.right_panel.log("警告: 无法保存 PGM 地图图像")
            else:
                with open(pgm_path, 'w') as f:
                    f.write("# 地图图像文件\n")
                if self.right_panel:
                    self.right_panel.log("警告: 使用空的 PGM 文件占位")
            
            yaml_info = {
                'image': f'{map_name}.pgm',
                'resolution': 0.05,
                'origin': [0.0, 0.0, 0.0],
                'negate': 0,
                'occupied_thresh': 0.65,
                'free_thresh': 0.196
            }
            
            if hasattr(self.map_canvas, 'get_map_info'):
                map_info = self.map_canvas.get_map_info()
                if map_info:
                    yaml_info['resolution'] = map_info['resolution']
                    yaml_info['origin'] = [
                        map_info['origin_x'],
                        map_info['origin_y'],
                        0.0
                    ]
            
            with open(yaml_path, 'w', encoding='utf-8') as f:
                yaml.dump(yaml_info, f, default_flow_style=False)
            
            nav_points_file = base_path / f'{map_name}_points.json'
            self.save_navigation_points(str(nav_points_file))
            
            if self.right_panel:
                self.right_panel.log(f"地图 YAML 文件已保存: {yaml_path}")
                self.right_panel.log(f"地图 PGM 文件已保存: {pgm_path}")
                self.right_panel.log(f"导航点文件已保存: {nav_points_file}")
            
            success_msg = (
                f"地图文件已成功保存:\n\n"
                f"• 地图配置: {yaml_path.name}\n"
                f"• 地图图像: {pgm_path.name}\n"
                f"• 导航点: {nav_points_file.name}\n\n"
                f"保存位置: {base_path}"
            )
            
            QMessageBox.information(self, "保存成功", success_msg)
            
        except Exception as e:
            error_msg = f"保存地图文件失败: {str(e)}"
            QMessageBox.critical(self, "错误", error_msg)
            if self.right_panel:
                self.right_panel.log(f"错误: {error_msg}")

    def save_navigation_points(self, json_path):
        """保存导航点到 JSON 文件"""
        try:
            import json
            from pathlib import Path
            
            points_data = []
            if hasattr(self.left_panel, 'inspection_table'):
                points_data = self.left_panel.inspection_table.get_all_points()
            
            map_points = []
            if hasattr(self.map_canvas, 'editor'):
                for point in self.map_canvas.editor.target_points:
                    point_info = {
                        'name': point.name,
                        'x': float(point.world_x),
                        'y': float(point.world_y),
                        'theta': float(point.world_theta),
                        'type': 'NavGoal'
                    }
                    map_points.append(point_info)
            
            point_dict = {}
            for p in map_points:
                point_dict[p['name']] = p
            for p in points_data:
                if p['name'] in point_dict:
                    point_dict[p['name']]['x'] = p['x']
                    point_dict[p['name']]['y'] = p['y']
                    point_dict[p['name']]['theta'] = p.get('theta', 0.0)
                else:
                    point_dict[p['name']] = {
                        'name': p['name'],
                        'x': p['x'],
                        'y': p['y'],
                        'theta': p.get('theta', 0.0),
                        'type': 'NavGoal'
                    }
            
            navigation_data = {
                'map_name': Path(json_path).stem.replace('_points', ''),
                'map_property': {
                    'support_controllers': ['FollowPath', 'Backward', 'MPPI']
                },
                'points': list(point_dict.values())
            }
            
            with open(json_path, 'w', encoding='utf-8') as f:
                json.dump(navigation_data, f, indent=2, ensure_ascii=False)
            
            if self.right_panel:
                count = len(navigation_data['points'])
                self.right_panel.log(f"保存了 {count} 个导航点到: {json_path}")
            
        except Exception as e:
            raise Exception(f"保存导航点失败: {str(e)}")

    def relocate(self):
        if self.right_panel: self.right_panel.log("执行机器人重定位")
            
    def edit_map(self):
        if self.right_panel: self.right_panel.log("进入地图编辑模式")
            
    def open_map(self):
        try:
            file_path, _ = QFileDialog.getOpenFileName(self, "打开地图文件", "", "地图文件 (*.yaml *.yml);;所有文件 (*.*)")
            if not file_path: return
            if file_path.lower().endswith(('.yaml', '.yml')): self.load_map_yaml(file_path)
            elif file_path.lower().endswith('.json'): self.load_navigation_points(file_path)
            else: QMessageBox.warning(self, "警告", "不支持的文件格式")
        except Exception as e:
            QMessageBox.critical(self, "错误", f"打开地图失败: {str(e)}")
            if self.right_panel: self.right_panel.log(f"错误: 打开地图失败 - {str(e)}")
            
    def save_map(self):
        try:
            file_path, _ = QFileDialog.getSaveFileName(self, "保存地图", "", "地图文件 (*.yaml *.yml);;所有文件 (*.*)")
            if not file_path: return
            if not file_path.lower().endswith(('.yaml', '.yml')): file_path += '.yaml'
            self.save_map_files(file_path)
        except Exception as e:
            QMessageBox.critical(self, "错误", f"保存地图失败: {str(e)}")
            if self.right_panel: self.right_panel.log(f"错误: 保存地图失败 - {str(e)}")
            
    def save_as(self):
        if self.right_panel: self.right_panel.log("地图另存为")
            
    def load_points(self):
        try:
            file_path, _ = QFileDialog.getOpenFileName(self, "加载点位文件", "", "点位文件 (*.json);;所有文件 (*.*)")
            if file_path: self.load_navigation_points(file_path)
        except Exception as e:
            if self.right_panel: self.right_panel.log(f"错误: 加载点位失败 - {str(e)}")
            
    def save_points(self):
        try:
            file_path, _ = QFileDialog.getSaveFileName(self, "保存点位文件", "", "点位文件 (*.json);;所有文件 (*.*)")
            if file_path:
                if not file_path.lower().endswith('.json'): file_path += '.json'
                self.save_navigation_points(file_path)
        except Exception as e:
            if self.right_panel: self.right_panel.log(f"错误: 保存点位失败 - {str(e)}")