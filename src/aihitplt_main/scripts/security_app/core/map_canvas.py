#!/usr/bin/env python3

import math
import rospy
import numpy as np
from PyQt5.QtWidgets import *
from PyQt5.QtCore import Qt, QTimer, QPointF, pyqtSignal, QPoint
from PyQt5.QtGui import QPainter, QColor, QPen, QBrush, QPixmap, QImage, QCursor

# TF相关导入
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PointStamped

from ui.map_editor import MapEditor
from ui.relocate_dialog import RelocateDialog

class MapCanvas(QWidget):
    robot_pose_updated = pyqtSignal(float, float, float)
    map_position_clicked = pyqtSignal(float, float)
    map_position_moved = pyqtSignal(float, float)
    edit_mode_changed = pyqtSignal(bool)
    safe_update_signal = pyqtSignal()
    
    def __init__(self):
        super().__init__()
        
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0
        
        self.original_robot_x = 0.0
        self.original_robot_y = 0.0
        self.original_robot_theta = 0.0
        
        self.laser_data = []
        self.laser_points_map = []
        self.nav_path = []
        
        self.map_data = None
        self.costmap_data = None
        self.map_origin_x = 0.0
        self.map_origin_y = 0.0
        self.map_resolution = 0.05
        self.map_width = 0
        self.map_height = 0
        
        self.view_offset_x = 0.0
        self.view_offset_y = 0.0
        self.view_scale = 20.0
        self.is_dragging = False
        self.drag_start_x = 0
        self.drag_start_y = 0
        self.map_rotation = 0.0
        
        self.robot_pixmap = self.load_robot_icon()
        
        self.is_edit_mode = False
        self.edit_btn = None
        self.editor = MapEditor(self)
        self.editor.point_deleted.connect(self.on_point_deleted)
        self.editor.point_renamed.connect(self.on_point_renamed)
        self.editor.point_added.connect(self.on_point_added)
        self.editor.points_updated.connect(self.on_points_updated)
        
        self.target_icon = self.load_target_icon()
        self.cursor_icon = self.load_cursor_icon()
        
        self.relocate_dialog = None
        self.is_relocating = False
        self.initialpose_pub = None
        
        self.tf_buffer = None
        self.tf_listener = None
        self.tf_available = False
        
        self.safe_update_signal.connect(self.update)
        
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self.update)
        self.update_timer.start(100)
        
        QTimer.singleShot(500, self.delayed_init_ros)
        
        self.setMinimumHeight(300)
        self.setMouseTracking(True)
    
    def load_target_icon(self):
        pixmap = QPixmap('/home/aihit/aihitplt_ws/src/aihitplt_main/scripts/security_app/resources/icon/target.svg')
        if pixmap.isNull():
            pixmap = QPixmap(32, 32)
            pixmap.fill(Qt.transparent)
            painter = QPainter(pixmap)
            painter.setRenderHint(QPainter.Antialiasing)
            painter.setBrush(QBrush(QColor(255, 0, 0, 180)))
            painter.setPen(QPen(Qt.red, 2))
            painter.drawEllipse(8, 8, 16, 16)
            painter.drawLine(16, 16, 24, 16)
            painter.end()
        return pixmap
    
    def load_cursor_icon(self):
        pixmap = QPixmap('/home/aihit/aihitplt_ws/src/aihitplt_main/scripts/security_app/resources/icon/add_32.svg')
        if pixmap.isNull():
            pixmap = QPixmap(32, 32)
            pixmap.fill(Qt.transparent)
            painter = QPainter(pixmap)
            painter.setRenderHint(QPainter.Antialiasing)
            painter.setBrush(QBrush(QColor(0, 150, 0, 180)))
            painter.setPen(QPen(Qt.green, 2))
            painter.drawEllipse(0, 0, 32, 32)
            painter.drawLine(16, 16, 30, 16)
            painter.end()
        return pixmap
    
    def delayed_init_ros(self):
        try:
            self.init_ros_subscribers()
            self.init_tf()
        except Exception as e:
            QTimer.singleShot(5000, self.delayed_init_ros)
    
    def load_robot_icon(self):
        pixmap = QPixmap('/home/aihit/aihitplt_ws/src/aihitplt_main/scripts/security_app/resources/icon/robot.svg')
        if pixmap.isNull():
            pixmap = QPixmap(40, 40)
            pixmap.fill(Qt.transparent)
            painter = QPainter(pixmap)
            painter.setBrush(QBrush(QColor(255, 0, 0)))
            painter.drawEllipse(0, 0, 40, 40)
            painter.setPen(QPen(Qt.yellow, 3))
            painter.drawLine(20, 20, 35, 20)
            painter.end()
        return pixmap
    
    def init_tf(self):
        try:
            self.tf_buffer = tf2_ros.Buffer()
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
            self.tf_available = True
        except Exception as e:
            self.tf_available = False
            QTimer.singleShot(5000, self.init_tf)
    
    def init_ros_subscribers(self):
        try:
            from nav_msgs.msg import Path, OccupancyGrid
            from sensor_msgs.msg import LaserScan
            from geometry_msgs.msg import PoseWithCovarianceStamped
            
            rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.amcl_pose_callback, queue_size=10)
            rospy.Subscriber('/scan', LaserScan, self.laser_callback, queue_size=10)
            rospy.Subscriber('/move_base/NavfnROS/plan', Path, self.global_path_callback, queue_size=10)
            rospy.Subscriber('/move_base/global_costmap/costmap', OccupancyGrid, self.global_costmap_callback, queue_size=10)
            rospy.Subscriber('/map', OccupancyGrid, self.map_callback, queue_size=10)
        except Exception as e:
            pass
    
    def amcl_pose_callback(self, msg):
        try:
            if not self.is_relocating:
                self.robot_x = msg.pose.pose.position.x
                self.robot_y = msg.pose.pose.position.y
                orientation = msg.pose.pose.orientation
                self.robot_theta = math.atan2(2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
                                            1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z))
            self.robot_pose_updated.emit(self.robot_x, self.robot_y, self.robot_theta)
        except Exception as e:
            pass
    
    def laser_callback(self, msg):
        self.laser_data = []
        angle = msg.angle_min
        for range_val in msg.ranges:
            if not math.isinf(range_val) and not math.isnan(range_val):
                if msg.range_min <= range_val <= msg.range_max:
                    x = range_val * math.cos(angle)
                    y = range_val * math.sin(angle)
                    self.laser_data.append((x, y))
            angle += msg.angle_increment
        
        if self.tf_available and self.tf_buffer:
            try:
                transform = self.tf_buffer.lookup_transform('map', msg.header.frame_id, rospy.Time(0), rospy.Duration(0.1))
                self.laser_points_map = []
                angle = msg.angle_min
                for range_val in msg.ranges:
                    if not math.isinf(range_val) and not math.isnan(range_val):
                        if msg.range_min <= range_val <= msg.range_max:
                            laser_point = PointStamped()
                            laser_point.header.frame_id = msg.header.frame_id
                            laser_point.header.stamp = msg.header.stamp
                            laser_point.point.x = range_val * math.cos(angle)
                            laser_point.point.y = range_val * math.sin(angle)
                            laser_point.point.z = 0.0
                            try:
                                map_point = tf2_geometry_msgs.do_transform_point(laser_point, transform)
                                self.laser_points_map.append((map_point.point.x, map_point.point.y))
                            except:
                                pass
                    angle += msg.angle_increment
            except:
                pass
    
    def global_path_callback(self, msg):
        self.nav_path = []
        for pose in msg.poses:
            self.nav_path.append((pose.pose.position.x, pose.pose.position.y))
    
    def global_costmap_callback(self, msg):
        try:
            width = msg.info.width
            height = msg.info.height
            costmap_array = np.array(msg.data, dtype=np.int8).reshape((height, width))
            self.costmap_data = np.zeros((height, width, 4), dtype=np.uint8)
            for y in range(height):
                for x in range(width):
                    value = costmap_array[y, x]
                    if value > 0:
                        self.costmap_data[y, x] = [0, 0, min(200, value * 2), min(100, value * 0.5)]
        except:
            pass
    
    def map_callback(self, msg):
        try:
            self.map_resolution = msg.info.resolution
            self.map_width = msg.info.width
            self.map_height = msg.info.height
            self.map_origin_x = msg.info.origin.position.x
            self.map_origin_y = msg.info.origin.position.y
            
            map_array = np.array(msg.data, dtype=np.int8).reshape((self.map_height, self.map_width))
            self.map_data = np.zeros((self.map_height, self.map_width, 3), dtype=np.uint8)
            
            for y in range(self.map_height):
                for x in range(self.map_width):
                    value = map_array[y, x]
                    if value == -1: self.map_data[y, x] = [128, 128, 128]
                    elif value == 0: self.map_data[y, x] = [255, 255, 255]
                    elif value == 100: self.map_data[y, x] = [0, 0, 0]
                    elif 0 < value < 100:
                        gray = int(255 - value * 2.55)
                        self.map_data[y, x] = [gray, gray, gray]
            
            self.safe_update_signal.emit()
        except:
            pass
    
    def world_to_screen(self, world_x, world_y):
        screen_x = (world_x - self.view_offset_x) * self.view_scale + self.width() / 2
        screen_y = (-world_y - self.view_offset_y) * self.view_scale + self.height() / 2
        return screen_x, screen_y
    
    def screen_to_world(self, screen_x, screen_y):
        world_x = (screen_x - self.width() / 2) / self.view_scale + self.view_offset_x
        world_y = -((screen_y - self.height() / 2) / self.view_scale + self.view_offset_y)
        return world_x, world_y
    
    def wheelEvent(self, event):
        mouse_world_x, mouse_world_y = self.screen_to_world(event.x(), event.y())
        delta = event.angleDelta().y() / 1200.0
        self.view_scale *= (1.0 + delta)
        self.view_scale = max(1.0, min(100.0, self.view_scale))
        mouse_screen_x, mouse_screen_y = self.world_to_screen(mouse_world_x, mouse_world_y)
        self.view_offset_x -= (event.x() - mouse_screen_x) / self.view_scale
        self.view_offset_y -= (event.y() - mouse_screen_y) / self.view_scale
        self.update()
    
    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            self.is_dragging = True
            self.drag_start_x = event.x()
            self.drag_start_y = event.y()
            world_x, world_y = self.screen_to_world(event.x(), event.y())
            self.map_position_clicked.emit(world_x, world_y)
            if self.is_edit_mode:
                self.editor.check_point_click(event.x(), event.y())
        elif event.button() == Qt.RightButton and self.is_edit_mode:
            world_x, world_y = self.screen_to_world(event.x(), event.y())
            self.editor.add_target_point(world_x, world_y, event.x(), event.y())
    
    def mouseMoveEvent(self, event):
        if self.is_dragging:
            self.view_offset_x -= (event.x() - self.drag_start_x) / self.view_scale
            self.view_offset_y -= (event.y() - self.drag_start_y) / self.view_scale
            self.drag_start_x = event.x()
            self.drag_start_y = event.y()
            self.update()
        if self.rect().contains(event.pos()):
            world_x, world_y = self.screen_to_world(event.x(), event.y())
            self.map_position_moved.emit(world_x, world_y)
    
    def mouseReleaseEvent(self, event):
        if event.button() == Qt.LeftButton:
            self.is_dragging = False
    
    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        try:
            painter.fillRect(self.rect(), Qt.white)
            if self.map_data is not None: self.draw_map(painter)
            if self.costmap_data is not None: self.draw_global_costmap(painter)
            if self.nav_path: self.draw_global_path(painter)
            if self.laser_points_map: self.draw_laser_points(painter)
            self.draw_robot(painter)
            self.draw_target_points(painter)
        except:
            pass
        finally:
            painter.end()
    
    def draw_target_points(self, painter):
        painter.save()
        for point in self.editor.target_points:
            screen_x, screen_y = self.world_to_screen(point.world_x, point.world_y)
            point.update_screen_pos(screen_x, screen_y)
            
            target_size = 8 * self.view_scale / 10.0
            scaled_pixmap = self.target_icon.scaled(
                int(target_size), int(target_size),
                Qt.KeepAspectRatio, Qt.SmoothTransformation
            )
            
            painter.save()
            # 移动到坐标点中心
            painter.translate(screen_x, screen_y)
            
            # 1. 绘制正向的主图标（不再随角度旋转，图标永远立着）
            painter.drawPixmap(
                int(-scaled_pixmap.width() // 2),
                int(-scaled_pixmap.height() // 2),
                scaled_pixmap
            )
            
            painter.rotate(-point.world_theta)
            
            arrow_pen = QPen(QColor(33, 150, 243, 220), 2) # 蓝色指示箭头
            arrow_pen.setCapStyle(Qt.RoundCap)
            arrow_pen.setJoinStyle(Qt.RoundJoin)
            painter.setPen(arrow_pen)
            
            # 箭头起点位于图标边缘，终点向外延伸
            start_x = target_size * 0.4
            arrow_len = target_size * 0.8 + 4
            
            # 主线段
            painter.drawLine(int(start_x), 0, int(arrow_len), 0)
            # 箭头两边的倒刺
            painter.drawLine(int(arrow_len), 0, int(arrow_len - 6), -5)
            painter.drawLine(int(arrow_len), 0, int(arrow_len - 6), 5)
            
            painter.restore()
        painter.restore()
    
    def draw_map(self, painter):
        if self.map_data is None: return
        painter.save()
        height, width, _ = self.map_data.shape
        image = QImage(self.map_data.data, width, height, 3 * width, QImage.Format_RGB888)
        flipped_image = image.mirrored(False, True)
        
        map_center_x = self.map_origin_x + (width * self.map_resolution) / 2
        map_center_y = self.map_origin_y + (height * self.map_resolution) / 2
        screen_center_x, screen_center_y = self.world_to_screen(map_center_x, map_center_y)
        
        painter.translate(screen_center_x, screen_center_y)
        if self.map_rotation != 0.0: painter.rotate(self.map_rotation)
        
        draw_width = width * self.map_resolution * self.view_scale
        draw_height = height * self.map_resolution * self.view_scale
        painter.drawImage(QPointF(-draw_width / 2, -draw_height / 2),
                          flipped_image.scaled(int(draw_width), int(draw_height), Qt.IgnoreAspectRatio, Qt.SmoothTransformation))
        painter.restore()
    
    def draw_global_costmap(self, painter):
        if self.costmap_data is None or self.map_data is None: return
        painter.save()
        height, width, _ = self.costmap_data.shape
        image = QImage(self.costmap_data.data, width, height, 4 * width, QImage.Format_RGBA8888)
        flipped_image = image.mirrored(False, True)
        
        map_center_x = self.map_origin_x + (width * self.map_resolution) / 2
        map_center_y = self.map_origin_y + (height * self.map_resolution) / 2
        screen_center_x, screen_center_y = self.world_to_screen(map_center_x, map_center_y)
        
        painter.translate(screen_center_x, screen_center_y)
        if self.map_rotation != 0.0: painter.rotate(self.map_rotation)
        
        draw_width = width * self.map_resolution * self.view_scale
        draw_height = height * self.map_resolution * self.view_scale
        painter.setOpacity(0.4)
        painter.drawImage(QPointF(-draw_width / 2, -draw_height / 2),
                          flipped_image.scaled(int(draw_width), int(draw_height), Qt.IgnoreAspectRatio, Qt.SmoothTransformation))
        painter.setOpacity(1.0)
        painter.restore()
    
    def draw_global_path(self, painter):
        if len(self.nav_path) < 2: return
        painter.save()
        path_pen = QPen(QColor(0, 200, 0), 2)
        path_pen.setCapStyle(Qt.RoundCap)
        painter.setPen(path_pen)
        prev_point = None
        for world_x, world_y in self.nav_path:
            screen_x, screen_y = self.world_to_screen(world_x, world_y)
            current_point = QPointF(screen_x, screen_y)
            if prev_point is not None: painter.drawLine(prev_point, current_point)
            prev_point = current_point
        painter.restore()
    
    def draw_laser_points(self, painter):
        if not self.laser_points_map: return
        painter.save()
        laser_pen = QPen(QColor(255, 0, 0, 180), 2)
        laser_pen.setCapStyle(Qt.RoundCap)
        painter.setPen(laser_pen)
        for map_x, map_y in self.laser_points_map:
            screen_x, screen_y = self.world_to_screen(map_x, map_y)
            if 0 <= screen_x <= self.width() and 0 <= screen_y <= self.height():
                painter.drawPoint(int(screen_x), int(screen_y))
        painter.restore()
    
    def draw_robot(self, painter):
        screen_x, screen_y = self.world_to_screen(self.robot_x, self.robot_y)
        painter.save()
        painter.translate(screen_x, screen_y)
        rotation_angle = -math.degrees(self.robot_theta) + 45
        painter.rotate(rotation_angle)
        robot_size = 8 * self.view_scale / 10.0
        scaled_pixmap = self.robot_pixmap.scaled(int(robot_size), int(robot_size), Qt.KeepAspectRatio, Qt.SmoothTransformation)
        painter.drawPixmap(-scaled_pixmap.width() // 2, -scaled_pixmap.height() // 2, scaled_pixmap)
        painter.restore()
    
    def reset_view(self):
        self.view_offset_x = self.robot_x
        self.view_offset_y = -self.robot_y
        self.view_scale = 20.0
        self.update()
    
    def zoom_in(self):
        self.view_scale = min(100.0, self.view_scale * 1.2)
        self.update()
    
    def zoom_out(self):
        self.view_scale = max(1.0, self.view_scale / 1.2)
        self.update()
    
    def set_edit_mode(self, enabled):
        self.is_edit_mode = enabled
        if enabled:
            cursor_pixmap = self.cursor_icon.scaled(32, 32, Qt.KeepAspectRatio, Qt.SmoothTransformation)
            self.setCursor(QCursor(cursor_pixmap, cursor_pixmap.width() // 2, cursor_pixmap.height() // 2))
        else:
            self.setCursor(Qt.ArrowCursor)
        self.edit_mode_changed.emit(enabled)
        self.update()
    
    def on_point_deleted(self, point_name): self.update()
    def on_point_renamed(self, old_name, new_name): self.update()
    
    def show_relocate_dialog(self):
        if self.relocate_dialog: self.relocate_dialog.close()
        self.original_robot_x, self.original_robot_y, self.original_robot_theta = self.robot_x, self.robot_y, self.robot_theta
        self.is_relocating = True
        self.relocate_dialog = RelocateDialog(self.robot_x, self.robot_y, math.degrees(self.robot_theta), self)
        self.relocate_dialog.relocate_confirmed.connect(self.on_relocate_confirmed)
        self.relocate_dialog.cancelled.connect(self.on_relocate_cancelled)
        robot_screen_x, robot_screen_y = self.world_to_screen(self.robot_x, self.robot_y)
        self.relocate_dialog.move(self.mapToGlobal(QPoint(int(robot_screen_x + 20), int(robot_screen_y + 20))))
        self.relocate_dialog.show()
        self.relocate_dialog.x_spin.valueChanged.connect(self.update_robot_position)
        self.relocate_dialog.y_spin.valueChanged.connect(self.update_robot_position)
        self.relocate_dialog.theta_spin.valueChanged.connect(self.update_robot_position)
        self.relocate_dialog.destroyed.connect(self.on_relocate_dialog_closed)
    
    def update_robot_position(self):
        if self.relocate_dialog:
            self.robot_x, self.robot_y, self.robot_theta = self.relocate_dialog.x_spin.value(), self.relocate_dialog.y_spin.value(), math.radians(self.relocate_dialog.theta_spin.value())
            self.update()
    
    def on_relocate_dialog_closed(self):
        self.relocate_dialog, self.is_relocating = None, False
        self.update()
    
    def on_relocate_cancelled(self):
        self.robot_x, self.robot_y, self.robot_theta = self.original_robot_x, self.original_robot_y, self.original_robot_theta
        self.is_relocating = False
        self.update()
    
    def on_relocate_confirmed(self, x, y, theta_deg):
        try:
            self.robot_x, self.robot_y, self.robot_theta = x, y, math.radians(theta_deg)
            from geometry_msgs.msg import PoseWithCovarianceStamped
            from tf.transformations import quaternion_from_euler
            pose_msg = PoseWithCovarianceStamped()
            pose_msg.header.stamp, pose_msg.header.frame_id = rospy.Time.now(), "map"
            pose_msg.pose.pose.position.x, pose_msg.pose.pose.position.y, pose_msg.pose.pose.position.z = x, y, 0.0
            q = quaternion_from_euler(0, 0, math.radians(theta_deg))
            pose_msg.pose.pose.orientation.x, pose_msg.pose.pose.orientation.y, pose_msg.pose.pose.orientation.z, pose_msg.pose.pose.orientation.w = q[0], q[1], q[2], q[3]
            if self.initialpose_pub is None: self.initialpose_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=1)
            self.initialpose_pub.publish(pose_msg)
            self.is_relocating = False
            if hasattr(self.parent(), 'right_panel'): self.parent().right_panel.log(f"机器人重定位到: ({x:.2f}, {y:.2f}, {theta_deg:.1f}°)")
            self.update()
        except:
            pass
        
    def on_points_updated(self):
        parent = self.parent()
        while parent and not hasattr(parent, 'left_panel'): parent = parent.parent()
        if parent and hasattr(parent, 'left_panel') and hasattr(parent.left_panel, 'inspection_table'):
            parent.left_panel.inspection_table.set_map_points(self.editor.get_all_points())
        
    def on_point_added(self, point_name, x, y, theta):
        parent = self.parent()
        while parent and not hasattr(parent, 'left_panel'): parent = parent.parent()  
        if parent and hasattr(parent, 'left_panel') and hasattr(parent.left_panel, 'inspection_table'):
            parent.left_panel.inspection_table.set_map_points(self.editor.get_all_points())
    
    def save_target_points(self, filepath):
        import json
        with open(filepath, 'w') as f:
            json.dump([{'name': p.name, 'world_x': p.world_x, 'world_y': p.world_y, 'world_theta': p.world_theta} for p in self.editor.target_points], f, indent=2)
    
    def load_target_points(self, filepath):
        import json, os
        if not os.path.exists(filepath): return
        with open(filepath, 'r') as f: points_data = json.load(f)
        self.editor.clear_all_points()
        for p in points_data: self.editor.add_target_point(p['world_x'], p['world_y'], 0, 0, p['name'], p.get('world_theta', 0.0))
        self.update()
    
    def save_map_image(self, pgm_path):
        try:
            if self.map_data is None: return False
            height, width, _ = self.map_data.shape
            gray_data = np.zeros((height, width), dtype=np.uint8)
            for y in range(height):
                for x in range(width):
                    r, g, b = self.map_data[y, x]
                    if r == 0 and g == 0 and b == 0: gray_data[y, x] = 0
                    elif r == 255 and g == 255 and b == 255: gray_data[y, x] = 255
                    elif r == 128 and g == 128 and b == 128: gray_data[y, x] = 128
                    else: gray_data[y, x] = int(0.299 * r + 0.587 * g + 0.114 * b)
            with open(pgm_path, 'wb') as f:
                f.write(f"P5\n{width} {height}\n255\n".encode('ascii'))
                gray_data.tofile(f)
            return True
        except:
            return False

    def get_map_info(self):
        if self.map_data is None: return None
        return {'resolution': self.map_resolution, 'origin_x': self.map_origin_x, 'origin_y': self.map_origin_y, 'width': self.map_width, 'height': self.map_height}