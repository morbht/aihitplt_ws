#!/usr/bin/env python3
"""
异常处理模块 - 当环境指数降至C级时，启动应急相机巡查
"""

import rospy
import os
import subprocess
from PyQt5.QtWidgets import QMessageBox
from PyQt5.QtCore import QObject, pyqtSignal, QTimer
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist
from aihitplt_yolo.msg import DetectResult


class EmergencyHandler(QObject):
    """异常处理器"""
    
    emergency_triggered = pyqtSignal()
    emergency_cleared = pyqtSignal()
    voice_played = pyqtSignal(bool)
    show_dialog_signal = pyqtSignal(str, str)
    start_scan_signal = pyqtSignal()
    begin_scan_signal = pyqtSignal()
    send_preset_signal = pyqtSignal(int)
    request_close_pan_tilt = pyqtSignal()
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.parent = parent
        
        # 状态变量
        self.emergency_mode = False
        self.current_grade = "A"
        self.current_score = 100
        self.grade_history = []
        self.history_maxlen = 3
        self.trigger_count = 0
        self.recovery_count = 0
        self.trigger_threshold = 3
        self.recovery_threshold = 3
        
        # 语音播报
        self.voice_playing = False
        self.voice_process = None
        self.voice_file = "/home/aihit/aihitplt_ws/src/aihitplt_main/scripts/security_app/voice/smoke.wav"
        self.alert_voice_file = "/home/aihit/aihitplt_ws/src/aihitplt_main/scripts/security_app/voice/fire_alarm.wav"
        
        # 巡查状态
        self.is_scanning = False
        self.current_point_index = -1
        self.preset_points = list(range(1, 11))
        self.scan_started = False
        
        # 检测相关
        self.detection_history = []
        self.detection_threshold = 3
        self.alert_playing = False
        self.alert_count = 0
        self.alert_max_count = 3
        self.pending_continue = False
        self.current_point_alert_triggered = False
        self.navigation_in_progress_after_alert = False
        
        # 恢复信息
        self._pending_resume = None
        
        # 连接信号
        self.start_scan_signal.connect(self.start_camera_scan)
        self.begin_scan_signal.connect(self.begin_scanning_from_point1)
        self.send_preset_signal.connect(self._send_preset_command)
        self.show_dialog_signal.connect(self._show_confirm_dialog)
        self.request_close_pan_tilt.connect(self._handle_close_pan_tilt)
        
        self.init_ros()
        rospy.loginfo("异常处理模块已初始化")
    
    def init_ros(self):
        """初始化ROS"""
        try:
            self.grade_sub = rospy.Subscriber('/environment_grade', String, self.grade_callback)
            self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
            self.emergency_pub = rospy.Publisher('/emergency_state', Bool, queue_size=10)
            self.pan_tilt_pub = rospy.Publisher('/pan_tilt_camera/preset_control', String, queue_size=10)
            self.detect_sub = rospy.Subscriber('/pan_tilt_camera/DetectMsg', DetectResult, self.detect_callback)
        except Exception as e:
            rospy.logerr(f"ROS初始化失败: {e}")
    
    def grade_callback(self, msg):
        """等级回调"""
        try:
            data = msg.data.split(',')
            if len(data) != 2:
                return
            grade = data[0].strip()
            score = int(data[1].strip())
            self.current_grade = grade
            self.current_score = score
            self.update_grade_history(grade)
        except Exception as e:
            rospy.logerr(f"处理等级数据失败: {e}")
    
    def detect_callback(self, msg):
        """检测回调"""
        if not self.is_scanning or self.alert_playing or self.pending_continue or self.navigation_in_progress_after_alert:
            return
        if self.current_point_alert_triggered:
            return
        
        try:
            if msg.detected and msg.box_count > 0 and msg.class_name in ["Fire", "Smoke"] and msg.confidence > 0.7:
                rospy.loginfo(f"检测到 {msg.class_name}，可信度: {msg.confidence:.2f}")
                self.detection_history.append(True)
                if len(self.detection_history) > self.detection_threshold:
                    self.detection_history.pop(0)
                if len(self.detection_history) == self.detection_threshold and all(self.detection_history):
                    rospy.logwarn(f"连续{self.detection_threshold}次检测到{msg.class_name}，触发警报")
                    self.current_point_alert_triggered = True
                    self._trigger_alert()
            else:
                self.detection_history = []
        except Exception as e:
            rospy.logerr(f"处理检测消息失败: {e}")
    
    def _trigger_alert(self):
        """触发警报"""
        self.is_scanning = False
        self.pending_continue = True
        self.detection_history = []
        self.alert_count = 0
        self._play_alert_voice()
        rospy.logwarn("触发警报，停止巡查")
        if hasattr(self.parent, 'log'):
            self.parent.log("警告，发现火源/烟雾")
    
    def _play_alert_voice(self):
        """播报警报"""
        if self.alert_playing or not os.path.exists(self.alert_voice_file):
            if not os.path.exists(self.alert_voice_file):
                self._on_alert_complete()
            return
        
        self.alert_playing = True
        self.alert_count += 1
        self.voice_process = subprocess.Popen(['aplay', self.alert_voice_file], 
                                              stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        QTimer.singleShot(0, self._start_alert_timer)
    
    def _start_alert_timer(self):
        """启动警报定时器"""
        self.alert_timer = QTimer()
        self.alert_timer.timeout.connect(self._check_alert_status)
        self.alert_timer.start(500)
    
    def _check_alert_status(self):
        """检查警报状态"""
        if not self.voice_process:
            return
        self.voice_process.poll()
        if self.voice_process.returncode is not None:
            self.alert_timer.stop()
            self.voice_process = None
            self.alert_playing = False
            if self.alert_count < self.alert_max_count:
                QTimer.singleShot(500, self._play_alert_voice)
            else:
                self._on_alert_complete()
    
    def _on_alert_complete(self):
        """警报完成"""
        rospy.loginfo("警报播报完成")
        self.show_dialog_signal.emit("检测到火情/烟雾", 
                                     "检测到火情或烟雾，是否继续执行巡检任务？\n选择'是'则继续巡检，选择'否'则停止巡检任务")
    
    def _show_confirm_dialog(self, title, text):
        """显示确认对话框"""
        reply = QMessageBox.question(None, title, text, QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
        if reply == QMessageBox.Yes:
            self.pending_continue = False
            self.detection_history = []
            self.navigation_in_progress_after_alert = True
            self._exit_emergency_mode_immediately()
        else:
            rospy.loginfo("停止巡检")
            if hasattr(self.parent, 'log'):
                self.parent.log("停止巡检任务")
            self.pending_continue = False
            try:
                inspection_table = self.parent.parent.left_panel.inspection_table
                if inspection_table.is_inspecting:
                    inspection_table.stop_inspection()
            except:
                pass
            self.exit_emergency_mode()
    
    def _exit_emergency_mode_immediately(self):
        """立即退出应急模式"""
        rospy.loginfo("========== 立即退出应急模式，恢复巡检 ==========")
        
        self.is_scanning = False
        self.scan_started = False
        self._send_preset_command(40)
        
        # 保存巡检恢复信息
        self._pending_resume = None
        try:
            inspection_table = self.parent.parent.left_panel.inspection_table
            if inspection_table.is_paused:
                self._pending_resume = {
                    'index': inspection_table.paused_point_index,
                    'name': inspection_table.paused_point_name,
                    'x': inspection_table.paused_x,
                    'y': inspection_table.paused_y
                }
        except:
            pass
        
        # 通过信号关闭云台
        if hasattr(self.parent, 'pan_tilt_enabled') and self.parent.pan_tilt_enabled:
            self.request_close_pan_tilt.emit()
            QTimer.singleShot(2500, self._complete_exit_with_resume)
        else:
            self._complete_exit_with_resume()
        
        self.stop_voice()
        self.emergency_cleared.emit()
    
    def _handle_close_pan_tilt(self):
        """处理关闭云台请求"""
        try:
            if self.parent and hasattr(self.parent, 'toggle_pan_tilt'):
                self.parent.toggle_pan_tilt()
        except RuntimeError:
            rospy.logwarn("对象已删除，跳过关闭云台")
    
    def _complete_exit_with_resume(self):
        """完成退出并恢复巡检"""
        self.emergency_pub.publish(Bool(False))
        
        # 恢复巡检
        if self._pending_resume:
            try:
                inspection_table = self.parent.parent.left_panel.inspection_table
                inspection_table.paused_point_index = self._pending_resume['index']
                inspection_table.paused_point_name = self._pending_resume['name']
                inspection_table.paused_x = self._pending_resume['x']
                inspection_table.paused_y = self._pending_resume['y']
                inspection_table.is_paused = True
                QTimer.singleShot(500, inspection_table.resume_inspection)
            except:
                pass
        
        if hasattr(self.parent, 'log'):
            try:
                self.parent.log("应急模式已解除，云台已关闭，继续巡检任务")
            except RuntimeError:
                pass
    
    def update_grade_history(self, grade):
        """更新等级历史"""
        self.grade_history.append(grade)
        if len(self.grade_history) > self.history_maxlen:
            self.grade_history.pop(0)
        
        is_c_grade = (grade == "C")
        
        if self.navigation_in_progress_after_alert:
            return
        
        if not self.emergency_mode:
            if is_c_grade:
                self.trigger_count += 1
            else:
                self.trigger_count = 0
            if self.trigger_count >= self.trigger_threshold:
                QTimer.singleShot(0, self.enter_emergency_mode)
        else:
            if not self.is_scanning and not self.pending_continue:
                if not is_c_grade:
                    self.recovery_count += 1
                else:
                    self.recovery_count = 0
                if self.recovery_count >= self.recovery_threshold:
                    QTimer.singleShot(0, self.exit_emergency_mode)
    
    def on_navigation_complete(self):
        """导航完成回调"""
        self.navigation_in_progress_after_alert = False
    
    def enter_emergency_mode(self):
        """进入应急模式"""
        if self.emergency_mode or self.navigation_in_progress_after_alert:
            return
        
        rospy.logwarn(f"环境异常！进入应急模式 (等级: {self.current_grade}级)")
        self.emergency_mode = True
        self.trigger_count = 0
        self.recovery_count = 0
        self.scan_started = False
        self.detection_history = []
        self.pending_continue = False
        self.current_point_alert_triggered = False
        
        self.stop_robot()
        self.emergency_pub.publish(Bool(True))
        
        if not self.voice_playing:
            self.play_voice()
        
        self.emergency_triggered.emit()
        QTimer.singleShot(2000, self.start_scan_signal.emit)
    
    def start_camera_scan(self):
        """启动相机巡查"""
        if self.scan_started:
            return
        self.scan_started = True
        rospy.loginfo("启动应急相机巡查")
        
        if hasattr(self.parent, 'pan_tilt_enabled'):
            if not self.parent.pan_tilt_enabled:
                if hasattr(self.parent, 'detect_check'):
                    self.parent.detect_check.setChecked(True)
                if hasattr(self.parent, 'toggle_pan_tilt'):
                    self.parent.toggle_pan_tilt()
                QTimer.singleShot(10000, self.begin_scan_signal.emit)
            else:
                if hasattr(self.parent, 'init_preset_sent'):
                    self.parent.init_preset_sent = False
                QTimer.singleShot(2000, self.begin_scan_signal.emit)
    
    def begin_scanning_from_point1(self):
        """开始巡查"""
        if hasattr(self.parent, 'pan_tilt_enabled') and not self.parent.pan_tilt_enabled:
            self.scan_started = False
            return
        
        if self.is_scanning:
            return
        
        self.is_scanning = True
        self.current_point_index = 0
        self.current_point_alert_triggered = False
        
        if hasattr(self.parent, 'init_preset_sent'):
            self.parent.init_preset_sent = False
        
        self._send_preset_command(1)
        QTimer.singleShot(3000, self._start_point_detection)
    
    def _send_preset_command(self, preset_num):
        """发送云台指令"""
        try:
            self.pan_tilt_pub.publish(String(f"go,{preset_num}"))
        except Exception as e:
            rospy.logerr(f"发送云台指令失败: {e}")
    
    def _start_point_detection(self):
        """开始点位检测"""
        if not self.is_scanning or self.pending_continue:
            return
        self.detection_history = []
        QTimer.singleShot(3000, self._move_to_next_point)
    
    def _move_to_next_point(self):
        """移动到下一点"""
        if not self.is_scanning or self.pending_continue:
            return
        
        self.current_point_index += 1
        self.current_point_alert_triggered = False
        
        if self.current_point_index >= len(self.preset_points):
            self._on_scan_complete()
            return
        
        self._send_preset_command(self.preset_points[self.current_point_index])
        QTimer.singleShot(3000, self._start_point_detection)
    
    def _on_scan_complete(self):
        """巡查完成"""
        rospy.loginfo("应急巡查完成")
        self.is_scanning = False
        self.scan_started = False
        self.current_point_alert_triggered = False
        self._send_preset_command(40)
        
        if self.current_grade != "C":
            rospy.loginfo("环境已恢复正常，退出应急模式")
            QTimer.singleShot(1000, self.exit_emergency_mode)
    
    def exit_emergency_mode(self):
        """退出应急模式"""
        if not self.emergency_mode:
            return
        
        rospy.loginfo("========== 退出应急模式 ==========")
        
        self.emergency_mode = False
        self.is_scanning = False
        self.scan_started = False
        self.recovery_count = 0
        self.current_point_index = -1
        self.detection_history = []
        self.pending_continue = False
        self.current_point_alert_triggered = False
        
        self._send_preset_command(40)
        self.emergency_pub.publish(Bool(False))
        self.stop_voice()
        self.emergency_cleared.emit()
    
    def stop_robot(self):
        """停止机器人"""
        try:
            twist = Twist()
            for _ in range(3):
                self.cmd_vel_pub.publish(twist)
                rospy.sleep(0.1)
        except Exception as e:
            rospy.logerr(f"停止机器人失败: {e}")
    
    def play_voice(self):
        """播报语音"""
        try:
            if self.voice_playing:
                self.stop_voice()
            if not os.path.exists(self.voice_file):
                self.voice_played.emit(False)
                return
            self.voice_process = subprocess.Popen(['aplay', self.voice_file], 
                                                  stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            self.voice_playing = True
            QTimer.singleShot(0, self._start_voice_timer)
        except Exception as e:
            rospy.logerr(f"播报语音失败: {e}")
            self.voice_played.emit(False)
    
    def _start_voice_timer(self):
        """启动语音定时器"""
        self.voice_timer = QTimer()
        self.voice_timer.timeout.connect(self.check_voice_status)
        self.voice_timer.start(500)
    
    def stop_voice(self):
        """停止语音"""
        try:
            if self.voice_process:
                self.voice_process.terminate()
                self.voice_process = None
            self.voice_playing = False
            if hasattr(self, 'voice_timer'):
                self.voice_timer.stop()
        except Exception as e:
            rospy.logerr(f"停止语音失败: {e}")
    
    def check_voice_status(self):
        """检查语音状态"""
        if not self.voice_process:
            return
        self.voice_process.poll()
        if self.voice_process.returncode is not None:
            self.voice_playing = False
            self.voice_process = None
            if hasattr(self, 'voice_timer'):
                self.voice_timer.stop()
            self.voice_played.emit(True)
    
    def shutdown(self):
        """关闭处理器"""
        self.stop_voice()
        if self.emergency_mode:
            QTimer.singleShot(0, self.exit_emergency_mode)