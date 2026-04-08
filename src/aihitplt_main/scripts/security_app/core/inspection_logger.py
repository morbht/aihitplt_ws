#!/usr/bin/env python3
"""
巡检日志记录器 - 记录巡检任务的全过程
"""

import os
import yaml
from datetime import datetime


class InspectionLogger:
    """巡检日志记录器"""
    
    def __init__(self, config_path="/home/aihit/aihitplt_ws/src/aihitplt_main/scripts/security_app/config/log_config/log_config.yaml"):
        self.config = self.load_config(config_path)
        self.log_dir = self.config.get('log_config', {}).get('log_dir', "/home/aihit/aihitplt_ws/src/aihitplt_main/scripts/security_app/log")
        self.current_log_file = None
        self.task_id = None
        self.start_time = None
        self.events = []
        self.points_list = []
        
        # 确保日志目录存在
        os.makedirs(self.log_dir, exist_ok=True)
        
        # 清理旧日志（如果配置了）
        if self.config.get('log_config', {}).get('log_management', {}).get('max_log_days', 0) > 0:
            self._clean_old_logs()
    
    def load_config(self, config_path):
        """加载配置文件"""
        default_config = {
            'log_config': {
                'log_dir': "/home/aihit/aihitplt_ws/src/aihitplt_main/scripts/security_app/log",
                'record_conditions': {
                    'navigation': {'start_task': True, 'reach_point': True, 'navigation_failed': True},
                    'abnormal': {'grade_drop_to_c': True, 'emergency_mode': True, 'sensor_over_threshold': True},
                    'detection': {'detect_fire': True, 'detect_smoke': True, 'confidence_threshold': 0.7},
                    'system': {'log_interval': 0, 'record_battery': False, 'record_position': False}
                },
                'log_management': {'auto_save': True, 'max_log_files': 20, 'max_log_days': 7},
                'log_format': {
                    'time_format': "%H:%M:%S",
                    'date_format': "%Y-%m-%d",
                    'file_name_format': "巡检日志_%Y%m%d_%H%M%S.txt"
                }
            }
        }
        
        try:
            if os.path.exists(config_path):
                with open(config_path, 'r', encoding='utf-8') as f:
                    config = yaml.safe_load(f)
                    # 合并配置
                    if config and 'log_config' in config:
                        return config
            return default_config
        except Exception as e:
            print(f"加载日志配置文件失败: {e}")
            return default_config
    
    def _clean_old_logs(self):
        """清理旧日志文件"""
        try:
            max_days = self.config.get('log_config', {}).get('log_management', {}).get('max_log_days', 7)
            if max_days <= 0:
                return
            
            import time
            now = time.time()
            for filename in os.listdir(self.log_dir):
                filepath = os.path.join(self.log_dir, filename)
                if os.path.isfile(filepath) and filename.startswith("巡检日志_"):
                    # 检查文件修改时间
                    if os.path.getmtime(filepath) < now - (max_days * 86400):
                        os.remove(filepath)
                        print(f"清理旧日志: {filename}")
        except Exception as e:
            print(f"清理旧日志失败: {e}")
    
    def _get_file_name(self):
        """生成日志文件名"""
        format_str = self.config.get('log_config', {}).get('log_format', {}).get('file_name_format', "巡检日志_%Y%m%d_%H%M%S.txt")
        return datetime.now().strftime(format_str)
    
    def start_task(self, points_list, loop_enabled):
        """开始新任务"""
        self.start_time = datetime.now()
        self.task_id = self.start_time.strftime("%Y%m%d_%H%M%S")
        self.events = []
        self.points_list = points_list
        
        # 创建新日志文件
        filename = self._get_file_name()
        self.current_log_file = os.path.join(self.log_dir, filename)
        
        # 写入文件头
        date_format = self.config.get('log_config', {}).get('log_format', {}).get('date_format', "%Y-%m-%d")
        time_format = self.config.get('log_config', {}).get('log_format', {}).get('time_format', "%H:%M:%S")
        
        with open(self.current_log_file, 'w', encoding='utf-8') as f:
            f.write(f"=== 巡检任务日志 ===\n")
            f.write(f"任务ID: {self.task_id}\n")
            f.write(f"开始时间: {self.start_time.strftime(f'{date_format} {time_format}')}\n")
            f.write(f"巡检模式: {'循环任务' if loop_enabled else '单次任务'}\n")
            f.write(f"巡检点位: {', '.join(points_list)}\n")
            f.write(f"\n=== 详细记录 ===\n")
        
        # 记录开始事件
        self._log_event("系统", "开始巡检任务")
        
        return self.current_log_file
    
    def log_navigation_start(self, point_name, point_index, total_points):
        """记录开始导航到点位"""
        conditions = self.config.get('log_config', {}).get('record_conditions', {}).get('navigation', {})
        if conditions.get('start_task', True):
            self._log_event("导航", f"前往点位: {point_name} ({point_index}/{total_points})")
    
    def log_point_reached(self, point_name):
        """记录到达点位"""
        conditions = self.config.get('log_config', {}).get('record_conditions', {}).get('navigation', {})
        if conditions.get('reach_point', True):
            self._log_event("到达", f"已到达: {point_name}")
    
    def log_abnormal_detected(self, sensor_type, value, threshold, point_name=None):
        """记录异常检测"""
        conditions = self.config.get('log_config', {}).get('record_conditions', {}).get('abnormal', {})
        if conditions.get('sensor_over_threshold', True):
            location = f"于点位 {point_name}" if point_name else ""
            detail = f"值:{value} 阈值:{threshold}"
            self._log_event("异常", f"检测到{sensor_type}异常 {location} ({detail})")
            
            self.events.append({
                'time': datetime.now(),
                'type': sensor_type,
                'point': point_name,
                'detail': detail
            })
    
    def log_fire_detected(self, confidence, point_name):
        """记录火源检测"""
        conditions = self.config.get('log_config', {}).get('record_conditions', {}).get('detection', {})
        threshold = conditions.get('confidence_threshold', 0.7)
        
        if conditions.get('detect_fire', True) and confidence >= threshold:
            self._log_event("检测", f"在点位 {point_name} 检测到火源 (置信度: {confidence:.2f})")
            
            self.events.append({
                'time': datetime.now(),
                'type': '火源',
                'point': point_name,
                'detail': f"置信度:{confidence:.2f}"
            })
    
    def log_emergency_start(self):
        """记录应急模式启动"""
        conditions = self.config.get('log_config', {}).get('record_conditions', {}).get('abnormal', {})
        if conditions.get('emergency_mode', True):
            self._log_event("应急", "启动应急模式")
    
    def log_emergency_end(self):
        """记录应急模式结束"""
        conditions = self.config.get('log_config', {}).get('record_conditions', {}).get('abnormal', {})
        if conditions.get('emergency_mode', True):
            self._log_event("应急", "应急模式结束")
    
    def log_voice_alert(self, alert_type):
        """记录语音播报"""
        self._log_event("警报", f"播报警报: {alert_type}")
    
    def finish_task(self):
        """完成任务，写入汇总信息"""
        if not self.current_log_file:
            return
        
        end_time = datetime.now()
        date_format = self.config.get('log_config', {}).get('log_format', {}).get('date_format', "%Y-%m-%d")
        time_format = self.config.get('log_config', {}).get('log_format', {}).get('time_format', "%H:%M:%S")
        
        with open(self.current_log_file, 'a', encoding='utf-8') as f:
            f.write(f"\n=== 任务完成 ===\n")
            f.write(f"结束时间: {end_time.strftime(f'{date_format} {time_format}')}\n")
            f.write(f"总耗时: {self._format_duration(self.start_time, end_time)}\n")
            
            if self.events:
                f.write(f"\n=== 异常事件汇总 ===\n")
                f.write(f"{'时间':<20} | {'类型':<8} | {'点位':<12} | 详情\n")
                f.write("-" * 60 + "\n")
                
                for event in self.events:
                    time_str = event['time'].strftime(time_format)
                    point = event['point'] or '--'
                    f.write(f"{time_str:<20} | {event['type']:<8} | {point:<12} | {event['detail']}\n")
        
        # 自动清理旧日志
        self._cleanup_old_files()
    
    def _cleanup_old_files(self):
        """清理旧文件（按数量）"""
        try:
            max_files = self.config.get('log_config', {}).get('log_management', {}).get('max_log_files', 20)
            if max_files <= 0:
                return
            
            files = []
            for filename in os.listdir(self.log_dir):
                filepath = os.path.join(self.log_dir, filename)
                if os.path.isfile(filepath) and filename.startswith("巡检日志_"):
                    files.append((os.path.getmtime(filepath), filepath))
            
            # 按修改时间排序
            files.sort()
            
            # 删除多余的文件
            while len(files) > max_files:
                _, filepath = files.pop(0)  # 删除最旧的文件
                os.remove(filepath)
                print(f"清理旧日志: {os.path.basename(filepath)}")
                
        except Exception as e:
            print(f"清理旧日志失败: {e}")
    
    def get_current_log_path(self):
        """获取当前日志文件路径"""
        return self.current_log_file
    
    def _log_event(self, category, message):
        """记录单条事件"""
        if not self.current_log_file:
            return
        
        time_format = self.config.get('log_config', {}).get('log_format', {}).get('time_format', "%H:%M:%S")
        timestamp = datetime.now().strftime(time_format)
        log_line = f"{timestamp} [{category}] {message}\n"
        
        with open(self.current_log_file, 'a', encoding='utf-8') as f:
            f.write(log_line)
    
    def _format_duration(self, start, end):
        """格式化时长"""
        delta = end - start
        minutes = delta.seconds // 60
        seconds = delta.seconds % 60
        return f"{minutes}分{seconds}秒"