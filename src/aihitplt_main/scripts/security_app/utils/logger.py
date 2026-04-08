#!/usr/bin/env python3

import logging
from datetime import datetime

class SystemLogger:
    """系统日志记录器"""
    
    def __init__(self, log_widget=None):
        self.log_widget = log_widget
        self.setup_file_logger()
        
    def setup_file_logger(self):
        """设置文件日志记录器"""
        logging.basicConfig(
            level=logging.INFO,
            format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
            filename='robot_inspection.log',
            filemode='a'
        )
        self.logger = logging.getLogger('RobotInspection')
        
    def log(self, message, level='info'):
        """记录日志"""
        timestamp = datetime.now().strftime("%H:%M:%S")
        formatted_message = f"{timestamp} {message}"
        
        # 输出到控制台
        print(f"[{level.upper()}] {formatted_message}")
        
        # 输出到日志文件
        if level == 'info':
            self.logger.info(message)
        elif level == 'warning':
            self.logger.warning(message)
        elif level == 'error':
            self.logger.error(message)
            
        # 输出到UI日志组件
        if self.log_widget:
            self.log_widget.append(formatted_message)
            
    def info(self, message):
        """信息级别日志"""
        self.log(message, 'info')
        
    def warning(self, message):
        """警告级别日志"""
        self.log(message, 'warning')
        
    def error(self, message):
        """错误级别日志"""
        self.log(message, 'error')
