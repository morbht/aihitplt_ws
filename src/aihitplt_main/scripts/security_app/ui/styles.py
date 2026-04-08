#!/usr/bin/env python3

def get_main_stylesheet():
    """获取主样式表"""
    return """
        QMainWindow {
            background-color: #f5f5f5;
        }
        QGroupBox {
            font-weight: bold;
            border: 2px solid #e0e0e0;
            border-radius: 8px;
            margin-top: 10px;
            padding-top: 10px;
        }
        QGroupBox::title {
            subcontrol-origin: margin;
            left: 10px;
            padding: 0 5px 0 5px;
        }
        QPushButton {
            background-color: #1976d2;
            color: white;
            border: none;
            border-radius: 4px;
            padding: 6px 12px;
            font-weight: 500;
            font-size: 11px;
        }
        QPushButton:hover {
            background-color: #1565c0;
        }
        QPushButton:pressed {
            background-color: #0d47a1;
        }
        QComboBox {
            border: 1px solid #e0e0e0;
            border-radius: 4px;
            padding: 4px;
            background-color: white;
            font-size: 11px;
            height: 24px;
        }
        QComboBox:hover {
            border-color: #1976d2;
        }
        QListWidget {
            border: 1px solid #e0e0e0;
            border-radius: 4px;
            background-color: white;
            font-size: 11px;
        }
        QTextEdit {
            border: 1px solid #e0e0e0;
            border-radius: 4px;
            background-color: white;
            font-size: 10px;
        }
        QLineEdit {
            border: 1px solid #e0e0e0;
            border-radius: 4px;
            padding: 3px;
            background-color: white;
            font-size: 11px;
            height: 22px;
        }
        QCheckBox {
            font-size: 11px;
            spacing: 5px;
        }
        QCheckBox::indicator {
            width: 14px;
            height: 14px;
        }
        QLabel {
            font-size: 11px;
        }
    """

def get_map_toolbar_stylesheet():
    """获取地图工具栏样式表"""
    return """
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
    """

def get_start_button_stylesheet(color="#1976d2"):
    """获取开始按钮样式表"""
    return f"""
        QPushButton {{
            background-color: {color};
            color: white;
            font-weight: bold;
            font-size: 11px;
        }}
    """
