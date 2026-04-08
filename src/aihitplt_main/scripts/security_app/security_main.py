#!/usr/bin/env python3

import sys
import os
import ctypes
import rospkg, rospy
import numpy as np
from PyQt5.QtWidgets import QApplication
from PyQt5.QtCore import QTimer

from core.robot_interface import RobotInterface
sys.path.append(os.path.join(rospkg.RosPack().get_path('aihitplt_main'), 'scripts'))

class SimplePanTilt:
    """云台控制类"""
    def __init__(self):
        try:
            # 加载SDK
            sdk_path = "/home/aihit/aihitplt_ws/src/aihitplt_security/lib/libhcnetsdk.so"
            self.sdk = ctypes.cdll.LoadLibrary(sdk_path)
            
            # 初始化SDK
            self.sdk.NET_DVR_Init()
            
            # 登录云台
            self.user_id = self._login("192.168.2.64", 8000, "admin", "abcd1234")
            
            if self.user_id >= 0:
                print("云台登录成功")
            else:
                print(f"云台登录失败，错误码: {self.sdk.NET_DVR_GetLastError()}")
                
        except Exception as e:
            print(f"云台初始化失败: {e}")
            self.user_id = -1
    
    def _login(self, ip, port, user, passwd):
        """登录云台"""
        class DeviceInfo(ctypes.Structure):
            _fields_ = [("data", ctypes.c_byte * 200)]
        
        info = DeviceInfo()
        user_id = self.sdk.NET_DVR_Login_V30(
            ip.encode(), port, user.encode(), passwd.encode(), 
            ctypes.byref(info)
        )
        return user_id
    
    def goto_preset(self, preset_num):
        """转到预置点"""
        if self.user_id < 0:
            return False
            
        result = self.sdk.NET_DVR_PTZPreset_Other(
            self.user_id, 1, 39, preset_num
        )
        
        return bool(result)


# 主程序
def main():
    app = QApplication(sys.argv)
    
    # 设置应用程序样式
    app.setStyle('Fusion')
    
    # 创建并显示窗口
    window = RobotInterface()
    window.show()
    
    try:
        # 创建云台控制对象
        pan_tilt = SimplePanTilt()
        window.pan_tilt = pan_tilt
        
        # 延迟2秒发送休眠指令，确保ROS节点和云台初始化完成
        QTimer.singleShot(500, lambda: send_pan_tilt_sleep(pan_tilt, window))
        
    except Exception as e:
        print(f"云台初始化失败: {e}")
        window.pan_tilt = None
    
    sys.exit(app.exec_())

def send_pan_tilt_sleep(pan_tilt, window):
    if pan_tilt and pan_tilt.user_id >= 0:
        success = pan_tilt.goto_preset(40)

if __name__ == '__main__':
    main()