#!/usr/bin/env python3
import ctypes
import rospy

class SimpleCamera:
    def __init__(self):
        # 加载SDK
        self.sdk = ctypes.cdll.LoadLibrary(
            "/home/aihit/aihitplt_ws/src/aihitplt_pan_tilt_camera/lib/libhcnetsdk.so"
        )
        
        # 初始化SDK
        self.sdk.NET_DVR_Init()
        
        # 登录
        self.user_id = self._login("192.168.2.64", 8000, "admin", "abcd1234")
    
    def _login(self, ip, port, user, passwd):
        """登录"""
        class DeviceInfo(ctypes.Structure):
            _fields_ = [("data", ctypes.c_byte * 200)]
        
        info = DeviceInfo()
        user_id = self.sdk.NET_DVR_Login_V30(
            ip.encode(), port, user.encode(), passwd.encode(), 
            ctypes.byref(info)
        )
        
        if user_id < 0:
            print(f"登录失败，错误码: {self.sdk.NET_DVR_GetLastError()}")
        return user_id
    
    def goto_preset(self, preset_num):
        if self.user_id < 0:
            return False
            
        result = self.sdk.NET_DVR_PTZPreset_Other(
            self.user_id, 1, 39, preset_num
        )
        
        if result:
            print(f"转到预置点{preset_num}")
        else:
            print(f"失败: {self.sdk.NET_DVR_GetLastError()}")
        
        return bool(result)

# 使用示例
if __name__ == "__main__":
    cam = SimpleCamera()
    if cam.user_id >= 0:
        cam.goto_preset(1)  # 转到预置点1
        rospy.sleep(2)
        cam.goto_preset(40)
        