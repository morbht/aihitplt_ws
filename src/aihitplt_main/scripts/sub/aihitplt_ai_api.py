#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import rospy
import subprocess
from std_msgs.msg import Int32MultiArray, String

class aihitplt_ai_api:
    def __init__(self):
        self.off_line_data_txt = None
        self.voice_process = None
        self.latest_sensor_data = {}  
        self.multi_camera_process = None  
        
        # 1. 语音离线识别订阅
        rospy.Subscriber(
            "/voice/aihitplt_voice_off_line_topic", 
            Int32MultiArray, 
            self.voice_off_line_data
        )

        # 2. 传感器数据订阅
        rospy.Subscriber(
            "/muti_ai/sensor_data", 
            String, 
            self._sensor_callback
        )

        # 3. 云台控制发布器
        self.pan_tilt_pub = rospy.Publisher(
            "/muti_ai/pan_control", 
            String, 
            queue_size=10
        )

        rospy.sleep(0.5)  # 等待发布器和订阅器注册完成

    # ================= 语音系统 API =================

    def start_voice_system(self):
        """
        启动离线语音系统
        """
        if self.voice_process is not None and self.voice_process.poll() is None:
            rospy.logwarn("语音系统已经在运行中，无需重复启动！")
            return

        try:
            self.voice_process = subprocess.Popen([
                "roslaunch", "aihitplt_voice_system", "aihitplt_voice_off_line.launch"
            ])
            rospy.sleep(3)
            rospy.loginfo("语音系统启动完成")
        except Exception as e:
            rospy.logerr(f"启动语音系统失败: {e}")

    def stop_voice_system(self):
        """
        通过 API 动态关闭语音系统
        """
        if self.voice_process is not None:
            rospy.loginfo("正在关闭语音系统...")
            self.voice_process.terminate()
            self.voice_process.wait()
            self.voice_process = None
            rospy.loginfo("语音系统已关闭")

    def voice_off_line(self) -> str:
        self.off_line_data_txt = None
        while self.off_line_data_txt is None and not rospy.is_shutdown():
            rospy.sleep(0.1)
            
        if self.off_line_data_txt is not None:
            return "".join(map(str, self.off_line_data_txt))
        return ""
       
    def voice_off_line_data(self, msg: Int32MultiArray):
        self.off_line_data_txt = msg.data
        rospy.loginfo(f"接收到语音识别数据: {msg.data}")
        
    # ================= 双深度相机 API =================
        
    def start_multi_camera(self):
        """
        后台启动双深度相机系统
        """
        # 防止重复启动
        if self.multi_camera_process is not None and self.multi_camera_process.poll() is None:
            rospy.logwarn("双深度相机已经在运行中")
            return

        try:
            # 使用 subprocess.Popen 后台运行 roslaunch 命令
            self.multi_camera_process = subprocess.Popen([
                "roslaunch", "aihitplt_multimodal_ai", "multi_camera.launch"
            ])
            rospy.sleep(3) # 给节点一点启动时间
            rospy.loginfo("双深度相机启动完成")
        except Exception as e:
            rospy.logerr(f"启动双深度相机失败: {e}")

    def stop_multi_camera(self):
        """
        关闭双深度相机系统
        """
        if self.multi_camera_process is not None:
            rospy.loginfo("正在关闭双深度相机...")
            self.multi_camera_process.terminate()
            self.multi_camera_process.wait()
            self.multi_camera_process = None
            rospy.loginfo("双深度相机已关闭")

    # ================= 传感器获取 API =================

    def _sensor_callback(self, msg: String):
        """
        内部回调函数：解析传感器节点发来的单行字符串
        例如: timestamp:"1234" temperature:"25.00" humidity:"50.00" ...
        """
        try:
            data_str = msg.data.strip()
            items = data_str.split(' ')
            for item in items:
                if ':' in item:
                    key, val = item.split(':', 1)
                    val = val.strip('"')  # 移除前后的双引号
                    self.latest_sensor_data[key] = val
        except Exception as e:
            rospy.logerr(f"解析传感器数据失败: {e}")

    def get_sensor_data(self) -> dict:
        """
        获取最新的传感器数据
        
        Returns:
            dict: 包含各项传感器数据的字典，例如 {'temperature': '25.00', 'smoke': '0', ...}
        """
        return self.latest_sensor_data


    # ================= 云台控制 API =================

    def send_pan_tilt_cmd(self, cmd: str):
        """
        发送原生云台控制指令
        
        Args:
            cmd (str): 指令字符串 ('c', 'a', 'd', 'w', 's', 或者 '水平角度,垂直角度')
        """
        try:
            msg = String()
            msg.data = str(cmd)
            self.pan_tilt_pub.publish(msg)
            rospy.loginfo(f"已发送云台指令: {cmd}")
        except Exception as e:
            rospy.logerr(f"发送云台指令失败: {e}")

    def set_pan_tilt_angle(self, h: int, v: int):
        """
        设置云台绝对角度
        
        Args:
            h (int): 水平角度 (0-180)
            v (int): 垂直角度 (0-180)
        """
        # 限制角度范围防止越界报错
        h = max(0, min(180, int(h)))
        v = max(0, min(180, int(v)))
        self.send_pan_tilt_cmd(f"{h},{v}")

    def reset_pan_tilt(self):
        """
        将云台复位（居中 90, 90）
        """
        self.send_pan_tilt_cmd("c")


if __name__ == "__main__":
    try:
        rospy.init_node("aihitplt_function_api", anonymous=True)
        api = aihitplt_ai_api()
        
        # 测试云台与传感器
        # api.reset_pan_tilt()
        # rospy.sleep(1)
        # print("当前传感器数据:", api.get_sensor_data())
        
        rospy.spin()
    except rospy.ROSInterruptException:
        if 'api' in locals():
            api.stop_voice_system()
            api.stop_multi_camera()  # 确保在 ROS 退出时关闭相机进程