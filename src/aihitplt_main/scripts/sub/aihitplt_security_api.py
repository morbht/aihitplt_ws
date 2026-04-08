#!/usr/bin/env python3

import rospy
import cv2
import json
from std_msgs.msg import String, Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import os

class aihitplt_security_api:
    def __init__(self):
        """初始化API"""
        self.control_pub = rospy.Publisher('/pan_tilt_camera_control', String, queue_size=10)
        self.preset_pub = rospy.Publisher('/pan_tilt_camera/preset_control',String , queue_size=10)
        self.speed_pub = rospy.Publisher('/pan_tilt_camera_speed', String, queue_size=10)
        self.light_pub = rospy.Publisher('/pan_tilt_camera_light', Bool, queue_size=10)
        self.wiper_pub = rospy.Publisher('/pan_tilt_camera_wiper', Bool, queue_size=10)
        
        # 订阅传感器数据
        rospy.Subscriber('/security_sensors', String, self._sensor_callback)
        
        # 初始化传感器数据
        self.sensor_data = None
        
        # 图像处理
        self.bridge = CvBridge()
        self.latest_image = None
        
        rospy.loginfo("aihitplt_security_api initialized")
    
    def cam_move(self, direction, speed=3):
        """
        云台相机移动
        :param direction: "up"/"down"/"left"/"right"
        :param speed: 1-7
        :return: bool 成功/失败
        """
        try:
            dir_map = {"up": "w", "down": "s", "left": "a", "right": "d"}
            
            if direction not in dir_map:
                print(f"错误: 无效方向 {direction}")
                return False
            
            if speed < 1 or speed > 7:
                speed = 3
                print(f"警告: 速度超出1-7范围，使用默认值3")
            
            self._set_speed(speed)
            rospy.sleep(0.2)
            
            return self._send_control(dir_map[direction])
                
        except Exception as e:
            print(f"移动失败: {e}")
            return False
    
    def cam_stop(self):
        """
        云台相机停止
        :return: bool 成功/失败
        """
        try:
            return self._send_control("c")
        except Exception as e:
            print(f"停止失败: {e}")
            return False
        
    def light_control(self, command):
        """
        灯光控制
        :param command: True=开启, False=关闭
        :return: bool 成功/失败
        """
        try:
            self.light_pub.publish(Bool(command))
            return True
        except Exception as e:
            return False
        
    def wiper_control(self, command):
        """
        雨刷控制
        :param command: True=开启, False=关闭
        :return: bool 成功/失败
        """
        try:
            self.wiper_pub.publish(Bool(command))
            return True
        except Exception as e:
            return False
            
    def image_capture(self, save_path="/home/aihit/aihitplt_ws/src/aihitplt_main/img/image.jpg", topic="/pan_tilt_camera/image"):
        """
        捕获并保存图像
        :param save_path: 保存路径，默认当前目录
        :param topic: 图像话题
        :return: bool 成功/失败
        """
        try:
            # 订阅图像话题
            image_msg = rospy.wait_for_message(topic, Image, timeout=2.0)
            
            # 转换图像格式
            cv_image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
            
            # 保存图像
            cv2.imwrite(save_path, cv_image)
            
            rospy.loginfo(f"图像保存成功: {save_path}")
            return True
            
        except rospy.exceptions.ROSException:
            print(f"错误: 未收到{topic}话题的图像数据")
            return False
        except Exception as e:
            print(f"图像捕获失败: {e}")
            return False
    
    def adjust_focus(self,command):
        """
        调整焦距
        :param command: 类型为str,"1"为焦距变大,"2"为焦距变小
        :return: bool 成功/失败
        """
        try:
            adjust_focus_msg = String()
            adjust_focus_msg.data = command
            self.control_pub.publish(adjust_focus_msg)
            return True

        except Exception as e:
            return False
        
    def adjust_aperture(self,command):
        """
        调整光圈
        :param command: 类型为str,"7"为光圈变大,"8"为光圈缩小
        :return: bool 成功/失败
        """
        try:
            adjust_aperture_msg = String()
            adjust_aperture_msg.data = command
            self.control_pub.publish(adjust_aperture_msg)
            return True
        except Exception as e:
            return False
        
    def preset_point_control(self,command):
        
        preset_msg = String()
        preset_msg.data = command
        self.preset_pub.publish(preset_msg)        

    
    def get_sensor_data(self):
        """
        获取传感器数据
        :return: dict 传感器数据字典，如果无数据返回None
        """
        try:
            return self.sensor_data
        except Exception as e:
            print(f"获取传感器数据失败: {e}")
            return None
        
    def get_estop_state(self):
        """
        获取传感器数据
        :return: dict 传感器数据字典，如果无数据返回None
        """
        try:
            estop_state = self.sensor_data
            if estop_state['emergency_stop'] == 0:
                return True
            else:
                return False
        
        except Exception as e:
            pass

    #以下为内部处理函数
    
    def _sensor_callback(self, msg):
        """传感器数据回调函数"""
        try:
            self.sensor_data = json.loads(msg.data)
        except Exception as e:
            print(f"解析传感器数据失败: {e}")
            self.sensor_data = None
    
    def _set_speed(self, speed):
        """设置速度"""
        try:
            self.speed_pub.publish(str(speed))
            rospy.sleep(0.05)
            return True
        except Exception as e:
            print(f"设置速度失败: {e}")
            return False
    
    def _send_control(self, command):
        """发送控制命令"""
        try:
            self.control_pub.publish(command)
            rospy.sleep(0.05)
            return True
        except Exception as e:
            print(f"发送控制命令失败: {e}")
            return False