#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import time
import threading
from std_msgs.msg import ColorRGBA, String
from aihitplt_bringup.srv import SetRgb, SetRgbRequest

class RGBFadeController:
    def __init__(self):
        rospy.init_node('rgb_controller', anonymous=True)
        
        # 颜色映射表（颜色名称 -> RGB值）
        self.color_map = {
            'red': (255, 0, 0),
            'green': (0, 155, 0),
            'blue': (0, 0, 255),
            'yellow': (255, 255, 0),
            'cyan': (0, 255, 255),
            'magenta': (255, 0, 255),
            'white': (255, 255, 255),
            'black': (0, 0, 0),
            'orange': (255, 165, 0),
            'purple': (128, 0, 128),
            'pink': (255, 192, 203),
            'off': (0, 0, 0)  # 关闭灯光
        }
        
        # 当前RGB颜色值
        self.current_r = 0
        self.current_g = 155
        self.current_b = 0
        
        # 目标RGB颜色值
        self.target_r = 0
        self.target_g = 0
        self.target_b = 0
        
        # 渐变参数
        self.fade_duration = 2.0  # 渐变持续时间（秒）
        self.fade_steps = 50      # 渐变步数
        self.fade_interval = self.fade_duration / self.fade_steps
        
        # 渐变线程控制
        self.fade_thread = None
        self.fade_active = False
        self.stop_fade = False
        
        # 创建服务代理（用于实际设置颜色）
        rospy.wait_for_service('/set_rgb_color')
        self.set_rgb_service = rospy.ServiceProxy('/set_rgb_color', SetRgb)
        
        # 订阅/set_rgb_color话题
        rospy.Subscriber('/set_rgb_color', String, self.color_callback)
        
        # 创建颜色状态发布者（可选）
        self.color_pub = rospy.Publisher('/current_rgb_color', ColorRGBA, queue_size=10)
        
        rospy.loginfo("支持的颜色: " + ", ".join(self.color_map.keys()))
        
        # 初始化灯光为关闭状态
        self.set_rgb_color(0, 0, 0, enable=True)
        
        rospy.spin()
    
    def color_callback(self, msg):
        """颜色话题回调函数"""
        color_name = msg.data.lower().strip()
        
        if color_name in self.color_map:
            rospy.loginfo(f"接收到颜色命令: {color_name}")
            r, g, b = self.color_map[color_name]
            
            # 开始渐变到新颜色
            self.fade_to_color(r, g, b)
        else:
            # 尝试解析RGB值（格式: "r,g,b" 或 "255 0 0"）
            try:
                # 尝试逗号分隔
                if ',' in color_name:
                    r, g, b = map(int, color_name.split(','))
                # 尝试空格分隔
                else:
                    parts = color_name.split()
                    if len(parts) == 3:
                        r, g, b = map(int, parts)
                    else:
                        rospy.logwarn(f"无法识别的颜色: {color_name}")
                        return
                
                # 验证RGB值范围
                if 0 <= r <= 255 and 0 <= g <= 255 and 0 <= b <= 255:
                    rospy.loginfo(f"接收到RGB颜色: ({r}, {g}, {b})")
                    self.fade_to_color(r, g, b)
                else:
                    rospy.logwarn(f"RGB值超出范围(0-255): ({r}, {g}, {b})")
                    
            except ValueError:
                rospy.logwarn(f"无法解析的颜色格式: {color_name}")
    
    def set_rgb_color(self, r, g, b, enable=True):
        """设置RGB颜色"""
        try:
            req = SetRgbRequest()
            req.en = enable
            req.r = int(r)
            req.g = int(g)
            req.b = int(b)
            
            # 限制颜色值在0-255范围内
            req.r = max(0, min(255, req.r))
            req.g = max(0, min(255, req.g))
            req.b = max(0, min(255, req.b))
            
            response = self.set_rgb_service(req)
            rospy.logdebug(f"设置颜色: R={req.r}, G={req.g}, B={req.b}")
            
            # 更新当前颜色
            self.current_r = req.r
            self.current_g = req.g
            self.current_b = req.b
            
            # 发布当前颜色到话题
            self.publish_current_color()
            
            return True
        except rospy.ServiceException as e:
            rospy.logerr(f"服务调用失败: {e}")
            return False
    
    def publish_current_color(self):
        """发布当前颜色到话题"""
        color_msg = ColorRGBA()
        color_msg.r = self.current_r / 255.0  # 转换为0-1范围
        color_msg.g = self.current_g / 255.0
        color_msg.b = self.current_b / 255.0
        color_msg.a = 1.0
        self.color_pub.publish(color_msg)
    
    def fade_to_color(self, target_r, target_g, target_b):
        """渐变到目标颜色"""
        # 如果正在渐变，先停止
        if self.fade_active and self.fade_thread is not None:
            self.stop_fade = True
            rospy.sleep(0.1)  # 给线程一点时间停止
            if self.fade_thread.is_alive():
                self.fade_thread.join(timeout=0.5)
        
        # 设置目标颜色
        self.target_r = target_r
        self.target_g = target_g
        self.target_b = target_b
        
        # 启动渐变线程
        self.stop_fade = False
        self.fade_thread = threading.Thread(target=self._fade_thread_func)
        self.fade_thread.daemon = True
        self.fade_thread.start()
    
    def _fade_thread_func(self):
        """渐变线程函数"""
        self.fade_active = True
        
        try:
            # 计算每一步的增量
            start_r, start_g, start_b = self.current_r, self.current_g, self.current_b
            delta_r = (self.target_r - start_r) / self.fade_steps
            delta_g = (self.target_g - start_g) / self.fade_steps
            delta_b = (self.target_b - start_b) / self.fade_steps
            
            # 执行渐变
            for step in range(1, self.fade_steps + 1):
                if self.stop_fade:
                    break
                
                # 计算当前步的颜色
                current_r = int(start_r + delta_r * step)
                current_g = int(start_g + delta_g * step)
                current_b = int(start_b + delta_b * step)
                
                # 设置当前颜色
                self.set_rgb_color(current_r, current_g, current_b)
                
                # 等待
                time.sleep(self.fade_interval)
            
            # 确保最终颜色准确
            if not self.stop_fade:
                self.set_rgb_color(self.target_r, self.target_g, self.target_b)
        
        except Exception as e:
            rospy.logerr(f"渐变过程中出错: {e}")
        finally:
            self.fade_active = False
    
    def test_red_green_fade(self):

        # 先设置红色
        self.fade_to_color(255, 0, 0)
        
        # 等待渐变完成
        while self.fade_active:
            time.sleep(0.1)
        
        time.sleep(2.0)
        
        # 渐变到绿色
        self.fade_to_color(0, 255, 0)
        

if __name__ == '__main__':
    try:
        controller = RGBFadeController()
    except rospy.ROSInterruptException:
        pass