#!/usr/bin/env python3
# coding:utf-8

import rospy
import math
from aihitplt_bringup.srv import SetRgb

class LightChaser:
    def __init__(self):
        rospy.init_node('light_chaser', anonymous=True)
        self.rate = rospy.Rate(10)  # 10Hz
        
        # 等待服务可用
        rospy.wait_for_service('set_rgb_color')
        self.set_light = rospy.ServiceProxy('set_rgb_color', SetRgb)
        
        # 走马灯参数
        self.speed = 0.5  # 速度控制 (0.1-2.0)
        self.brightness = 255  # 亮度 (0-255)
        self.running = True
        
        # 颜色序列 (RGB)
        self.color_sequence = [
            (255, 0, 0),    # 红色
            (255, 127, 0),  # 橙色
            (255, 255, 0),  # 黄色
            (0, 255, 0),    # 绿色
            (0, 0, 255),    # 蓝色
            (75, 0, 130),   # 靛蓝色
            (148, 0, 211)   # 紫色
        ]
        
        rospy.loginfo("走马灯控制器启动成功！")
        
    def set_light_color(self, r, g, b):
        """设置灯带颜色"""
        try:
            resp = self.set_light(True, r, g, b)
            return resp.res
        except rospy.ServiceException as e:
            rospy.logerr("设置灯带颜色失败: %s", e)
            return "Failed"
    
    def rainbow_chaser(self):
        """彩虹走马灯效果"""
        rospy.loginfo("开始彩虹走马灯效果...")
        color_index = 0
        
        while not rospy.is_shutdown() and self.running:
            # 获取当前颜色
            r, g, b = self.color_sequence[color_index]
            
            # 设置颜色
            result = self.set_light_color(r, g, b)
            if "successfully" not in result:
                rospy.logwarn("颜色设置可能失败: %s", result)
            
            # 更新颜色索引
            color_index = (color_index + 1) % len(self.color_sequence)
            
            # 控制速度
            rospy.sleep(1.0 / self.speed)
    
    def breathing_chaser(self):
        """呼吸灯走马灯效果"""
        rospy.loginfo("开始呼吸灯走马灯效果...")
        color_index = 0
        breath_direction = 1
        breath_value = 0
        
        while not rospy.is_shutdown() and self.running:
            # 获取基础颜色
            base_r, base_g, base_b = self.color_sequence[color_index]
            
            # 计算呼吸效果
            breath_value += breath_direction * 10
            if breath_value >= 100:
                breath_value = 100
                breath_direction = -1
                # 切换到下一个颜色
                color_index = (color_index + 1) % len(self.color_sequence)
            elif breath_value <= 0:
                breath_value = 0
                breath_direction = 1
            
            # 应用呼吸效果
            factor = breath_value / 100.0
            r = int(base_r * factor)
            g = int(base_g * factor)
            b = int(base_b * factor)
            
            # 设置颜色
            self.set_light_color(r, g, b)
            
            # 控制速度
            rospy.sleep(0.05 / self.speed)
    
    def wave_chaser(self):
        """波浪走马灯效果"""
        rospy.loginfo("开始波浪走马灯效果...")
        position = 0
        
        while not rospy.is_shutdown() and self.running:
            # 计算波浪位置
            for i in range(len(self.color_sequence)):
                # 计算每个LED的亮度
                phase = (position + i) % len(self.color_sequence)
                intensity = (math.sin(phase * 2 * math.pi / len(self.color_sequence)) + 1) / 2
                
                # 获取基础颜色
                base_r, base_g, base_b = self.color_sequence[i]
                
                # 应用波浪效果
                r = int(base_r * intensity)
                g = int(base_g * intensity)
                b = int(base_b * intensity)
                
                # 设置颜色 (这里简化处理，实际可能需要分区域控制)
                if i == 0:  # 只设置主颜色，实际应用中可能需要更复杂的控制
                    self.set_light_color(r, g, b)
            
            # 更新位置
            position = (position + 1) % len(self.color_sequence)
            
            # 控制速度
            rospy.sleep(0.2 / self.speed)
    
    def stop(self):
        """停止走马灯"""
        self.running = False
        self.set_light_color(0, 0, 0)  # 关闭灯带
        rospy.loginfo("走马灯已停止")
    
    def run_demo(self):
        """运行演示序列"""
        try:
            # 彩虹走马灯 (10秒)
            self.rainbow_chaser()
            rospy.sleep(10)
            
            # 呼吸灯走马灯 (10秒)
            self.breathing_chaser()
            rospy.sleep(10)
            
            # 波浪走马灯 (10秒)
            self.wave_chaser()
            rospy.sleep(10)
            
        except rospy.ROSInterruptException:
            pass
        finally:
            self.stop()

if __name__ == "__main__":
    try:
        chaser = LightChaser()
        
        # 设置参数 (可以通过ROS参数服务器动态调整)
        chaser.speed = rospy.get_param("~speed", 0.5)
        chaser.brightness = rospy.get_param("~brightness", 255)
        
        # 运行演示
        chaser.run_demo()
        
    except Exception as e:
        rospy.logerr("走马灯控制器出错: %s", e)
