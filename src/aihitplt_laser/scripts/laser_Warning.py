#!/usr/bin/env python3
# coding:utf-8
import math
import numpy as np
import rospy
from common import *
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from dynamic_reconfigure.server import Server
from aihitplt_laser.cfg import laserWarningPIDConfig

RAD2DEG = 180 / math.pi

class laserWarning:
    def __init__(self):
        rospy.on_shutdown(self.cancel)
        
        # 状态变量
        self.Moving = False
        self.switch = False
        self.Buzzer_state = False
        self.has_target = False
        
        # 控制参数
        self.detection_range = 3.0  # 探测范围：3米
        self.detection_angle = 150  # 探测角度：前方150度（-75°到75°）
        self.angle_tolerance = 5    # 角度容差：±5度
        
        # 速度限制
        self.max_angular_speed = 1.0
        
        # ROS组件
        self.ros_ctrl = ROSCtrl()
        self.ang_pid = SinglePID(2.0, 0.0, 0.5)
        
        # 动态参数服务器
        Server(laserWarningPIDConfig, self.dynamic_reconfigure_callback)
        
        # 发布者和订阅者
        self.pub_Buzzer = rospy.Publisher('/Buzzer', Bool, queue_size=1)
        self.sub_laser = rospy.Subscriber('/scan', LaserScan, self.registerScan, queue_size=1)
        
        # 目标信息
        self.target_distance = 0.0
        self.target_angle = 0.0
        
        rospy.loginfo("激光雷达警戒系统启动！")
        rospy.loginfo("探测范围: %.1fm, 探测角度: ±%d度", self.detection_range, self.detection_angle//2)

    def cancel(self):
        """安全关闭"""
        self.ros_ctrl.pub_vel.publish(Twist())
        self.ros_ctrl.cancel()
        self.sub_laser.unregister()
        rospy.loginfo('激光雷达警戒系统已关闭！')

    def registerScan(self, scan_data):
        """处理激光雷达数据"""
        if not isinstance(scan_data, LaserScan) or self.switch: 
            return
            
        ranges = np.array(scan_data.ranges)
        
        # 将无效值设为探测范围外
        ranges[ranges == 0] = float('inf')
        ranges[np.isinf(ranges)] = self.detection_range + 1
        ranges[np.isnan(ranges)] = self.detection_range + 1
        
        min_distance = float('inf')
        min_angle = 0.0
        self.has_target = False
        
        # 扫描前方指定角度范围内的点
        for i in range(len(ranges)):
            # 计算当前激光点的角度（弧度转度）
            angle_rad = scan_data.angle_min + i * scan_data.angle_increment
            angle_deg = angle_rad * RAD2DEG
            
            # 将角度标准化到 -180 到 180 范围
            if angle_deg > 180:
                angle_deg -= 360
            elif angle_deg < -180:
                angle_deg += 360
            
            # 只处理前方指定角度范围内的点
            if abs(angle_deg) <= self.detection_angle / 2:
                distance = ranges[i]
                
                # 在探测范围内找到最近的目标
                if distance <= self.detection_range and distance < min_distance:
                    min_distance = distance
                    min_angle = angle_deg
                    self.has_target = True
        
        if self.has_target:
            self.target_distance = min_distance
            self.target_angle = min_angle
            
            # 触发报警
            if not self.Buzzer_state:
                b = Bool()
                b.data = True
                self.pub_Buzzer.publish(b)
                self.Buzzer_state = True
                rospy.loginfo("发现目标！距离: %.2fm, 方向: %.1f°", min_distance, min_angle)
            
            # 控制机器人转向面对障碍物
            self.control_robot()
            
        else:
            # 没有发现目标，停止报警和运动
            if self.Buzzer_state:
                self.pub_Buzzer.publish(Bool())
                self.Buzzer_state = False
                rospy.loginfo("目标消失")
            
            if self.Moving:
                self.ros_ctrl.pub_vel.publish(Twist())
                self.Moving = False

    def control_robot(self):
        """控制机器人转向面对障碍物 - 修正的转向逻辑"""
        if not self.has_target:
            return
            
        velocity = Twist()
        
        # 修正的转向逻辑：直接转向目标角度
        # 当前机器人朝向为0度，目标角度为target_angle
        # 需要让机器人旋转target_angle度来面对障碍物
        
        # 使用PID控制，目标值是target_angle，当前值是0
        angular_z = self.ang_pid.pid_compute(self.target_angle, 0)
        
        # 限制角速度范围
        angular_z = max(min(angular_z, self.max_angular_speed), -self.max_angular_speed)
        
        # 如果角度误差很小，停止转动
        current_angle_error = abs(self.target_angle)
        if current_angle_error < self.angle_tolerance:
            angular_z = 0
            if self.Moving:
                self.Moving = False
        else:
            self.Moving = True
        
        velocity.angular.z = angular_z
        
        # 发布控制命令
        self.ros_ctrl.pub_vel.publish(velocity)
        
        rospy.loginfo_throttle(1.0, 
            "转向控制 - 目标角度: %.1f°, 当前误差: %.1f°, 角速度: %.2f rad/s", 
            self.target_angle, current_angle_error, angular_z)

    def dynamic_reconfigure_callback(self, config, level):
        """动态参数配置回调"""
        self.switch = config['switch']
        self.detection_range = config['ResponseDist']
        self.detection_angle = config['laserAngle']
        self.ang_pid.Set_pid(config['ang_Kp'], config['ang_Ki'], config['ang_Kd'])
        
        # 更新角度容差
        self.angle_tolerance = max(2, self.detection_angle / 30)
        
        rospy.loginfo("参数更新: 开关=%s, 警戒距离=%.1fm, 扫描角度=±%d度", 
                     "开启" if not self.switch else "关闭", 
                     self.detection_range, self.detection_angle//2)
        return config

if __name__ == '__main__':
    rospy.init_node('laser_Warning', anonymous=False)
    tracker = laserWarning()
    rospy.loginfo("激光雷达警戒节点启动完成")
    rospy.spin()
