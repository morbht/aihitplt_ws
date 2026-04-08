#!/usr/bin/env python3
# coding:utf-8
import math
import numpy as np
import time
from common import *
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from dynamic_reconfigure.server import Server
from aihitplt_laser.cfg import laserAvoidPIDConfig

RAD2DEG = 180 / math.pi

class laserAvoid:
    def __init__(self):
        rospy.on_shutdown(self.cancel)
        self.r = rospy.Rate(10)  # 10Hz
        self.Moving = False
        self.switch = False
        self.Right_warning = 0
        self.Left_warning = 0
        self.front_warning = 0
        self.ros_ctrl = ROSCtrl()
        
        # 参数服务器
        Server(laserAvoidPIDConfig, self.dynamic_reconfigure_callback)
        
        # 默认参数
        self.linear = 0.2
        self.angular = 0.5
        self.ResponseDist = 0.8
        self.LaserAngle = 90  # 检测角度
        
        self.sub_laser = rospy.Subscriber('/scan', LaserScan, self.registerScan, queue_size=1)
        rospy.loginfo("Laser Avoidance node started!")

    def cancel(self):
        self.ros_ctrl.pub_vel.publish(Twist())
        self.ros_ctrl.cancel()
        self.sub_laser.unregister()
        rospy.loginfo("Shutting down this node.")

    def dynamic_reconfigure_callback(self, config, level):
        self.switch = config['switch']
        self.linear = config['linear']
        self.angular = config['angular']
        self.LaserAngle = config['LaserAngle']
        self.ResponseDist = config['ResponseDist']
        rospy.loginfo(f"Parameters updated: LaserAngle={self.LaserAngle}, ResponseDist={self.ResponseDist}")
        return config

    def registerScan(self, scan_data):
        if not isinstance(scan_data, LaserScan) or self.switch:
            if self.Moving:
                self.ros_ctrl.pub_vel.publish(Twist())
                self.Moving = False
            return

        ranges = np.array(scan_data.ranges)
        # 替换无限值为最大范围
        ranges[np.isinf(ranges)] = scan_data.range_max
        
        # 重置警告计数器
        self.Right_warning = 0
        self.Left_warning = 0
        self.front_warning = 0
        
        # 正确的角度区域划分
        half_angle = self.LaserAngle / 2
        
        for i in range(len(ranges)):
            angle = (scan_data.angle_min + scan_data.angle_increment * i) * RAD2DEG
            
            # 前方区域：-half_angle 到 +half_angle
            if -half_angle <= angle <= half_angle:
                if ranges[i] < self.ResponseDist:
                    self.front_warning += 1
            
            # 左侧区域：+half_angle 到 180 (或 +90 如果LaserAngle=90)
            elif half_angle < angle <= 180:
                if ranges[i] < self.ResponseDist:
                    self.Left_warning += 1
            
            # 右侧区域：-180 到 -half_angle (或 -90 如果LaserAngle=90)
            elif -180 <= angle < -half_angle:
                if ranges[i] < self.ResponseDist:
                    self.Right_warning += 1
        
        # 调试信息
        rospy.loginfo_throttle(1.0, f"Front: {self.front_warning}, Left: {self.Left_warning}, Right: {self.Right_warning}")
        
        self.avoid_obstacle()

    def avoid_obstacle(self):
        twist = Twist()
        self.Moving = True
        
        # 阈值设置
        front_threshold = 5    # 前方障碍物阈值
        side_threshold = 8     # 侧方障碍物阈值
        
        # 正确的避障逻辑：
        # 1. 前方有障碍物 - 根据两侧情况选择转向方向
        if self.front_warning > front_threshold:
            twist.linear.x = 0.1  # 减速但不停止
            
            # 哪边障碍物少就往哪边转
            if self.Left_warning < self.Right_warning:
                twist.angular.z = self.angular  # 向左转（左侧障碍少）
                rospy.loginfo("Front obstacle - Turning LEFT")
            else:
                twist.angular.z = -self.angular  # 向右转（右侧障碍少）
                rospy.loginfo("Front obstacle - Turning RIGHT")
                
        # 2. 左侧有障碍物 - 应该向右轻微转向避开
        elif self.Left_warning > side_threshold:
            twist.linear.x = self.linear * 0.7
            twist.angular.z = -self.angular * 0.6  # 向右转避开左侧障碍
            rospy.loginfo("Left obstacle - Steering RIGHT")
            
        # 3. 右侧有障碍物 - 应该向左轻微转向避开
        elif self.Right_warning > side_threshold:
            twist.linear.x = self.linear * 0.7
            twist.angular.z = self.angular * 0.6  # 向左转避开右侧障碍
            rospy.loginfo("Right obstacle - Steering LEFT")
            
        # 4. 无障碍物 - 直行
        else:
            twist.linear.x = self.linear
            twist.angular.z = 0
            rospy.loginfo("No obstacle - Moving FORWARD")

        self.ros_ctrl.pub_vel.publish(twist)

if __name__ == '__main__':
    rospy.init_node('laser_Avoidance', anonymous=False)
    tracker = laserAvoid()
    rospy.spin()
