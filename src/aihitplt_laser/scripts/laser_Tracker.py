#!/usr/bin/env python3
# coding:utf-8
import math
import numpy as np
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from dynamic_reconfigure.server import Server
from aihitplt_laser.cfg import laserTrackerPIDConfig

class LaserFollower:
    def __init__(self):
        rospy.init_node('laser_follower', anonymous=False)
        rospy.on_shutdown(self.cancel)
        
        # 初始化dynamic_reconfigure服务器
        self.dynamic_reconfigure_srv = Server(laserTrackerPIDConfig, self.dynamic_reconfigure_callback)
        
        # 默认参数设置（将被dynamic_reconfigure覆盖）
        self.detection_range = 3.0  # 探测范围：3米
        self.target_distance = 1.0  # 目标跟随距离：1米
        self.distance_tolerance = 0.1  # 距离容差：±0.1米
        self.detection_angle = 150  # 探测角度：前方150度（-75°到75°）
        self.angle_tolerance = 5    # 角度容差：±5度
        
        # PID参数
        self.linear_kp = 0.8
        self.linear_ki = 0.0
        self.linear_kd = 0.0
        self.angular_kp = 3.0
        self.angular_ki = 0.0
        self.angular_kd = 0.0
        
        self.laserAngle = 65
        self.ResponseDist = 1.0
        self.priorityAngle = 30
        self.tracker_switch = False  
        
        # 速度限制
        self.max_linear_speed = 0.3  # 最大线速度：0.3 m/s
        self.min_linear_speed = 0.1  # 最小线速度
        self.max_angular_speed = 0.8 # 最大角速度
        
        # PID控制器状态变量
        self.prev_distance_error = 0.0
        self.distance_integral = 0.0
        self.prev_angle_error = 0.0
        self.angle_integral = 0.0
        
        # 状态变量
        self.has_target = False
        self.target_distance_current = 0.0
        self.target_angle = 0.0
        self.state = "SEARCHING"  # 状态：SEARCHING, TURNING, FOLLOWING
        
        # 发布器和订阅器
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.laser_callback, queue_size=1)
        
        rospy.loginfo(f"探测范围: {self.detection_range}m, 目标距离: {self.target_distance}m")

    def dynamic_reconfigure_callback(self, config, level):
        """dynamic_reconfigure参数回调函数"""
        # PID参数
        self.linear_kp = config.lin_Kp
        self.linear_ki = config.lin_Ki
        self.linear_kd = config.lin_Kd
        self.angular_kp = config.ang_Kp
        self.angular_ki = config.ang_Ki
        self.angular_kd = config.ang_Kd
        
        # 其他参数
        self.laserAngle = config.laserAngle
        self.ResponseDist = config.ResponseDist
        self.priorityAngle = config.priorityAngle
        self.tracker_switch = config.switch
        
        # 更新相关参数
        self.detection_range = self.ResponseDist  # 使用ResponseDist作为探测范围
        self.target_distance = self.ResponseDist  # 目标距离设置为ResponseDist
        
        # 根据laserAngle设置探测角度
        self.detection_angle = self.laserAngle * 2  # 前方角度范围
        
        rospy.loginfo(f"探测角度: {self.laserAngle}°, 响应距离: {self.ResponseDist}m")
        
        return config

    def cancel(self):
        """关闭节点时停止机器人"""
        stop_cmd = Twist()
        self.cmd_vel_pub.publish(stop_cmd)
        rospy.loginfo("节点关闭，机器人已停止")

    def limit_velocity(self, linear_x, angular_z):
        """限制速度在安全范围内"""
        # 限制线速度
        linear_x = max(min(linear_x, self.max_linear_speed), -self.max_linear_speed)
        if abs(linear_x) < self.min_linear_speed and linear_x != 0:
            linear_x = math.copysign(self.min_linear_speed, linear_x)
        
        # 限制角速度
        angular_z = max(min(angular_z, self.max_angular_speed), -self.max_angular_speed)
        
        return linear_x, angular_z

    def calculate_pid(self, error, prev_error, integral, kp, ki, kd, dt=0.1):
        """计算PID输出"""
        # 积分项
        integral += error * dt
        integral = max(min(integral, 2.0), -2.0)  # 积分限幅
        
        # 微分项
        derivative = (error - prev_error) / dt if dt > 0 else 0
        
        # PID输出
        output = kp * error + ki * integral + kd * derivative
        
        return output, integral

    def laser_callback(self, scan_data):
        """激光雷达数据回调函数"""
        # 检查开关状态
        if not self.tracker_switch:
            # 如果开关关闭，停止机器人
            if self.state != "DISABLED":
                self.state = "DISABLED"
                stop_cmd = Twist()
                self.cmd_vel_pub.publish(stop_cmd)
            return
        
        if not isinstance(scan_data, LaserScan):
            return
        
        ranges = np.array(scan_data.ranges)
        
        # 将无效值（0或无穷大）设为探测范围外
        ranges[ranges == 0] = float('inf')
        ranges[ranges == float('inf')] = self.detection_range + 1
        
        min_distance = float('inf')
        min_angle = 0.0
        self.has_target = False
        
        # 扫描前方指定角度范围内的点
        for i in range(len(ranges)):
            # 计算当前激光点的角度（弧度转度）
            angle_rad = scan_data.angle_min + i * scan_data.angle_increment
            angle_deg = math.degrees(angle_rad)
            
            # 将角度标准化到 -180 到 180 范围
            if angle_deg > 180:
                angle_deg -= 360
            
            # 只处理前方指定角度范围内的点
            if abs(angle_deg) <= self.detection_angle / 2:
                distance = ranges[i]
                
                # 在探测范围内找到最近的目标
                if distance <= self.detection_range and distance < min_distance:
                    min_distance = distance
                    min_angle = angle_deg
                    self.has_target = True
        
        if self.has_target:
            self.target_distance_current = min_distance
            self.target_angle = min_angle
            
            # 状态转换逻辑
            if self.state == "SEARCHING" or self.state == "DISABLED":
                self.state = "TURNING"
                rospy.loginfo(f"发现目标，开始转向: 距离={min_distance:.2f}m, 角度={min_angle:.1f}°")
            elif self.state == "TURNING":
                if abs(min_angle) <= self.angle_tolerance:
                    self.state = "FOLLOWING"
                    rospy.loginfo("转向完成，开始跟随")
            elif self.state == "FOLLOWING":
                # 只有在角度偏差较大时才重新转向
                if abs(min_angle) > self.angle_tolerance * 2:
                    self.state = "TURNING"
                    rospy.loginfo(f"目标偏离，重新转向: 角度={min_angle:.1f}°")
        else:
            # 没有发现目标
            if self.state != "SEARCHING":
                self.state = "SEARCHING"
                rospy.loginfo("目标丢失，重新搜索")
        
        self.control_robot()

    def control_robot(self):
        """控制机器人运动 - 基于状态机"""
        if not self.tracker_switch:
            return  # 开关关闭，不控制机器人
        
        cmd_vel = Twist()
        linear_x = 0.0
        angular_z = 0.0
        
        # 获取当前时间用于PID计算
        current_time = rospy.Time.now().to_sec()
        
        if self.state == "SEARCHING":
            # 搜索状态：缓慢向前移动并小范围旋转搜索
            linear_x = self.min_linear_speed
            # 添加小幅旋转帮助搜索
            angular_z = 0.1 if rospy.Time.now().to_sec() % 4 < 2 else -0.1
            
            rospy.loginfo_throttle(3.0, "搜索模式：向前移动并搜索目标...")
            
        elif self.state == "TURNING" and self.has_target:
            # 转向状态：旋转对准目标
            linear_x = 0.0  # 转向时停止前进
            
            # 角度控制逻辑 - 使用完整的PID控制
            angle_error = math.radians(self.target_angle)  # 转换为弧度
            angular_z, self.angle_integral = self.calculate_pid(
                angle_error, 
                self.prev_angle_error, 
                self.angle_integral,
                self.angular_kp, 
                self.angular_ki, 
                self.angular_kd
            )
            self.prev_angle_error = angle_error
            
            # 如果角度误差很小，给予一个最小转向速度
            if abs(angular_z) < 0.05 and abs(angle_error) > 0.1:
                angular_z = 0.05 if angle_error > 0 else -0.05
            
            rospy.loginfo_throttle(1.0, 
                f"转向模式 - 目标角度: {self.target_angle:.1f}°, "
                f"角速度: {angular_z:.2f}rad/s, "
                f"Kp={self.angular_kp:.2f}, Ki={self.angular_ki:.2f}, Kd={self.angular_kd:.2f}")
                
        elif self.state == "FOLLOWING" and self.has_target:
            # 跟随状态：朝目标前进并保持距离
            distance_error = self.target_distance_current - self.target_distance
            
            # 添加距离容差控制
            if abs(distance_error) <= self.distance_tolerance:
                # 在目标距离范围内，停止前进
                linear_x = 0.0
                rospy.loginfo_throttle(2.0, "已达到目标距离，保持位置")
            else:
                # 使用完整的PID控制计算线速度
                linear_x, self.distance_integral = self.calculate_pid(
                    distance_error, 
                    self.prev_distance_error, 
                    self.distance_integral,
                    self.linear_kp, 
                    self.linear_ki, 
                    self.linear_kd
                )
                self.prev_distance_error = distance_error
            
            # 小角度微调 - 使用PID控制
            angle_error = math.radians(self.target_angle)
            if abs(self.target_angle) > self.angle_tolerance:
                angular_z, self.angle_integral = self.calculate_pid(
                    angle_error, 
                    self.prev_angle_error, 
                    self.angle_integral,
                    self.angular_kp * 0.5,  # 跟随时减小角度增益
                    self.angular_ki * 0.5,
                    self.angular_kd * 0.5
                )
                self.prev_angle_error = angle_error
            else:
                angular_z = 0.0
            
            rospy.loginfo_throttle(1.0, 
                f"跟随模式 - 距离: {self.target_distance_current:.2f}m (目标: {self.target_distance}m), "
                f"角度: {self.target_angle:.1f}°, "
                f"线速度: {linear_x:.2f}m/s, "
                f"角速度: {angular_z:.2f}rad/s")
        
        # 应用速度限制
        linear_x, angular_z = self.limit_velocity(linear_x, angular_z)
        
        cmd_vel.linear.x = linear_x
        cmd_vel.angular.z = angular_z
        
        self.cmd_vel_pub.publish(cmd_vel)

    def run(self):
        """运行节点"""
        rate = rospy.Rate(10)  # 10Hz
        while not rospy.is_shutdown():
            rate.sleep()

if __name__ == '__main__':
    try:
        follower = LaserFollower()
        follower.run()
    except rospy.ROSInterruptException:
        pass