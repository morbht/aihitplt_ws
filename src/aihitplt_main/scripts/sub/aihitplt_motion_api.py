#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import rospy
import math 
from math import *
from aihitplt_bringup.msg import Supersonic
import time
from sensor_msgs.msg import Range,LaserScan
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion, Twist, PoseWithCovariance, PoseWithCovarianceStamped  ,Twist ,Pose2D
from move_base_msgs.msg import *
import actionlib
import numpy as np  
import tf2_ros
from tf2_geometry_msgs import PointStamped, PoseStamped 
import tf.transformations
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from std_msgs.msg import String ,Bool ,Float32

class aihitplt_motion_api:
    def __init__(self):
        # 初始化数据变量
        self.ultrasonic_data = None
        self.user_button_status = False
        self.collide_status = False
        self.power_voltage = 0.0
        self.fall_status = 0.0

        # 订阅导航状态参数
        self.navigation_goal_id_ = ""
        self.navigation_status_ = None
        self.SLEEP_MS = 100

        self.approach_distance_ = 0.1
        self.approach_angular_vel_ = 0.1
        self.approach_linear_vel_ = 0.05
        self.approach_interval_sleep_ms_ = 500
        self.approach_pose_list_ = []

        self.warning_range = 0.16
        self.danger_range = 0.14

        self.SAFE = 0
        self.WARNING = 1
        self.DANGER = 2
        self.obstacle_status = self.SAFE

        self.leave_angular_vel_ = 0.1  
        self.leave_linear_vel_ = 0.05  

        # 初始化tf监听器  
        self.tf_buffer = tf2_ros.Buffer()  
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)  

        self.init_pose_pub_ = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10) 
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.odom_sub = rospy.Subscriber('odom', Odometry, self.odom_callback)
        self.goal_nav_pub = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=10)
        self.cancel_nav_pub = rospy.Publisher('move_base/cancel', GoalID, queue_size=10)
        
        # 订阅导航状态
        self.navigation_result_sub = rospy.Subscriber('move_base/status', GoalStatusArray, self.navigation_result_callback)
        # Subscriber to laser scan data
        self.laser_scan_sub = rospy.Subscriber("scan", LaserScan , self.laser_scan_callback)    
        # 超声波数据订阅
        self.ultrasonic_sub = rospy.Subscriber("/Distance", Supersonic, self.ultrasonic_callback)
        # 用户按钮状态订阅
        self.user_button_sub = rospy.Subscriber("/user_button", Bool, self.user_button_callback)
        # 防碰撞传感器订阅
        self.collide_sub = rospy.Subscriber("/collision_sensor", Bool, self.collide_callback)
        # 电池电量订阅
        self.power_voltage_sub = rospy.Subscriber("/PowerVoltage", Float32, self.power_voltage_callback)  
        # 跌落传感器订阅
        self.ir_distance_sub = rospy.Subscriber("/ir_distance", Float32, self.ir_distance_callback)
        
        
        self.rate = rospy.Rate(50)  # 50Hz
        rospy.loginfo("aihitplt_motion_api initialized")
        rospy.sleep(1)


    #设置底盘在RVIZ中的位置
    def rviz_pose_setting(self,args = [0,0,0,0,0,0,1.0]):
        rospy.sleep(1.0)  # 1秒 

        # 创建一个PoseWithCovariance消息  
        init_pose = PoseWithCovariance()  
        init_pose.pose.position.x = args[0]
        init_pose.pose.position.y = args[1]
        init_pose.pose.position.z = args[2] 
        init_pose.pose.orientation.x = args[3]
        init_pose.pose.orientation.y = args[4]
        init_pose.pose.orientation.z = args[5]
        init_pose.pose.orientation.w = args[6]  # 四元数表示(0, 0, 0, 1)对应于无旋转  

        # 设置协方差矩阵  
        cov = np.zeros(36)  
        cov[0] = 0.25  
        cov[7] = 0.25  
        cov[35] = 0.06853891945200942  
        init_pose.covariance = cov.tolist() 
        self.publish_rviz_pose(init_pose) 
        return True

    def publish_rviz_pose(self,init_pose):  
        # 创建并发布PoseWithCovarianceStamped消息  
        init_msg = PoseWithCovarianceStamped()  
        init_msg.header.frame_id = "map"  
        init_msg.header.stamp = rospy.Time.now()  
        init_msg.pose = init_pose  

        self.init_pose_pub_.publish(init_msg)  
        rospy.loginfo("RVIZ pose published.")  
        rospy.sleep(1.0)  # 等待3秒  

    #寻找AR码目标(AR码要带有ar_marker的字符串)
    def find_AR_marker(self,marker_name1,angular_vel1 = 0.5):
        """
        寻找AR标记目标
        Args:
            marker_name1: 标记名称
            angular_vel: 角速度 (rad/s)
        """
        pos = marker_name1.find("ar_marker")
        if pos != -1:
            marker_name = marker_name1[pos:]
            rospy.logwarn(f"Find Object : {marker_name}") 
            # time.sleep(1)
            linear_vel = 0.0
            angular_vel = angular_vel1
            approach_msg = Twist()
            approach_msg.linear.x = linear_vel
            approach_msg.angular.z = angular_vel
            self.publish_cmd_vel_msg(approach_msg)

            moving_time = 2 * pi / angular_vel
            sleep_time = 0.1  # seconds
            total_moving_index = int(moving_time / sleep_time)
            for ix in range(total_moving_index):
                success, pose = self.get_target_pose(marker_name)
                if success or rospy.is_shutdown(): 
                    rospy.loginfo("Success to find the target object")
                    approach_msg.linear.x = 0.0
                    approach_msg.angular.z = 0.0
                    self.publish_cmd_vel_msg(approach_msg)
                    return True

                time.sleep(sleep_time)

            approach_msg.linear.x = 0.0
            approach_msg.angular.z = 0.0
            self.publish_cmd_vel_msg(approach_msg)
            return False
        else:
            return False

    def publish_cmd_vel_msg(self, msg):
        rospy.loginfo("Publish Cmd_vel msg: linear.x = %f, angular.z = %f", msg.linear.x, msg.angular.z)
        self.cmd_vel_pub.publish(msg)

    def get_target_pose(self, target_name, base_frame="map"):  
        """
        Args:
            target_name: 目标坐标系名称，任何在TF树中存在的坐标系，例如："ar_marker_0"
            base_frame: 基准坐标系，默认为"base_link"map
        """
        try:  
            # 获取变换  
            trans = self.tf_buffer.lookup_transform(base_frame, target_name, rospy.Time(0))  

            # 创建Pose消息  
            target_pose = Pose()  
            target_pose.position.x = trans.transform.translation.x  
            target_pose.position.y = trans.transform.translation.y  
            target_pose.position.z = trans.transform.translation.z  

            target_pose.orientation.w = trans.transform.rotation.w  
            target_pose.orientation.x = trans.transform.rotation.x  
            target_pose.orientation.y = trans.transform.rotation.y  
            target_pose.orientation.z = trans.transform.rotation.z  

            return True, target_pose  

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:  
            rospy.logerr(f"Error looking up transform: {e}")  
            return False, None  

    def get_quaternion(self, roll, pitch, yaw):
        q = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
        quaternion = Quaternion()
        quaternion.x = q[0]
        quaternion.y = q[1]
        quaternion.z = q[2]
        quaternion.w = q[3]
        return quaternion

    def navigation_target(self, target, recovery_target=None, approach_distance=1.0):
        """
            带恢复机制的导航函数
            
            Args:
                target: 导航目标 (AR标记名称、坐标列表)
                recovery_target: 恢复点坐标 [x, y, z, qx, qy, qz, qw]，None表示使用默认导航起点
                approach_distance: 接近距离(米)，0表示直接导航到目标，>0表示在目标前方停靠
        """
        # 设置默认恢复点（导航起点）
        if recovery_target is None:
            recovery_target = [0, 0, 0, 0, 0, 0, 1]  # 默认导航起点
        max_retries  = 3
        recovery_strategies = [
            self._recovery_rotate_and_retry,    # 第一次失败：旋转搜索后重试
            self._recovery_go_to_recovery_point, # 第二次失败：前往恢复点后重试  
            self._recovery_wait_and_retry       # 第三次失败：等待后重试
        ]
        for attempt in range(max_retries):
            rospy.loginfo(f"Navigation attempt {attempt + 1}/{max_retries} to target: {target}")
            
            # 执行导航
            success = self.nav_to_target(target, approach_distance)
            
            if success:
                rospy.loginfo(f"Navigation to {target} succeeded!")
                return True
            else:
                rospy.logwarn(f"Navigation attempt {attempt + 1} failed")
                
                # 执行恢复策略
                if attempt < len(recovery_strategies):
                    recovery_success = recovery_strategies[attempt](target, recovery_target)
                    if not recovery_success:
                        rospy.logerr("Recovery strategy failed, stopping navigation")
                        break
                else:
                    rospy.logwarn("No more recovery strategies available")
                    break
                        
        rospy.logerr(f"Failed to navigate to {target} after {max_retries} attempts")
        return False

    def nav_to_target(self, target_name, approach_distance=0.0):
        """
        导航到指定目标
        
        Args:
            target_name: 目标名称 (AR标记、坐标或"nav_start")
            approach_distance: 接近距离，0表示直接到目标，>0表示在目标前方停靠
        """
        target_pose = None
        if isinstance(target_name,str):
            # go ar_marker
            pos = target_name.find("ar_marker")
            if pos != -1:
                marker_name = target_name[pos:]
                success, target_pose = self.get_target_pose(marker_name)
                if not success:
                    rospy.logwarn(f"Failed to find AR marker: {marker_name}")
                    return False

                # 根据approach_distance计算目标位置
                if approach_distance > 0:
                    # 在AR标记前方approach_distance米处停靠
                    target_pose = self._calculate_ar_marker_approach_pose(target_pose, approach_distance)
                    rospy.loginfo(f"Will approach AR marker from {approach_distance}m distance")
                else:
                    # 直接导航到AR标记位置
                    rospy.loginfo("Will navigate directly to AR marker position")
                
            else:
                rospy.logwarn(f"Unknown target name: {target_name}")
                return False
                
        # 处理坐标列表类型目标 [x, y, z, qx, qy, qz, qw]
        elif isinstance(target_name, (list, tuple)) and len(target_name) >= 7:
            target_pose = self._create_pose_from_list(target_name)
            
        else:
            rospy.logwarn(f"Invalid target type: {type(target_name)}")
            return False
        
        # 执行实际导航
        if target_pose is not None:
            return self.nav_to_target_go(target_pose)
        else:
            rospy.logwarn("Target pose is None, navigation aborted")
            return False

    def _create_pose_from_list(self, coord_list):
        """从坐标列表创建位姿对象"""
        if len(coord_list) < 7:
            rospy.logwarn(f"Coordinate list too short: {len(coord_list)} elements, need 7")
            return None
        
        pose = Pose()
        pose.position.x = coord_list[0]
        pose.position.y = coord_list[1]
        pose.position.z = coord_list[2]
        pose.orientation.x = coord_list[3]
        pose.orientation.y = coord_list[4]
        pose.orientation.z = coord_list[5]
        pose.orientation.w = coord_list[6]
        return pose

    def _calculate_ar_marker_approach_pose(self, marker_pose, approach_distance):
        """计算AR标记前方的接近位姿"""
        # 在标记前方approach_distance米处停靠
        offset = np.array([0, 0, approach_distance])
        
        # 获取标记的旋转矩阵
        target_orientation = [
            marker_pose.orientation.x, marker_pose.orientation.y,
            marker_pose.orientation.z, marker_pose.orientation.w
        ]
        object_position = np.array([
            marker_pose.position.x, marker_pose.position.y, marker_pose.position.z
        ])
        
        rotation_matrix = tf.transformations.quaternion_matrix(target_orientation)[:3, :3]
        global_offset = np.dot(rotation_matrix, offset)
        global_offset[2] = 0.0  # 保持水平
        
        # 计算目标位置
        target_position = object_position + global_offset
        
        # 创建目标位姿
        approach_pose = Pose()
        approach_pose.position.x = target_position[0]
        approach_pose.position.y = target_position[1]
        approach_pose.position.z = target_position[2]
        
        # 计算朝向：面向标记
        yaw = np.arctan2(-global_offset[1], -global_offset[0])
        approach_pose.orientation = self.get_quaternion(0.0, 0.0, yaw)
        
        rospy.loginfo(f"Approach pose calculated: {target_position[0]:.2f}, {target_position[1]:.2f}, yaw: {yaw:.2f}")
        return approach_pose

    def nav_to_target_go(self, target_pose):

        nav_msg = PoseStamped()
        nav_msg.header.stamp = rospy.Time.now()
        nav_msg.header.frame_id = "map"
        nav_msg.pose = target_pose

        self.publish_goal_nav_msg(nav_msg)
        rospy.loginfo("Navigation goal published")

        # 等待导航目标被接受
        wait_start_time = rospy.Time.now().to_sec()
        while (self.navigation_status_ != GoalStatus.ACTIVE and 
            not rospy.is_shutdown()):
            # 添加超时机制
            if rospy.Time.now().to_sec() - wait_start_time > 10.0:  # 10秒超时
                rospy.logwarn("Navigation goal acceptance timeout")
                return False
            rospy.sleep(rospy.Duration(self.SLEEP_MS / 1000.0))

        rospy.loginfo(f"Navigation goal accepted, status: {self.navigation_status_}")
        # 监控导航过程
        navigation_start_time = rospy.Time.now().to_sec()
        while (self.navigation_status_ == GoalStatus.ACTIVE and 
            not rospy.is_shutdown()):
            
            # 导航超时检查（5分钟）
            if rospy.Time.now().to_sec() - navigation_start_time > 120.0:
                rospy.logwarn("Navigation timeout (2 minutes), canceling")
                self.cancel_nav()
                return False
                
            rospy.sleep(rospy.Duration(self.SLEEP_MS / 1000.0))

        # 检查最终导航状态
        if self.navigation_status_ == GoalStatus.SUCCEEDED:
            rospy.loginfo("Navigation completed successfully")
            return True
        else:
            rospy.logwarn(f"Navigation finished with status: {self.navigation_status_}")
            return False

    # ========== 恢复策略函数 ==========

    def _recovery_rotate_and_retry(self, target, recovery_target):
        """恢复策略1：旋转搜索后重试"""
        rospy.loginfo("Recovery: Rotating to search for target")
        
        # 先停止当前可能的活动
        self.cancel_nav()
        rospy.sleep(1.0)
        
        # 旋转搜索目标（如果是AR标记）
        if isinstance(target, str) and "ar_marker" in target:
            search_success = self.find_AR_marker(target)
            if search_success:
                rospy.loginfo("Target found during recovery rotation")
                return True
            else:
                rospy.logwarn("Target not found during recovery rotation")
        
        return True  # 仍然尝试重试导航

    def _recovery_go_to_recovery_point(self, target, recovery_target):
        """恢复策略2：前往恢复点后重试"""
        rospy.loginfo(f"Recovery: Going to recovery point: {recovery_target}")
        # 停止当前导航
        self.cancel_nav()
        rospy.sleep(1.0)
        # 导航到恢复点坐标
        recovery_success = self.nav_to_target(recovery_target)
        if recovery_success:
            rospy.loginfo(f"Successfully reached recovery point")
            rospy.sleep(2.0)  # 等待稳定
            return True
        else:
            rospy.logwarn("⚠️ Failed to reach recovery point, trying alternative recovery")
            # 即使恢复点导航失败，也尝试其他方式
            # 比如原地旋转重新定位
            rospy.loginfo("Attempting reorientation...")
            self._perform_reorientation()
            return True  # 仍然继续重试，不放弃

    def _recovery_wait_and_retry(self, target, recovery_target):
        """恢复策略3：等待环境变化后重试"""
        rospy.loginfo("Recovery: Waiting for environment changes")
        # 停止当前导航
        self.cancel_nav()
        rospy.sleep(1.0)
        # 等待环境变化
        wait_time = 10
        rospy.loginfo(f"Waiting {wait_time} seconds for environment changes...")
        for i in range(wait_time):
            remaining = wait_time - i
            rospy.loginfo_throttle(2.0, f"Waiting... {remaining}s remaining")
            rospy.sleep(1.0)
        
        # 等待后尝试重新定位
        rospy.loginfo("Attempting reorientation after waiting...")
        self._perform_reorientation()
        
        rospy.loginfo("Proceeding with final navigation retry")
        return True

    def _perform_reorientation(self):
        """执行重新定位操作"""
        rospy.loginfo("Performing reorientation...")
        
        # 执行360度旋转扫描环境
        try:
            # 简单的旋转扫描
            twist = Twist()
            twist.angular.z = 0.3  # 缓慢旋转
            scan_duration = 6.0  # 大约旋转180度
            
            start_time = rospy.Time.now().to_sec()
            while (rospy.Time.now().to_sec() - start_time < scan_duration and 
                not rospy.is_shutdown()):
                self.cmd_vel_pub.publish(twist)
                rospy.sleep(0.1)
            
            # 停止旋转
            stop_twist = Twist()
            for _ in range(3):
                self.cmd_vel_pub.publish(stop_twist)
                rospy.sleep(0.1)
                
            rospy.loginfo("Reorientation completed")
            
        except Exception as e:
            rospy.logwarn(f"Reorientation interrupted: {e}")

    def publish_goal_nav_msg(self, goal_msg):
        rospy.loginfo("Publish Nav msg")
        rospy.loginfo(goal_msg)
        self.goal_nav_pub.publish(goal_msg)

    def cancel_nav(self):
        cancel_msg = GoalID()
        cancel_msg.id = self.navigation_goal_id_
        self.cancel_nav_pub.publish(cancel_msg)
        rospy.loginfo("Nav goal is canceled.")

    def navigation_result_callback(self, msg):
        if not msg.status_list:
            return
        # 获取最后一个导航目标的状态
        last_status = msg.status_list[-1]
        self.navigation_goal_id_ = last_status.goal_id.id
        self.navigation_status_ = last_status.status

    # def check_distance(self, from_frame, to_frame, max_distance):
    #     if isinstance(to_frame, str):
    #         result, target_pose = self.get_target_pose(to_frame)
    #         if not result:
    #             return False
    #     else:
    #         target_pose = to_frame

    #     result, present_pose = self.get_target_pose(from_frame)
    #     if not result:
    #         return False

    #     diff_x = target_pose.position.x - present_pose.position.x
    #     diff_y = target_pose.position.y - present_pose.position.y

    #     distance = sqrt(diff_x ** 2 + diff_y ** 2)

    #     return distance <= max_distance


    def approach(self, target_name, repeat_number):
        """
        接近目标方法
        Args:(
            target_name (str): 目标名称，例如"ar_marker_0"
            repeat_number (int): 重复尝试次数
        """
        
        for ix in range(repeat_number):
            approach_result = self.approach_target(target_name, repeat_number, ix + 1)
            
            if not approach_result:
                self.leave_target("leave_back_inter")
                rospy.sleep(rospy.Duration(self.SLEEP_MS * 10/ 1000.0))

                approach_result = self.approach_target(target_name, repeat_number, ix + 1)
                
                if not approach_result:
                    self.leave_target("leave_back_inter")
                    rospy.logerr("Failed to approach")
            
            else:
                return True
            
            return False

    def approach_target(self, target_name, total_count, present_count):
        # go ar_marker
        pos = target_name.find("ar_marker")
        if pos != -1:
            marker_name = target_name[pos:]
            rospy.loginfo(f"=== 开始接近AR码: {marker_name} ===")
            
            target_pose = Pose()
            present_pose = Pose()
            base_frame_id = "rgb_camera_link"
            
            # 1. 获取AR码位置
            rospy.loginfo(f"尝试获取AR码位置: {marker_name} -> map")
            result, target_pose = self.get_target_pose(marker_name, "map")
            
            if not result:
                rospy.logerr(f"✗ 无法找到AR码: {marker_name}")
                rospy.loginfo("可能原因:")
                rospy.loginfo("  1. AR码不在视野中")
                rospy.loginfo("  2. TF变换未发布")
                rospy.loginfo("  3. 坐标系名称错误")
                return False
            else:
                rospy.loginfo(f"✓ 找到AR码位置:")
                rospy.loginfo(f"  坐标: ({target_pose.position.x:.3f}, {target_pose.position.y:.3f}, {target_pose.position.z:.3f})")
            
            # 2. 获取机器人当前位置
            rospy.loginfo(f"尝试获取机器人位置: {base_frame_id} -> map")
            result, present_pose = self.get_target_pose(base_frame_id, "map")
            
            if not result:
                rospy.logerr(f"✗ 无法获取机器人位置")
                return False
            else:
                rospy.loginfo(f"✓ 机器人当前位置:")
                rospy.loginfo(f"  坐标: ({present_pose.position.x:.3f}, {present_pose.position.y:.3f}, {present_pose.position.z:.3f})")
            
            # 3. 计算距离
            dx = target_pose.position.x - present_pose.position.x
            dy = target_pose.position.y - present_pose.position.y
            distance = math.sqrt(dx**2 + dy**2)
            rospy.loginfo(f"与AR码距离: {distance:.3f}m")
            
            final_offset = self.approach_distance_ + (total_count - present_count) * 0.05
            via_offset = final_offset + 0.05
            
            rospy.loginfo(f"计算参数:")
            rospy.loginfo(f"  final_offset: {final_offset:.3f}")
            rospy.loginfo(f"  via_offset: {via_offset:.3f}")
            rospy.loginfo(f"  total_count: {total_count}")
            rospy.loginfo(f"  present_count: {present_count}")

            offset = np.array([0, 0, via_offset])

            target_orientation = [target_pose.orientation.x,
                                    target_pose.orientation.y,
                                    target_pose.orientation.z,
                                    target_pose.orientation.w]
            object_position = np.array([target_pose.position.x,
                                        target_pose.position.y,
                                        target_pose.position.z])
            rotation_matrix = tf.transformations.quaternion_matrix(target_orientation)[:3, :3]
            global_offset = np.dot(rotation_matrix, offset)
            global_offset[2] = 0.0
            target_position = object_position + global_offset

            target_pose.position.x = target_position[0]
            target_pose.position.y = target_position[1]
            target_pose.position.z = target_position[2]

            target_yaw = np.arctan2(-global_offset[1], -global_offset[0])

            present_pose_2d = Pose2D()
            present_pose_2d.x = present_pose.position.x
            present_pose_2d.y = present_pose.position.y
            p_roll, p_pitch, p_yaw = self.get_euler_angle(present_pose.orientation)
            present_pose_2d.theta = p_yaw

            target_pose_2d = Pose2D()
            target_pose_2d.x = target_pose.position.x
            target_pose_2d.y = target_pose.position.y
            target_pose_2d.theta = target_yaw

            rospy.loginfo(f"移动参数:")
            rospy.loginfo(f"  起点: ({present_pose_2d.x:.3f}, {present_pose_2d.y:.3f}, θ={math.degrees(present_pose_2d.theta):.1f}°)")
            rospy.loginfo(f"  目标点: ({target_pose_2d.x:.3f}, {target_pose_2d.y:.3f}, θ={math.degrees(target_pose_2d.theta):.1f}°)")

            self.approach_pose_list_.clear()
            self.approach_pose_list_.append(present_pose_2d)
            self.approach_pose_list_.append(target_pose_2d)

            is_final = (total_count == present_count)
            rospy.loginfo(f"是否为最终接近: {is_final}")
            
            success = self.approach_target_thread(present_pose_2d, target_pose_2d, is_final)
            
            if success:
                rospy.loginfo(f"✓ 接近AR码成功")
            else:
                rospy.logerr(f"✗ 接近AR码失败")
                
            return success
            
        rospy.logerr(f"目标不是AR码: {target_name}")
        return False

    def approach_target_thread(self, present_pose, target_pose, is_final_approach):
        rospy.logwarn(f"present : {present_pose.x}, {present_pose.y} | {present_pose.theta}")
        rospy.logwarn(f"target : {target_pose.x}, {target_pose.y} | {target_pose.theta}")

        diff_x = target_pose.x - present_pose.x
        diff_y = target_pose.y - present_pose.y

        distance = np.sqrt(diff_x**2 + diff_y**2)
        yaw_1 = np.arctan2(diff_y, diff_x) - present_pose.theta
        yaw_2 = target_pose.theta - np.arctan2(diff_y, diff_x)

        rospy.loginfo(f"yaw_1 : {yaw_1 * 180 / np.pi}, distance : {distance}, yaw_2 : {yaw_2 * 180 / np.pi}")

        # turn to yaw_1
        if abs(yaw_1) > 0.01:  
            # 或者直接使用角速度控制
            approach_msg = Twist()
            approach_msg.angular.z = self.approach_angular_vel_ if yaw_1 > 0 else -self.approach_angular_vel_
            self.publish_cmd_vel_msg(approach_msg)
            
            # 旋转完成后停止
            rospy.sleep(abs(yaw_1) / abs(approach_msg.angular.z))
            approach_msg.angular.z = 0.0
            self.publish_cmd_vel_msg(approach_msg)
            rospy.sleep(0.5)  # 等待稳定

        # go to via - 使用 linear_move 而不是时间控制
        if distance > 0.01:  # 增加容差
            # 计算移动方向
            move_distance = distance
            
            # 使用 linear_move 方法，提供里程计反馈
            success = self.linear_move(move_distance, speed=self.approach_linear_vel_)
            if not success:
                rospy.logerr("直线移动失败")
                return False
            
            rospy.sleep(0.5)  # 等待稳定

        # turn to target theta
        if abs(yaw_2) > 0.01:  # 增加容差
            approach_msg = Twist()
            approach_msg.angular.z = self.approach_angular_vel_ if yaw_2 > 0 else -self.approach_angular_vel_
            self.publish_cmd_vel_msg(approach_msg)
            
            rospy.sleep(abs(yaw_2) / abs(approach_msg.angular.z))
            approach_msg.angular.z = 0.0
            self.publish_cmd_vel_msg(approach_msg)
            rospy.sleep(0.5)  # 等待稳定

        if is_final_approach:
            final_approach_distance = 0.05
            
            # 使用 linear_move 进行最终接近
            success = self.linear_move(final_approach_distance, speed=self.approach_linear_vel_)
            if not success:
                rospy.logerr("最终接近失败")
                return False

        return True


    def leave_target(self, command):
        """
        离开指定目标（AR标记等）
        Args:
            command (str,float,int): 例如“back”,0.3等。
        """
        
        leave_back_range = 0.25
        leave_back_inter_range = 0.15

        if isinstance(command,str):
            if "back" in command:
                pose_1 = Pose2D()
                pose_2 = Pose2D()

                if "inter" in command:
                    pose_2.x = -leave_back_inter_range
                else:
                    pose_2.x = -leave_back_range

                return self.leave_target_thread(pose_1, pose_2)
            else:
                if len(self.approach_pose_list_) != 2:
                    rospy.logerr("No approach data!!!")
                    return False
                return self.leave_target_thread(self.approach_pose_list_[1], self.approach_pose_list_[0])
        
        elif isinstance(command,float) or isinstance(command,int):
            pose_1 = Pose2D()
            pose_2 = Pose2D()
            pose_2.x = -command
            return self.leave_target_thread(pose_1, pose_2)
        
        else:
            return False


    def leave_target_thread(self, present_pose, target_pose):
        """
        使用里程计反馈确保移动准确距离
        """
        rospy.loginfo(f"开始后退: present=({present_pose.x:.2f},{present_pose.y:.2f}), "
                    f"target=({target_pose.x:.2f},{target_pose.y:.2f})")
        
        # 计算目标距离
        target_distance = math.sqrt(
            (target_pose.x - present_pose.x)**2 + 
            (target_pose.y - present_pose.y)**2
        )
        
        rospy.loginfo(f"目标后退距离: {target_distance:.3f}m")
        
        # 如果是直接后退（没有y分量）
        if abs(target_pose.y - present_pose.y) < 0.001:
            # 直接使用 linear_move 方法（使用里程计反馈）
            return self.linear_move(-target_distance, speed=self.leave_linear_vel_)
        
        # 如果有y分量，需要旋转和移动
        # 1. 计算需要旋转的角度
        angle_to_target = math.atan2(
            target_pose.y - present_pose.y,
            target_pose.x - present_pose.x
        )
        
        # 旋转到目标方向
        if abs(angle_to_target) > 0.05:
            rospy.loginfo(f"旋转 {math.degrees(angle_to_target):.1f}°")
            self.rotate_to_angle(math.degrees(angle_to_target))
            rospy.sleep(0.5)
        
        # 2. 移动到目标位置
        rospy.loginfo(f"移动 {target_distance:.3f}m")
        success = self.linear_move(target_distance, speed=self.leave_linear_vel_)
        
        # 3. 旋转到目标朝向
        if abs(target_pose.theta - present_pose.theta) > 0.05:
            rotate_angle = target_pose.theta - present_pose.theta
            rotate_angle = ((rotate_angle + math.pi) % (2 * math.pi)) - math.pi
            rospy.loginfo(f"最终旋转 {math.degrees(rotate_angle):.1f}°")
            self.rotate_to_angle(math.degrees(rotate_angle))
        
        return success

    def laser_scan_callback(self, msg):
        try:

            angle_min = msg.angle_min
            angle_max = msg.angle_max
            angle_increment = msg.angle_increment
            ranges = np.array(msg.ranges)
            
            # Check front_left (45 deg ~ 60 deg)
            start_index = int(((45.0 * np.pi / 180.0) - angle_min) / angle_increment)
            end_index = int(((60.0 * np.pi / 180.0) - angle_min) / angle_increment)
            
            # Filter out invalid ranges
            valid_ranges = ranges[start_index:end_index+1]
            valid_ranges = valid_ranges[(valid_ranges >= msg.range_min) & (valid_ranges <= msg.range_max)]
            
            if len(valid_ranges) > 0:
                distance_average = np.mean(valid_ranges)
                if distance_average < self.danger_range:
                    self.obstacle_status = self.DANGER
                    rospy.logerr(f"laser scan [front_left]: [{start_index} - {end_index}] : {distance_average}, count: {len(valid_ranges)}")
                elif distance_average < self.warning_range:
                    self.obstacle_status = self.WARNING
                    rospy.logwarn(f"laser scan [front_left]: [{start_index} - {end_index}] : {distance_average}, count: {len(valid_ranges)}")
                else:
                    self.obstacle_status = self.SAFE
                    # rospy.loginfo(f"laser scan [front_left]: [{start_index} - {end_index}] : {distance_average}, count: {len(valid_ranges)}")

            # Check front_right (300 deg ~ 315 deg)
            start_index = int(((300.0 * np.pi / 180.0) - angle_min) / angle_increment)
            end_index = int(((315.0 * np.pi / 180.0) - angle_min) / angle_increment)
            
            # Filter out invalid ranges
            valid_ranges = ranges[start_index:end_index+1]
            valid_ranges = valid_ranges[(valid_ranges >= msg.range_min) & (valid_ranges <= msg.range_max)]
            
            if len(valid_ranges) > 0:
                distance_average = np.mean(valid_ranges)
                if distance_average < self.danger_range:
                    self.obstacle_status = self.DANGER
                    rospy.logerr(f"laser scan [front_right]: [{start_index} - {end_index}] : {distance_average}, count: {len(valid_ranges)}")
                elif distance_average < self.warning_range:
                    self.obstacle_status = max(self.obstacle_status, self.WARNING)  # Update status based on max severity
                    rospy.logwarn(f"laser scan [front_right]: [{start_index} - {end_index}] : {distance_average}, count: {len(valid_ranges)}")
                else:
                    self.obstacle_status = max(self.obstacle_status, self.SAFE)
                    # rospy.loginfo(f"laser scan [front_right]: [{start_index} - {end_index}] : {distance_average}, count: {len(valid_ranges)}")

        except Exception as e:
            rospy.logerr(f"Error in checking obstacle: {str(e)}")


    def get_euler_angle(self,quaternion):
        # roll (x-axis rotation)
        sinr_cosp = 2.0 * (quaternion.w * quaternion.x + quaternion.y * quaternion.z)
        cosr_cosp = 1.0 - 2.0 * (quaternion.x * quaternion.x + quaternion.y * quaternion.y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        # pitch (y-axis rotation)
        sinp = 2.0 * (quaternion.w * quaternion.y - quaternion.z * quaternion.x)
        if np.abs(sinp) >= 1:
            pitch = np.sign(sinp) * np.pi / 2  # use 90 degrees if out of range
        else:
            pitch = np.arcsin(sinp)

        # yaw (z-axis rotation)
        siny_cosp = 2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
        cosy_cosp = 1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

    def turn_to_target(self, target_pose_list):
        """
        使用rotate_to_angle的改进版本
        """
        if len(target_pose_list) < 2:
            rospy.logerr("目标点坐标至少需要x,y")
            return False
        
        rospy.loginfo(f"转向目标点: ({target_pose_list[0]:.3f}, {target_pose_list[1]:.3f})")
        
        # 获取机器人当前位置和朝向
        success, robot_pose = self.get_target_pose("base_footprint", "map")
        if not success:
            rospy.logerr("无法获取机器人位置")
            return False
        
        # 获取当前朝向
        roll, pitch, current_yaw = self.get_euler_angle(robot_pose.orientation)
        
        # 计算目标方向
        dx = target_pose_list[0] - robot_pose.position.x
        dy = target_pose_list[1] - robot_pose.position.y
        
        # 计算距离
        distance = math.sqrt(dx**2 + dy**2)
        if distance < 0.001:
            rospy.loginfo("目标点就在当前位置")
            return True
        
        # 计算目标角度
        target_yaw = math.atan2(dy, dx)
        
        # 计算需要旋转的角度
        rotate_angle_rad = target_yaw - current_yaw
        rotate_angle_rad = math.atan2(math.sin(rotate_angle_rad), math.cos(rotate_angle_rad))
        rotate_angle_deg = math.degrees(rotate_angle_rad)
        
        rospy.loginfo(f"需要旋转: {rotate_angle_deg:.1f}°")
        
        # 使用现有的精确旋转方法
        return self.rotate_to_angle(rotate_angle_deg, angular_speed=0.3)

    def turn_to_target_thread(self, yaw):
        vel_offset = self.leave_angular_vel_ + 0.2
        if yaw == 0:
            return True

        approach_msg = Twist()
        approach_msg.angular.z = vel_offset if yaw > 0 else -vel_offset
        self.publish_cmd_vel_msg(approach_msg)

        moving_time = int(abs(yaw) * 1000 / abs(approach_msg.angular.z))
        rospy.sleep(moving_time / 1000.0)

        approach_msg.angular.z = 0.0
        self.publish_cmd_vel_msg(approach_msg)
        return True


    '''
        输入目标点，到达指定的目标点的接口函数
        输入参数为：map坐标系下的小车的期望位置target_x,target_y,target_z以及期望姿态target_qx,target_qy,target_qz,target_qw
    '''
    def Target_point(self,target_pose_list):
        move_base = actionlib.SimpleActionClient("/move_base", MoveBaseAction)  

        rospy.loginfo("Waiting for move_base action server...")  

        # 等待连接服务器，5s等待时间限制 
        while move_base.wait_for_server(rospy.Duration(5.0)) == 0:
            rospy.loginfo("Connected to move base server")  

        # 设定目标点  
        target_1 = Pose(Point(target_pose_list[0], target_pose_list[1], target_pose_list[2]), Quaternion(target_pose_list[3], target_pose_list[4],target_pose_list[5],target_pose_list[6]))  
        goal_1 = MoveBaseGoal()  
        goal_1.target_pose.pose = target_1  
        goal_1.target_pose.header.frame_id = 'map'  
        goal_1.target_pose.header.stamp = rospy.Time.now()  

        rospy.loginfo("Going to: " + str(target_1))  

        # 向目标进发  
        move_base.send_goal(goal_1)  

        # 五分钟时间限制  
        finished_within_time = move_base.wait_for_result(rospy.Duration(120))   

        # 查看是否成功到达  
        if not finished_within_time:  
            move_base.cancel_goal()  
            rospy.loginfo("Timed out achieving goal")  
            return False
        else:  
            state = move_base.get_state()  
            if state == GoalStatus.SUCCEEDED:  
                rospy.loginfo("目标点：(%.4f,%.4f,%.4f)成功到达！",target_pose_list[0], target_pose_list[1], target_pose_list[2])
                return True
            else:  
                rospy.loginfo("目标点：(%.4f,%.4f,%.4f)无法到达！",target_pose_list[0], target_pose_list[1], target_pose_list[2])
                return False

    def linear_move(self, distance, speed=0.1):
        """
        基于里程计反馈的精确直线移动
        
        :param distance: 移动距离（米），正值为向前，负值为向后
        :param speed: 移动速度（米/秒），默认为0.1 m/s
        :return: 移动是否成功完成
        """
        # 等待里程计数据
        if self.current_position is None:
            rospy.loginfo("Waiting for odometry data...")
            wait_start = rospy.Time.now()
            while (not rospy.is_shutdown() and 
                self.current_position is None and
                (rospy.Time.now() - wait_start).to_sec() < 5.0):
                self.rate.sleep()
            
            if self.current_position is None:
                rospy.logerr("Timeout waiting for odometry data")
                return False

        # 记录起始位置
        start_x = self.current_position.x
        start_y = self.current_position.y
        rospy.loginfo(f"Starting linear move: {distance:.3f}m at {speed:.2f}m/s")

        # 设置运动指令
        twist = Twist()
        twist.linear.x = speed if distance >= 0 else -speed
        actual_distance = abs(distance)

        try:
            # 执行移动
            while not rospy.is_shutdown():
                # 计算已移动距离
                current_x = self.current_position.x
                current_y = self.current_position.y
                moved_distance = math.sqrt(
                    (current_x - start_x) ** 2 + 
                    (current_y - start_y) ** 2
                )

                # 日志输出（限频）
                rospy.loginfo_throttle(1.0, 
                    f"Linear move progress: {moved_distance:.3f}/{actual_distance:.3f}m"
                )

                # 检查是否到达目标
                if moved_distance >= actual_distance:
                    rospy.loginfo(f"Linear move completed: {moved_distance:.3f}m")
                    break

                # 发布控制指令
                self.cmd_vel_pub.publish(twist)
                self.rate.sleep()

        except Exception as e:
            rospy.logerr(f"Error during linear move: {e}")
            # 确保在异常时也停止机器人
            stop_twist = Twist()
            stop_twist.linear.x = 0.0
            for _ in range(3):
                self.cmd_vel_pub.publish(stop_twist)
                self.rate.sleep()
            return False

        # 正常停止机器人
        stop_twist = Twist()
        stop_twist.linear.x = 0.0
        for _ in range(3):
            self.cmd_vel_pub.publish(stop_twist)
            self.rate.sleep()
        
        rospy.loginfo("Linear move finished successfully")
        return True

    def rotate_to_angle(self, target_angle, angular_speed=0.3):
        """
        精确角度移动
        Args:
            target_angle: 目标角度设置
            angular_speed (float, 可选): 角速度设置，默认为0.3

        """
        if self.current_yaw is None:
            rospy.logerr("No orientation data")
            return False
        
        self.target_angle = target_angle
        # 记录起始朝向
        start_yaw = self.current_yaw
        start_time = time.time()
        
        # 计算目标绝对角度
        target_abs_rad = start_yaw + math.radians(target_angle)
        target_abs_rad = math.atan2(math.sin(target_abs_rad), math.cos(target_abs_rad))
        
        rospy.loginfo(f"旋转 {target_angle:.1f}°, 速度 {angular_speed:.2f} rad/s")
        
        # 控制参数
        deceleration_angle_deg = 45.0  # 提前45度开始减速
        min_speed = 0.05  # 最小角速度
        target_tolerance_deg = 2.0  # 目标容差
        
        # 状态跟踪
        last_yaw = start_yaw
        last_time = start_time
        angular_velocity = 0.0  # 估计的角速度
        
        twist = Twist()
        twist.angular.z = angular_speed if target_angle >= 0 else -angular_speed
        
        try:
            while not rospy.is_shutdown():
                current_yaw = self.current_yaw
                current_time = time.time()
                
                # 计算角速度估计
                dt = current_time - last_time
                if dt > 0.001:
                    delta_yaw = current_yaw - last_yaw
                    # 处理360度边界
                    if delta_yaw > math.pi:
                        delta_yaw -= 2 * math.pi
                    elif delta_yaw < -math.pi:
                        delta_yaw += 2 * math.pi
                    angular_velocity = delta_yaw / dt
                
                # 计算角度误差
                error = target_abs_rad - current_yaw
                if error > math.pi:
                    error -= 2 * math.pi
                elif error < -math.pi:
                    error += 2 * math.pi
                
                error_deg = math.degrees(abs(error))
                
                # 计算剩余旋转距离（考虑惯性）
                # 根据当前角速度估计制动距离
                if abs(angular_velocity) > 0.001:
                    # 假设最大减速度为 angular_speed * 2
                    max_deceleration = angular_speed * 2
                    braking_distance = (angular_velocity ** 2) / (2 * max_deceleration)
                    braking_distance_deg = math.degrees(braking_distance)
                    
                    # 预测停止位置
                    predicted_stop = error_deg - braking_distance_deg
                else:
                    predicted_stop = error_deg
                
                rospy.loginfo_throttle(0.3,
                    f"误差: {error_deg:.1f}°, "
                    f"角速度: {math.degrees(angular_velocity):.1f}°/s, "
                    f"目标角度: {self.target_angle:.1f}°"
                )
                
                # 检查是否应该停止
                if error_deg <= target_tolerance_deg:
                    rospy.loginfo(f"到达目标! 最终误差: {error_deg:.1f}°")
                    break
                
                # 高级减速逻辑
                if error_deg < deceleration_angle_deg:
                    # 更早开始减速，更平滑
                    speed_factor = (error_deg / deceleration_angle_deg) ** 0.5  # 使用平方根更平滑
                    
                    # 根据预测调整
                    if predicted_stop < 0:  # 预测会超调
                        speed_factor *= 0.3  # 更激进地减速
                    
                    current_speed = angular_speed * speed_factor
                    
                    # 确保最小速度
                    if current_speed < min_speed:
                        current_speed = min_speed
                    
                    twist.angular.z = current_speed * (1 if target_angle >= 0 else -1)
                
                # 如果预测会超调很多，提前停止
                if predicted_stop < -5.0:  # 预测超调超过5度
                    rospy.loginfo("预测会超调，提前停止")
                    break
                
                # 更新状态
                last_yaw = current_yaw
                last_time = current_time
                
                # 发布控制
                self.cmd_vel_pub.publish(twist)
                self.rate.sleep()
                
        except Exception as e:
            rospy.logerr(f"旋转错误: {e}")
        
        # 强制停止
        self._force_stop()
        
        # 最终验证
        final_error = math.degrees(target_abs_rad - self.current_yaw)
        final_error = ((final_error + 180) % 360) - 180
        
        rospy.loginfo(f"旋转完成: 实际误差 {final_error:.1f}°")
        
        return abs(final_error) <= 5.0

    def _force_stop(self):
        """强制停止，使用反向扭矩"""
        # 先正常停止
        twist = Twist()
        twist.angular.z = 0.0
        for _ in range(5):
            self.cmd_vel_pub.publish(twist)
            rospy.sleep(0.01)
        
        # 如果还有旋转动量，施加反向扭矩
        time.sleep(0.1)  # 等待一下
        
        # 检查是否还在旋转
        start_yaw = self.current_yaw
        time.sleep(0.05)
        end_yaw = self.current_yaw
        
        delta = end_yaw - start_yaw
        if abs(delta) > 0.01:  # 还在旋转
            # 施加短暂反向扭矩
            twist.angular.z = -delta * 2.0  # 反向补偿
            for _ in range(3):
                self.cmd_vel_pub.publish(twist)
                rospy.sleep(0.02)
            
            # 再停止
            twist.angular.z = 0.0
            for _ in range(5):
                self.cmd_vel_pub.publish(twist)
                rospy.sleep(0.01)

    def gazebo_pose_to_nav_pose(self,point):
        quaternion_obj = self.get_quaternion(point[3], point[4], point[5])
        return [point[0],point[1],point[2],quaternion_obj.x,quaternion_obj.y,quaternion_obj.z,quaternion_obj.w]

    def set_speed(self,x,z):
        """
        设置线速度与角速度
        
        Args:
            x: 线速度 (正值为前进，负值为倒退)
            z: 角速度（正值为沿着逆时针方向旋转，负值为沿着顺时针方向旋转）
        """    
        twist = Twist()
        twist.linear.x = x
        twist.angular.z = z
        self.publish_cmd_vel_msg(twist)
        return True

    def odom_callback(self, msg):
        # 从四元数中提取yaw角（即机器人的朝向角度）
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        self.current_yaw = yaw
        self.current_position = msg.pose.pose.position
        self.rate.sleep()

    def get_position(self):
        time.sleep(1)
        try:
            tfs = self.buffer.lookup_transform("map","base_footprint",rospy.Time(0))
            # euler = transformations.euler_from_quaternion([tfs.transform.rotation.x,tfs.transform.rotation.y,tfs.transform.rotation.z,tfs.transform.rotation.w])
            return [tfs.transform.translation.x,tfs.transform.translation.y,0,0,0,tfs.transform.rotation.z,tfs.transform.rotation.w]
        except Exception as e:
            return [0,0,0,0,0,0,0]

    def ultrasonic_cal(self,target_distance,Per = 10):
        angular_speed = 0.1
        move_speed = 0.07
        if target_distance >= 0:
            self.ultra_FL = None  
            self.ultra_FR = None
            while not rospy.is_shutdown() and (self.ultra_FL is None or self.ultra_FR is None):
                self.rate.sleep()
        else:
            self.ultra_BL = None  
            self.ultra_BR = None
            while not rospy.is_shutdown() and (self.ultra_BL is None or self.ultra_BR is None):
                self.rate.sleep()

        twist = Twist()
        flag_loop = True
        flag = 0
        flag_i = 0
        while not rospy.is_shutdown() and (flag_loop == True) and (flag <= Per/2):
            flag += 0.02
            if target_distance >= 0:
                # 平行校准：让左右距离相等
                if abs(self.ultra_FL - self.ultra_FR) <= 0.001:  # 假设0.01米的误差可以忽略
                    flag_i += 1
                else:
                    flag_i = 0
                    twist.angular.z =  angular_speed if self.ultra_FL < self.ultra_FR else -angular_speed
                    self.cmd_vel_pub.publish(twist)
            else:
                # 平行校准：让左右距离相等
                if abs(self.ultra_BL - self.ultra_BR) <= 0.001:  # 假设0.01米的误差可以忽略
                    flag_i += 1
                else:
                    flag_i = 0
                    twist.angular.z =  -angular_speed if self.ultra_BL < self.ultra_BR else angular_speed
                    self.cmd_vel_pub.publish(twist)
            if flag_i < 3:
                flag_loop = True
            else:# 停止机器人
                flag_loop = False  
                twist.angular.z = 0.0
                self.cmd_vel_pub.publish(twist)
            self.rate.sleep()
        if flag_loop:
            return not flag_loop

        flag_loop = True
        flag = 0
        flag_i = 0
        while not rospy.is_shutdown() and (flag_loop == True) and (flag <= Per/2):
            if target_distance >= 0:
                # 前进或后退到目标距离
                current_distance = (self.ultra_FL + self.ultra_FR) / 2
                if abs(current_distance - target_distance) <= 0.01:  # 假设0.01米的误差可以忽略
                    flag_i += 1
                else:
                    flag_i = 0
                    twist.linear.x = move_speed if current_distance > target_distance else -move_speed
                    self.cmd_vel_pub.publish(twist)
            else:
                # 前进或后退到目标距离
                current_distance = (self.ultra_BL + self.ultra_BR) / 2
                if abs(current_distance - abs(target_distance)) <= 0.01:  # 假设0.01米的误差可以忽略
                    flag_i += 1
                else:
                    flag_i = 0
                    twist.linear.x = -move_speed if current_distance > target_distance else move_speed
                    self.cmd_vel_pub.publish(twist)
            if flag_i < 3:
                flag_loop = True
            else:# 停止机器人
                flag_loop = False 
                twist.linear.x = 0.0
                self.cmd_vel_pub.publish(twist)
                rospy.loginfo("Reached target distance from the wall.")
            self.rate.sleep()
        twist.linear.x = 0.0
        self.cmd_vel_pub.publish(twist)
        self.rate.sleep()
        return not flag_loop

    # ========== 回调函数 ==========
    
    def ultrasonic_callback(self, data):
        """超声波数据回调"""
        self.ultrasonic_data = [
            data.distanceA, data.distanceB, data.distanceC, 
            data.distanceD, data.distanceE, data.distanceF
        ]
        self.ultra_FL = self.ultrasonic_data[0]  
        self.ultra_FR = self.ultrasonic_data[1]  
        self.ultra_BL = self.ultrasonic_data[4]  
        self.ultra_BR = self.ultrasonic_data[5] 
    
    def user_button_callback(self, data):
        """用户按钮回调"""
        self.user_button_status = data.data
    
    def collide_callback(self, data):
        """防碰撞传感器回调"""
        self.collide_status = data.data
    
    def power_voltage_callback(self, data):
        """电池电量回调"""
        self.power_voltage = data.data
    
    def ir_distance_callback(self, data):
        """红外距离传感器回调"""
        self.fall_status = data.data
    


if __name__ == "__main__":
    try:
        rospy.init_node("aihitplt_motion_api", anonymous=True)
        agv = aihitplt_motion_api()
        rospy.loginfo("启动成功")
 
        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass
      