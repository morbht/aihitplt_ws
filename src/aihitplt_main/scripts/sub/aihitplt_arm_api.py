#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import rospy, sys, math, time
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler

class aihitplt_arm_api:
    def __init__(self):
        """初始化API"""
        # 初始化ROS节点
        try:
            rospy.init_node('arm_api_node', anonymous=True)
        except:
            pass  
        
        # 初始化发布者
        self.joint_pub = rospy.Publisher('/command_joint_states', JointState, queue_size=10)
        self.cartesian_pub = rospy.Publisher('/command_work_space', JointState, queue_size=10)
        
        # 等待发布者建立连接
        time.sleep(0.5)
        
        rospy.loginfo("aihitplt_arm_api nitialized")
    
    def arm_joint_deg_control(self, j1, j2, j3, j4, speed=30):
        """
        控制机械臂到指定关节角度（度）
        参数范围：
        j1: 关节1角度 (-150° to 170°)
        j2: 关节2角度 (-20° to 90°)
        j3: 关节3角度 (-5° to 110°)
        j4: 关节4角度 (-179° to 179°)
        """
        try:
            # 转换为弧度
            joint_angles_radians = [
                math.radians(j1),
                math.radians(j2),
                math.radians(j3),
                math.radians(j4)
            ]
            
            rospy.loginfo(f"设置关节目标角度: [{j1:.1f}°, {j2:.1f}°, {j3:.1f}°, {j4:.1f}°]")
            
            # 发布关节命令
            msg = JointState()
            msg.header.stamp = rospy.Time.now()
            msg.name = ['joint1', 'joint2', 'joint3', 'joint4']
            msg.position = joint_angles_radians
            
            self.joint_pub.publish(msg)
            time.sleep(0.5)  # 等待执行
            
            return True
            
        except Exception as e:
            rospy.logerr(f"关节控制失败: {e}")
            return False
    
    def arm_joint_rad_control(self, j1, j2, j3, j4, speed=30):
        """
        控制机械臂到指定关节角度（弧度）
        参数范围：
        j1: 关节1弧度 (-2.6179 to 2.967 rad)
        j2: 关节2弧度 (-0.3490 to 1.5708 rad)
        j3: 关节3弧度 (-0.0872 to 1.9198 rad)
        j4: 关节4弧度 (-3.1241 to 3.1241 rad)
        """
        try:
            joint_angles_radians = [j1, j2, j3, j4]
            
            rospy.loginfo(f"设置关节目标弧度: {joint_angles_radians}")
            
            msg = JointState()
            msg.header.stamp = rospy.Time.now()
            msg.name = ['joint1', 'joint2', 'joint3', 'joint4']
            msg.position = joint_angles_radians
            
            self.joint_pub.publish(msg)
            time.sleep(0.5)
            
            return True
            
        except Exception as e:
            rospy.logerr(f"关节控制失败: {e}")
            return False
    
    def arm_pose_control(self, x, y, z, roll=0.0, pitch=0.0, yaw=0.0):
        """
        控制机械臂末端到指定位置和姿态（笛卡尔空间）
        x, y, z: 末端位置（毫米）
        roll, pitch, yaw: 末端姿态（弧度，欧拉角）
        """
        try:
            rospy.loginfo(f"设置笛卡尔目标位置: [{x:.1f}, {y:.1f}, {z:.1f}]")
            rospy.loginfo(f"设置姿态: roll={roll:.3f}, pitch={pitch:.3f}, yaw={yaw:.3f}")
            
            # 发布笛卡尔命令
            msg = JointState()
            msg.header.stamp = rospy.Time.now()
            msg.name = ['x', 'y', 'z', 'roll', 'pitch', 'yaw']
            msg.position = [x, y, z, roll, pitch, yaw]
            
            self.cartesian_pub.publish(msg)
            time.sleep(0.5)
            
            return True
            
        except Exception as e:
            rospy.logerr(f"位姿控制失败: {e}")
            return False
    
    def gripper_control(self, position):
        """
        控制夹爪开合
        position: 夹爪角度（弧度），范围：-0.7（完全闭合）到 0.15（完全打开）
        """
        try:
            # 检查输入范围
            if position < -0.7 or position > 0.15:
                rospy.logwarn(f"夹爪角度超出范围 [-0.7, 0.15]，将进行限制")
                position = max(-0.7, min(0.15, position))
            
            rospy.loginfo(f"设置夹爪角度: {position:.4f} rad")
            
            msg = JointState()
            msg.header.stamp = rospy.Time.now()
            msg.name = ['gripper']
            msg.position = [position]
            
            self.joint_pub.publish(msg)
            time.sleep(0.3)
            
            return True
            
        except Exception as e:
            rospy.logerr(f"夹爪控制失败: {e}")
            return False
    
    def open_gripper(self):
        """完全打开夹爪"""
        return self.gripper_control(0.15)
    
    def close_gripper(self):
        """完全闭合夹爪"""
        return self.gripper_control(-0.7)
    
    def go_to_target(self, target_name):
        """
        运动到命名目标位置
        target_name: 预定义的命名位置，如 "home", "zero"
        """
        try:
            rospy.loginfo(f"运动到命名位置: {target_name}")
            
            if target_name == "home" or target_name == "zero":
                # 回零位置
                return self.arm_joint_rad_control(0.0, 0.0, 0.0, 0.0)
            else:
                rospy.logwarn(f"未知的命名位置: {target_name}")
                return False
            
        except Exception as e:
            rospy.logerr(f"命名位置运动失败: {e}")
            return False


if __name__ == "__main__":
    # 测试代码
    try:
        rospy.init_node('arm_api_node', anonymous=True)
        
        # 创建API实例
        arm_api = aihitplt_arm_api()
        
        # # 测试关节控制（角度）
        # arm_api.arm_joint_deg_control(0, 0, 0, 0)
        
        # 测试关节控制（弧度）
        # arm_api.arm_joint_rad_control(0.5, 0.5, 0.5, 0.2)
        # time.sleep(2)
        
        # # 测试夹爪控制
        # arm_api.open_gripper()
        # time.sleep(1)
        # arm_api.close_gripper()
        
        # time.sleep(2)
        
        # 测试笛卡尔控制
        # arm_api.arm_pose_control(198, 150, 50, 0, 0, 0)
        
        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"测试程序运行错误: {e}")