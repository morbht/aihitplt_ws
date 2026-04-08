#!/usr/bin/env python3

import rospy
import actionlib
import time
import math
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal, FollowJointTrajectoryResult
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from pymycobot.ultraArm import ultraArm

class Aihit3AxisHardware:
    def __init__(self):
        rospy.init_node('aihit_3axis_hardware')
        
        # ROS参数
        port = rospy.get_param('~port', '/dev/ttyUSB2')
        baud = rospy.get_param('~baud', 115200)
        
        # MoveIt配置的3个运动关节（对应URDF中的关节）
        # URDF关节: joint1_to_base, joint2_to_joint1, joint3_to_joint2
        # 对应机械臂的前3个关节
        self.moveit_joint_names = ['joint1_to_base', 'joint2_to_joint1', 'joint3_to_joint2']
        
        rospy.loginfo(f"MoveIt关节: {self.moveit_joint_names}")
        
        # 连接机械臂
        rospy.loginfo(f"连接机械臂: {port}@{baud}")
        try:
            self.arm = ultraArm(port, baud)
            time.sleep(2)
            
            # 测试连接
            angles = self.arm.get_angles_info()
            if angles is None:
                rospy.logerr("无法读取机械臂角度")
                raise Exception("机械臂连接失败")
            
            rospy.loginfo(f"机械臂连接成功! 所有关节角度: {angles}")
            rospy.loginfo(f"运动关节(前3个): {angles[:3] if len(angles) >= 3 else angles}")
            
            # 初始化位置：前3个关节为运动关节，第4个为夹爪
            self.current_positions = [0.0, 0.0, 0.0]  # 只包含3个运动关节
            
            if angles and len(angles) >= 3:
                # 只取前3个关节（运动关节）
                self.current_positions = [angles[i] * math.pi / 180 for i in range(3)]
            
            # 发布关节状态（只发布3个运动关节）
            self.joint_state_pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
            
            # Action服务器 - 注意控制器名称
            self.server = actionlib.SimpleActionServer(
                '/aihit_arm_controller/follow_joint_trajectory',
                FollowJointTrajectoryAction,
                execute_cb=self.execute_trajectory,
                auto_start=False
            )
            self.server.start()
            
            # 定时更新关节状态
            self.update_timer = rospy.Timer(rospy.Duration(0.1), self.update_joint_state)
            
            rospy.loginfo("Aihit 3轴机械臂硬件接口已启动")
            rospy.loginfo("注意: 夹爪关节(joint4)不包含在MoveIt控制中")
            
        except Exception as e:
            rospy.logerr(f"初始化失败: {str(e)}")
            raise
    
    def update_joint_state(self, event):
        """更新并发布关节状态（只发布3个运动关节）"""
        try:
            # 从机械臂读取所有角度
            angles = self.arm.get_angles_info()
            if angles and len(angles) >= 3:
                # 只取前3个运动关节，转换为弧度
                self.current_positions = [angles[i] * math.pi / 180 for i in range(3)]
            
            # 发布关节状态（只包含3个运动关节）
            msg = JointState()
            msg.header = Header()
            msg.header.stamp = rospy.Time.now()
            msg.name = self.moveit_joint_names  # 使用MoveIt的关节名称
            msg.position = self.current_positions
            msg.velocity = [0.0, 0.0, 0.0]
            msg.effort = [0.0, 0.0, 0.0]
            
            self.joint_state_pub.publish(msg)
            
        except Exception as e:
            rospy.logwarn(f"更新关节状态失败: {e}")
            # 使用最后已知位置
            msg = JointState()
            msg.header = Header()
            msg.header.stamp = rospy.Time.now()
            msg.name = self.moveit_joint_names
            msg.position = self.current_positions
            msg.velocity = [0.0, 0.0, 0.0]
            msg.effort = [0.0, 0.0, 0.0]
            self.joint_state_pub.publish(msg)
    
    def execute_trajectory(self, goal):
        """执行MoveIt规划的轨迹（只控制前3个关节）"""
        rospy.loginfo("收到MoveIt轨迹执行请求")
        
        trajectory = goal.trajectory
        result = FollowJointTrajectoryResult()
        
        rospy.loginfo(f"轨迹关节: {trajectory.joint_names}")
        rospy.loginfo(f"期望关节: {self.moveit_joint_names}")
        
        try:
            for i, point in enumerate(trajectory.points):
                rospy.loginfo(f"执行点 {i+1}/{len(trajectory.points)}")
                rospy.loginfo(f"目标位置(弧度): {point.positions}")
                
                # 从机械臂读取当前所有角度
                current_angles = self.arm.get_angles_info()
                if current_angles is None:
                    rospy.logerr("无法读取当前角度")
                    continue
                
                # 构建要发送的4个角度：前3个来自MoveIt，第4个保持当前夹爪角度
                target_angles = []
                
                # 前3个：运动关节（转换为角度）
                for pos in point.positions:
                    target_angles.append(pos * 180 / math.pi)
                
                # 第4个：保持当前夹爪角度
                if len(current_angles) >= 4:
                    target_angles.append(current_angles[3])  # 保持当前夹爪角度
                else:
                    target_angles.append(0)  # 默认值
                
                rospy.loginfo(f"发送到机械臂的角度: {target_angles}")
                
                # 发送到机械臂
                self.arm.send_angles(target_angles, 50)
                
                # 更新当前状态（只更新前3个）
                self.current_positions = list(point.positions)
                
                # 等待执行时间
                if point.time_from_start.to_sec() > 0:
                    time.sleep(point.time_from_start.to_sec())
                else:
                    time.sleep(0.5)
            
            result.error_code = result.SUCCESSFUL
            self.server.set_succeeded(result)
            rospy.loginfo("轨迹执行完成")
            
        except Exception as e:
            rospy.logerr(f"执行失败: {str(e)}")
            result.error_code = result.INVALID_GOAL
            self.server.set_aborted(result)
    
    def cleanup(self):
        """清理资源"""
        rospy.loginfo("清理硬件接口资源")
        try:
            self.arm.release_all_servos()
        except:
            pass

def main():
    hardware = None
    try:
        hardware = Aihit3AxisHardware()
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("用户中断")
    except Exception as e:
        rospy.logerr(f"运行错误: {str(e)}")
    finally:
        if hardware:
            hardware.cleanup()
        rospy.loginfo("硬件接口已关闭")

if __name__ == '__main__':
    main()