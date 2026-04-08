#!/usr/bin/env python3
"""
正确的硬件接口 - 关节名称和数量完全匹配
"""

import rospy
import actionlib
import time
import math
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal, FollowJointTrajectoryResult
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

from pymycobot.ultraArmP340 import ultraArmP340

class AihitHardwareProper:
    def __init__(self):
        rospy.init_node('aihit_hardware_proper')
        
        # ROS参数
        port = rospy.get_param('~port', '/dev/ttyUSB1')
        baud = rospy.get_param('~baud', 115200)
        
        # 关键：必须与URDF完全一致！
        # 你的URDF关节：joint1_to_base, joint2_to_joint1, joint3_to_joint2
        self.joint_names = ['joint1_to_base', 'joint2_to_joint1', 'joint3_to_joint2']
        rospy.loginfo(f"使用URDF关节名称: {self.joint_names}")
        
        # 连接机械臂
        rospy.loginfo(f"连接机械臂: {port}@{baud}")
        try:
            # 修复：使用ultraArmP340类
            self.arm = ultraArmP340(port, baud)
            time.sleep(2)
            
            # 测试连接
            angles = self.arm.get_angles_info()
            if angles is None:
                rospy.logerr("无法读取机械臂角度")
                raise Exception("机械臂连接失败")
            
            rospy.loginfo(f"机械臂连接成功! 原始角度: {angles}")
            
            # 只取前3个关节（运动关节）
            if len(angles) >= 3:
                self.current_positions = [angles[i] * math.pi / 180 for i in range(3)]
                rospy.loginfo(f"运动关节角度(弧度): {self.current_positions}")
            else:
                rospy.logwarn(f"机械臂返回角度不足3个: {angles}")
                self.current_positions = [0.0, 0.0, 0.0]
            
            # 发布关节状态 - 确保名称和位置数量完全一致！
            self.joint_state_pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
            
            # Action服务器
            self.server = actionlib.SimpleActionServer(
                '/arm_group_controller/follow_joint_trajectory',
                FollowJointTrajectoryAction,
                execute_cb=self.execute_trajectory,
                auto_start=False
            )
            self.server.start()
            
            # 立即发布一次关节状态
            self.publish_joint_state()
            
            # 定时更新
            self.update_timer = rospy.Timer(rospy.Duration(0.1), self.update_joint_state)
            
            rospy.loginfo("Aihit硬件接口已正确启动")
            
        except Exception as e:
            rospy.logerr(f"初始化失败: {str(e)}")
            raise
    
    def publish_joint_state(self):
        """发布关节状态 - 确保名称和位置数量匹配"""
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()
        
        # 关键：关节名称和位置必须数量相同！
        msg.name = self.joint_names  # 3个名称
        msg.position = self.current_positions  # 3个位置
        
        # 确保所有数组长度相同
        msg.velocity = [0.0] * len(msg.name)
        msg.effort = [0.0] * len(msg.name)
        
        self.joint_state_pub.publish(msg)
    
    def update_joint_state(self, event):
        """更新关节状态"""
        try:
            angles = self.arm.get_angles_info()
            if angles and len(angles) >= 3:
                # 只取前3个关节
                self.current_positions = [angles[i] * math.pi / 180 for i in range(3)]
            
            self.publish_joint_state()
            
        except Exception as e:
            rospy.logwarn(f"更新关节状态失败: {e}")
    
    def execute_trajectory(self, goal):
        """执行轨迹"""
        rospy.loginfo("收到MoveIt轨迹")
        
        trajectory = goal.trajectory
        result = FollowJointTrajectoryResult()
        
        rospy.loginfo(f"轨迹关节: {trajectory.joint_names}")
        rospy.loginfo(f"期望关节: {self.joint_names}")
        
        try:
            for i, point in enumerate(trajectory.points):
                rospy.loginfo(f"执行点 {i+1}/{len(trajectory.points)}")
                
                # 读取当前所有角度
                current_angles = self.arm.get_angles_info()
                if current_angles is None:
                    continue
                
                # 构建4个角度：前3个来自MoveIt，第4个保持当前夹爪角度
                target_angles = []
                
                # 前3个：运动关节
                for pos in point.positions:
                    target_angles.append(pos * 180 / math.pi)  # 弧度转角度
                
                # 第4个：保持夹爪当前角度
                if len(current_angles) >= 4:
                    target_angles.append(current_angles[3])
                else:
                    target_angles.append(0)
                
                rospy.loginfo(f"发送到机械臂: {target_angles}")
                
                # 发送角度指令
                self.arm.set_angles(target_angles, 100)  # 增加速度到100
                
                # 更新当前状态
                self.current_positions = list(point.positions)
                
                # 等待机械臂运动完成（重要！）
                if i < len(trajectory.points) - 1:
                    # 检查机械臂是否还在运动
                    moving_end = False
                    start_time = time.time()
                    while not moving_end and (time.time() - start_time) < 5.0:  # 最多等待5秒
                        # 检查运动结束标志
                        try:
                            moving_end = self.arm.is_moving_end()
                        except:
                            # 如果is_moving_end不可用，使用简单延时
                            time.sleep(0.1)
                            break
                        if moving_end:
                            break
                        time.sleep(0.05)  # 短暂检查间隔
                
                # 或者使用固定延时（更简单的方法）
                # time.sleep(0.3)  # 每个点之间等待300ms
            
            result.error_code = result.SUCCESSFUL
            self.server.set_succeeded(result)
            rospy.loginfo("轨迹执行完成")
            
        except Exception as e:
            rospy.logerr(f"执行失败: {e}")
            result.error_code = result.INVALID_GOAL
            self.server.set_aborted(result)
    
    def cleanup(self):
        rospy.loginfo("清理资源")
        try:
            self.arm.release_all_servos()
        except:
            pass

def main():
    hardware = None
    try:
        hardware = AihitHardwareProper()
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("用户中断")
    except Exception as e:
        rospy.logerr(f"运行错误: {e}")
    finally:
        if hardware:
            hardware.cleanup()
        rospy.loginfo("硬件接口关闭")

if __name__ == '__main__':
    main()