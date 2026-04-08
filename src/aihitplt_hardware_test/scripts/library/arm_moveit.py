#!/usr/bin/env python3

import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal, FollowJointTrajectoryResult
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from pymycobot.ultraArm import ultraArm
import threading
import time

class AihitMoveItHardware:
    def __init__(self):
        # ROS参数
        port = rospy.get_param('~port', '/dev/ttyUSB1')
        baud = rospy.get_param('~baud', 115200)
        
        # 机械臂连接
        rospy.loginfo(f"连接机械臂: {port}@{baud}")
        self.arm = ultraArm(port, baud)
        time.sleep(1)
        
        # 检查连接
        if self.arm.get_angles_info() is None:
            rospy.logerr("无法连接到机械臂！")
            return
        
        rospy.loginfo("机械臂连接成功！")
        
        # 关节名称（必须与URDF中的一致）
        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        
        # 关节状态发布器
        self.joint_state_pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
        
        # Action服务器
        self.server = actionlib.SimpleActionServer(
            '/aihit_arm_controller/follow_joint_trajectory',
            FollowJointTrajectoryAction,
            execute_cb=self.execute_trajectory,
            auto_start=False
        )
        self.server.start()
        
        # 定时发布关节状态
        self.timer = rospy.Timer(rospy.Duration(0.1), self.publish_joint_state)
        
        rospy.loginfo("Aihit MoveIt硬件接口已启动")
    
    def publish_joint_state(self, event):
        """发布关节状态"""
        try:
            # 从机械臂读取角度
            angles = self.arm.get_angles_info()
            
            if angles is None:
                rospy.logwarn("无法读取机械臂角度")
                return
            
            # 转换为弧度（ROS使用弧度）
            positions = [angle * 3.14159 / 180 for angle in angles]
            
            # 创建JointState消息
            msg = JointState()
            msg.header = Header()
            msg.header.stamp = rospy.Time.now()
            msg.name = self.joint_names
            msg.position = positions
            msg.velocity = [0.0] * len(positions)  # 如果没有速度信息
            msg.effort = [0.0] * len(positions)    # 如果没有力信息
            
            self.joint_state_pub.publish(msg)
            
        except Exception as e:
            rospy.logerr(f"发布关节状态时出错: {e}")
    
    def execute_trajectory(self, goal):
        """执行轨迹"""
        rospy.loginfo("收到轨迹执行请求")
        
        trajectory = goal.trajectory
        result = FollowJointTrajectoryResult()
        
        # 验证关节名称
        if trajectory.joint_names != self.joint_names:
            rospy.logerr(f"关节名称不匹配: {trajectory.joint_names} != {self.joint_names}")
            result.error_code = result.INVALID_JOINTS
            self.server.set_aborted(result)
            return
        
        try:
            # 执行每个轨迹点
            for i, point in enumerate(trajectory.points):
                rospy.loginfo(f"执行第 {i+1}/{len(trajectory.points)} 个轨迹点")
                
                # 将弧度转换为角度
                angles = [pos * 180 / 3.14159 for pos in point.positions]
                
                # 发送到机械臂
                self.arm.send_angles(angles, 50)  # 速度50
                
                # 等待轨迹点执行时间
                if point.time_from_start.to_sec() > 0:
                    time.sleep(point.time_from_start.to_sec())
                else:
                    # 默认等待
                    time.sleep(0.5)
            
            # 执行成功
            result.error_code = result.SUCCESSFUL
            self.server.set_succeeded(result)
            rospy.loginfo("轨迹执行完成")
            
        except Exception as e:
            rospy.logerr(f"轨迹执行失败: {e}")
            result.error_code = result.INVALID_GOAL
            self.server.set_aborted(result)

if __name__ == '__main__':
    rospy.init_node('aihit_moveit_hardware')
    
    try:
        hardware = AihitMoveItHardware()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        rospy.loginfo("关闭硬件接口")
