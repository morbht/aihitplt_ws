#!/usr/bin/env python3

import rospy
import math
import tf
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl
from visualization_msgs.msg import Marker, InteractiveMarkerFeedback
from geometry_msgs.msg import Twist, Pose, Point, Quaternion
from sensor_msgs.msg import JointState

class AIHIT_AGV_TF_FixedControl:
    def __init__(self):
        rospy.init_node('aihit_agv_control')
        
        self.base_frame = "odom_combined"  
        
        self.cmd_vel_topic = "/cmd_vel"
        
        # 速度限制
        self.max_linear_speed = 1.0
        self.max_angular_speed = 1.0
        
        # 控制参数
        self.control_scale = 1.5
        self.marker_height = 0.05
        self.marker_scale = 0.6
        
        # 轮子参数
        self.wheel_radius = 0.076
        self.wheel_separation = 0.306
        
        # 关节状态
        self.left_wheel_pos = 0.0
        self.right_wheel_pos = 0.0
        self.last_time = rospy.Time.now()
        
        
        self.tf_listener = tf.TransformListener()
        
        # 创建服务器和发布器
        self.server = InteractiveMarkerServer("aihit_agv_control")
        self.cmd_vel_pub = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size=10)
        self.joint_pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
        
        # 标记名称
        self.marker_name = "aihitplt_control"
        
        # 等待系统初始化
        rospy.sleep(1.0)
        
        # 创建控制标记
        self.create_control_marker()
        
        # 定时发布
        rospy.Timer(rospy.Duration(0.02), self.publish_joint_states)
        
        rospy.spin()
    
    def get_base_link_position(self):
        try:
            # 获取base_link在odom_combined坐标系中的位置
            (trans, rot) = self.tf_listener.lookupTransform(
                'odom_combined',  # 目标坐标系
                'base_link',      # 源坐标系
                rospy.Time(0)     # 最新时间
            )
            return trans, rot
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return [0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]
    
    def create_control_marker(self):
        """创建交互控制标记"""

        trans, rot = self.get_base_link_position()
        
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = self.base_frame  
        int_marker.header.stamp = rospy.Time.now()
        int_marker.name = self.marker_name
        int_marker.description = "AIHIT AGV Control\nDrag arrow to move\nDrag circle to rotate"
        int_marker.scale = self.marker_scale
        
 
        int_marker.pose.position = Point(trans[0], trans[1], trans[2] + self.marker_height)
        int_marker.pose.orientation = Quaternion(rot[0], rot[1], rot[2], rot[3])

        # 视觉控制
        visual_control = InteractiveMarkerControl()
        visual_control.always_visible = True
        
        # 旋转控制
        rotate_control = InteractiveMarkerControl()
        rotate_control.name = "rotate_z"
        rotate_control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        rotate_control.orientation = Quaternion(0, 0.7071, 0, 0.7071)  # 水平圆环
        
        # 移动控制
        move_control = InteractiveMarkerControl()
        move_control.name = "move_x"
        move_control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        move_control.orientation = Quaternion(0.7071, 0, 0, 0.7071)  # X轴移动
        
        # 添加控制
        int_marker.controls.append(visual_control)
        int_marker.controls.append(rotate_control)
        int_marker.controls.append(move_control)
        
        # 插入服务器
        self.server.insert(int_marker, self.process_feedback)
        self.server.applyChanges()
        
    
    def process_feedback(self, feedback):

        if feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
            try:
                # 获取当前base_link位置
                trans, rot = self.get_base_link_position()
                
                # 计算标记相对于base_link的位移
                dx = feedback.pose.position.x - trans[0]
                dy = feedback.pose.position.y - trans[1]
                
                # 计算标记相对于base_link的旋转

                q_marker = feedback.pose.orientation
                q_base = Quaternion(rot[0], rot[1], rot[2], rot[3])
                
                # 计算速度
                twist = self.calculate_velocity(dx, dy, q_marker, q_base)
                
                # 发布速度
                self.cmd_vel_pub.publish(twist)
                
                # 更新关节位置
                self.update_wheel_positions(twist)
                
                # 重新定位标记到base_link当前位置
                self.reposition_marker()
                
            except Exception as e:
                rospy.logwarn(f"反馈处理错误: {e}")
        
        elif feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP:
            # 停止
            twist = Twist()
            self.cmd_vel_pub.publish(twist)
            
            # 重新定位标记
            self.reposition_marker()
    
    def calculate_velocity(self, dx, dy, q_marker, q_base):
        """计算速度 - 基于相对位移"""
        twist = Twist()
        
        linear_x = dx * self.control_scale
        if abs(linear_x) < 0.01:
            linear_x = 0.0
        twist.linear.x = max(-self.max_linear_speed, 
                            min(self.max_linear_speed, linear_x))
        
        # 计算旋转角度差
        yaw_marker = math.atan2(2.0*(q_marker.w*q_marker.z + q_marker.x*q_marker.y), 
                               1.0 - 2.0*(q_marker.y*q_marker.y + q_marker.z*q_marker.z))
        yaw_base = math.atan2(2.0*(q_base.w*q_base.z + q_base.x*q_base.y), 
                             1.0 - 2.0*(q_base.y*q_base.y + q_base.z*q_base.z))
        
        yaw_diff = yaw_marker - yaw_base
        # 处理角度跨越π的情况
        if yaw_diff > math.pi:
            yaw_diff -= 2 * math.pi
        elif yaw_diff < -math.pi:
            yaw_diff += 2 * math.pi
        
        angular_z = yaw_diff * self.control_scale
        if abs(angular_z) < 0.02:
            angular_z = 0.0
        twist.angular.z = max(-self.max_angular_speed, 
                             min(self.max_angular_speed, angular_z))
        
        return twist
    
    def update_wheel_positions(self, twist):
        """更新轮子位置"""
        current_time = rospy.Time.now()
        dt = (current_time - self.last_time).to_sec()
        self.last_time = current_time
        
        if dt <= 0:
            return
        
        v = twist.linear.x
        w = twist.angular.z
        
        # 差分驱动运动学
        v_left = v - w * self.wheel_separation / 2.0
        v_right = v + w * self.wheel_separation / 2.0
        
        w_left = v_left / self.wheel_radius
        w_right = v_right / self.wheel_radius
        
        # 积分
        self.left_wheel_pos += w_left * dt
        self.right_wheel_pos += w_right * dt
    
    def reposition_marker(self):

        try:
            trans, rot = self.get_base_link_position()
            
            new_pose = Pose()
            new_pose.position = Point(trans[0], trans[1], trans[2] + self.marker_height)
            new_pose.orientation = Quaternion(rot[0], rot[1], rot[2], rot[3])
            
            self.server.setPose(self.marker_name, new_pose)
            self.server.applyChanges()
        except Exception as e:
            rospy.logwarn(f"重新定位标记失败: {e}")
    
    def publish_joint_states(self, event):
        """发布关节状态"""
        joint_msg = JointState()
        joint_msg.header.stamp = rospy.Time.now()
        
        joint_msg.name = ['left_wheel_joint', 'right_wheel_joint']
        joint_msg.position = [self.left_wheel_pos, self.right_wheel_pos]
        joint_msg.velocity = [0.0, 0.0]
        
        self.joint_pub.publish(joint_msg)

if __name__ == "__main__":
    try:
        AIHIT_AGV_TF_FixedControl()
    except rospy.ROSInterruptException:
        rospy.loginfo("节点关闭")