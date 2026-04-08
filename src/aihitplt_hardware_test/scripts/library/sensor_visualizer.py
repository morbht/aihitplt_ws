#!/usr/bin/env python3
# -*- coding:utf-8 -*-
import rospy
import tf
from geometry_msgs.msg import PointStamped, Vector3
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header, ColorRGBA
from aihitplt_bringup.msg import supersonic
from std_msgs.msg import Bool, Float32

class SensorVisualizer:
    def __init__(self):
        rospy.init_node('sensor_visualizer', anonymous=True)
        
        # 订阅传感器数据
        self.ultrasonic_sub = rospy.Subscriber("/Distance", supersonic, self.ultrasonic_callback)
        self.ir_distance_sub = rospy.Subscriber("/ir_distance", Float32, self.ir_distance_callback)
        
        # 发布可视化标记
        self.marker_pub = rospy.Publisher("/sensor_markers", MarkerArray, queue_size=10)
        self.text_marker_pub = rospy.Publisher("/sensor_text_markers", MarkerArray, queue_size=10)
        
        # 初始化传感器数据
        self.ultrasonic_data = [0.0] * 6
        self.fall_status = 0.0
        
        # TF广播器
        self.tf_broadcaster = tf.TransformBroadcaster()
        
        # 坐标系名称
        self.ultrasonic_frames = [
            "ultrasonic_left_front_link",
            "ultrasonic_right_front_link", 
            "ultrasonic_left_link",
            "ultrasonic_right_link",
            "ultrasonic_left_back_link",
            "ultrasonic_right_back_link"
        ]
        self.anti_fall_frame = "anti_fall_link"
        
        # 传感器位置（相对于base_link，请根据实际情况调整）
        self.sensor_positions = {
            "ultrasonic_left_front_link": (0.3, 0.15, 0.1),
            "ultrasonic_right_front_link": (0.3, -0.15, 0.1),
            "ultrasonic_left_link": (0.0, 0.2, 0.1),
            "ultrasonic_right_link": (0.0, -0.2, 0.1),
            "ultrasonic_left_back_link": (-0.3, 0.15, 0.1),
            "ultrasonic_right_back_link": (-0.3, -0.15, 0.1),
            "anti_fall_link": (0.0, 0.0, -0.1)
        }
        
        # 定时发布TF和标记
        self.timer = rospy.Timer(rospy.Duration(0.1), self.publish_tf_and_markers)

    def publish_tf_and_markers(self, event):
        """发布TF坐标系和可视化标记"""
        current_time = rospy.Time.now()
        
        # 发布超声波传感器TF
        for i, frame in enumerate(self.ultrasonic_frames):
            if frame in self.sensor_positions:
                pos = self.sensor_positions[frame]
                self.tf_broadcaster.sendTransform(
                    pos,
                    (0, 0, 0, 1),  # 无旋转
                    current_time,
                    frame,
                    "base_link"  # 请根据实际情况修改父坐标系
                )
        
        # 发布防跌落传感器TF
        if self.anti_fall_frame in self.sensor_positions:
            pos = self.sensor_positions[self.anti_fall_frame]
            self.tf_broadcaster.sendTransform(
                pos,
                (0, 0, 0, 1),
                current_time,
                self.anti_fall_frame,
                "base_link"
            )
        
        # 发布可视化标记
        self.publish_ultrasonic_markers(current_time)
        self.publish_fall_sensor_marker(current_time)

    def publish_ultrasonic_markers(self, timestamp):
        """发布超声波传感器数据的可视化标记"""
        marker_array = MarkerArray()
        text_marker_array = MarkerArray()
        
        for i, frame in enumerate(self.ultrasonic_frames):
            if i < len(self.ultrasonic_data):
                distance = self.ultrasonic_data[i]
                
                # 创建距离测量线标记
                marker = Marker()
                marker.header = Header(frame_id=frame, stamp=timestamp)
                marker.ns = "ultrasonic_rays"
                marker.id = i
                marker.type = Marker.LINE_STRIP
                marker.action = Marker.ADD
                marker.scale.x = 0.02  # 线宽
                
                # 设置颜色（根据距离变化）
                if distance < 0.5:
                    marker.color = ColorRGBA(1.0, 0.0, 0.0, 0.8)  # 红色，距离近
                elif distance < 1.0:
                    marker.color = ColorRGBA(1.0, 1.0, 0.0, 0.8)  # 黄色，距离中等
                else:
                    marker.color = ColorRGBA(0.0, 1.0, 0.0, 0.8)  # 绿色，距离远
                
                # 设置线的起点和终点
                start_point = PointStamped()
                start_point.header = marker.header
                start_point.point.x = 0.0
                start_point.point.y = 0.0
                start_point.point.z = 0.0
                
                end_point = PointStamped()
                end_point.header = marker.header
                end_point.point.x = distance  # 超声波测量距离
                end_point.point.y = 0.0
                end_point.point.z = 0.0
                
                marker.points = [start_point.point, end_point.point]
                marker_array.markers.append(marker)
                
                # 创建文本标记显示具体数值
                text_marker = Marker()
                text_marker.header = Header(frame_id=frame, stamp=timestamp)
                text_marker.ns = "ultrasonic_text"
                text_marker.id = i
                text_marker.type = Marker.TEXT_VIEW_FACING
                text_marker.action = Marker.ADD
                text_marker.pose.position.x = distance / 2
                text_marker.pose.position.y = 0.0
                text_marker.pose.position.z = 0.05
                text_marker.scale.z = 0.05  # 文字大小
                text_marker.color = ColorRGBA(1.0, 1.0, 1.0, 1.0)  # 白色
                text_marker.text = f"{distance:.2f}m"
                text_marker_array.markers.append(text_marker)
        
        self.marker_pub.publish(marker_array)
        self.text_marker_pub.publish(text_marker_array)

    def publish_fall_sensor_marker(self, timestamp):
        """发布防跌落传感器可视化标记"""
        marker_array = MarkerArray()
        
        # 创建防跌落传感器状态球体
        marker = Marker()
        marker.header = Header(frame_id=self.anti_fall_frame, stamp=timestamp)
        marker.ns = "fall_sensor"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0
        marker.scale.x = 0.08
        marker.scale.y = 0.08
        marker.scale.z = 0.08
        
        # 根据跌落状态设置颜色
        if self.fall_status < 0.1:  # 假设小于0.1表示有跌落风险
            marker.color = ColorRGBA(1.0, 0.0, 0.0, 0.8)  # 红色，有风险
        else:
            marker.color = ColorRGBA(0.0, 1.0, 0.0, 0.8)  # 绿色，安全
        
        marker_array.markers.append(marker)
        
        # 添加文本显示具体数值
        text_marker = Marker()
        text_marker.header = Header(frame_id=self.anti_fall_frame, stamp=timestamp)
        text_marker.ns = "fall_sensor_text"
        text_marker.id = 1
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.action = Marker.ADD
        text_marker.pose.position.x = 0.0
        text_marker.pose.position.y = 0.0
        text_marker.pose.position.z = 0.1
        text_marker.scale.z = 0.04
        text_marker.color = ColorRGBA(1.0, 1.0, 1.0, 1.0)
        text_marker.text = f"Fall: {self.fall_status:.2f}"
        marker_array.markers.append(text_marker)
        
        self.marker_pub.publish(marker_array)

    def ultrasonic_callback(self, data):
        """超声波数据回调"""
        self.ultrasonic_data = [
            data.distanceA, data.distanceB, data.distanceC, 
            data.distanceD, data.distanceE, data.distanceF
        ]

    def ir_distance_callback(self, data):
        """红外距离传感器回调"""
        self.fall_status = data.data

    def run(self):
        rospy.loginfo("传感器可视化节点已启动")
        rospy.spin()

if __name__ == "__main__":
    try:
        visualizer = SensorVisualizer()
        visualizer.run()
    except rospy.ROSInterruptException:
        pass