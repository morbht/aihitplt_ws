#! /usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import cv2
import cv_bridge
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

class RGBD深度融合:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        # 订阅深度图像、彩色图像和相机信息话题
        self.depth_image_sub = rospy.Subscriber('/camera/depth/image_raw', Image, self.depth_image_callback, queue_size=1, buff_size=2**24)
        self.rgb_image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.rgb_image_callback, queue_size=1, buff_size=2**24)
        self.camera_info_sub = rospy.Subscriber('/camera/depth/camera_info', CameraInfo, self.camera_info_callback)
        # 发布带有彩色信息的点云话题
        self.point_cloud_pub = rospy.Publisher('/colored_point_cloud', PointCloud2, queue_size=10)
        self.camera_matrix = None
        self.rgb_image = None
        self.rgb_image_timestamp = None
        self.depth_image = None
        self.depth_image_timestamp = None

    def camera_info_callback(self, msg):
        """获取相机内参矩阵"""
        self.camera_matrix = np.array(msg.K).reshape(3, 3)

    def rgb_image_callback(self, msg):
        """获取彩色图像回调函数"""
        try:
            # 将ROS图像消息转换为OpenCV格式
            self.rgb_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.rgb_image_timestamp = msg.header.stamp
        except cv_bridge.CvBridgeError as e:
            rospy.logerr(e)

    def depth_image_callback(self, msg):
        """深度图像回调函数，处理深度图像并发布带有彩色信息的点云"""
        if self.camera_matrix is None or self.rgb_image is None:
            return
        try:
            # 将ROS深度图像消息转换为OpenCV格式
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            self.depth_image = depth_image.astype(np.float32) / 1000.0  # 转换为米
            self.depth_image_timestamp = msg.header.stamp
        except cv_bridge.CvBridgeError as e:
            rospy.logerr(e)
            return

        # 确保深度图像和彩色图像时间戳匹配
        if self.rgb_image_timestamp != self.depth_image_timestamp:
            return

        # 获取图像尺寸和相机内参
        height, width = self.depth_image.shape
        fx = self.camera_matrix[0, 0]
        fy = self.camera_matrix[1, 1]
        cx = self.camera_matrix[0, 2]
        cy = self.camera_matrix[1, 2]

        # 创建网格坐标
        u, v = np.meshgrid(np.arange(width), np.arange(height))
        u = u.astype(np.float32)
        v = v.astype(np.float32)

        # 计算三维坐标
        x = (u - cx) * self.depth_image / fx
        y = (v - cy) * self.depth_image / fy
        z = self.depth_image

        # 获取彩色信息
        rgb_values = self.rgb_image.reshape(-1, 3)
        rgb = (rgb_values[:, 0] << 16) | (rgb_values[:, 1] << 8) | rgb_values[:, 2]

        # 组合点云数据
        points = np.column_stack((x.reshape(-1), y.reshape(-1), z.reshape(-1), rgb))

        # 创建并发布带有彩色信息的点云消息
        header = msg.header
        header.frame_id = 'camera_depth_optical_frame'
        fields = [
            pc2.PointField('x', 0, pc2.PointField.FLOAT32, 1),
            pc2.PointField('y', 4, pc2.PointField.FLOAT32, 1),
            pc2.PointField('z', 8, pc2.PointField.FLOAT32, 1),
            pc2.PointField('rgb', 16, pc2.PointField.UINT32, 1)
        ]
        point_cloud_msg = pc2.create_cloud(header, fields, points.tolist())
        self.point_cloud_pub.publish(point_cloud_msg)
        print("彩色信息融合完成，点云已发布")

if __name__ == '__main__':
    rospy.init_node('rgbd_bind')
    rgbd_fusion = RGBD深度融合()
    rospy.spin()
