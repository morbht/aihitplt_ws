#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from sensor_msgs.msg import PointCloud2, PointField
from cv_bridge import CvBridge
from message_filters import TimeSynchronizer, Subscriber
from geometry_msgs.msg import Point32

class AstraRGBDToPointCloud:
    def __init__(self):
        self.bridge = CvBridge()
        self.depth_sub = Subscriber("/camera/depth/image_raw", Image)
        self.rgb_sub = Subscriber("/camera/rgb/image_raw", Image)
        self.ts = TimeSynchronizer([self.depth_sub, self.rgb_sub], queue_size=10)
        self.ts.registerCallback(self.callback)
        self.point_cloud_pub = rospy.Publisher("/astra_rgbd_point_cloud", PointCloud2, queue_size=10)
        self.camera_matrix_depth = None
        self.camera_matrix_rgb = None

    def camera_info_callback_depth(self, msg):
        self.camera_matrix_depth = np.array(msg.K).reshape(3, 3)

    def camera_info_callback_rgb(self, msg):
        self.camera_matrix_rgb = np.array(msg.K).reshape(3, 3)

    def callback(self, depth_msg, rgb_msg):
        if self.camera_matrix_depth is None or self.camera_matrix_rgb is None:
            return

        try:
            # 转换深度图（单位：米）
            depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
            depth_meters = depth_image.astype(np.float32) / 1000.0  # 假设原始单位为毫米
        except CvBridgeError as e:
            rospy.logerr(e)
            return

        # 获取图像尺寸
        height, width = depth_meters.shape

        # 创建PointCloud2消息
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "camera_depth_optical_frame"

        # 定义点云字段（包含RGB颜色）
        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="rgb", offset=12, datatype=PointField.FLOAT32, count=1),
        ]

        # 初始化点云数据
        points = []
        for v in range(height):
            for u in range(width):
                depth = depth_meters[v, u]
                if depth == 0:
                    continue

                # 计算3D坐标（使用深度相机内参）
                x = (u - self.camera_matrix_depth[0, 2]) * depth / self.camera_matrix_depth[0, 0]
                y = (v - self.camera_matrix_depth[1, 2]) * depth / self.camera_matrix_depth[1, 1]
                z = depth

                # 获取对应RGB颜色（使用RGB相机内参）
                try:
                    bgr_pixel = self.bridge.imgmsg_to_cv2(rgb_msg, "bgr8")[v, u]
                    rgb = (bgr_pixel[2] << 16) | (bgr_pixel[1] << 8) | bgr_pixel[0]  # 转换为ROS RGB格式
                except CvBridgeError as e:
                    rospy.logerr(e)
                    rgb = 0

                # 添加点到列表
                points.append(Point32(x, y, z))
                points[-1].rgba = rgb  # 或拆分BGR通道存储

        # 发布点云
        pc2_msg = pc2.create_cloud(header, fields, points)
        self.point_cloud_pub.publish(pc2_msg)

if __name__ == "__main__":
    rospy.init_node("astra_rgbd_pointcloud")
    print("RGB-D has been converted to point cloud")
    node = AstraRGBDToPointCloud()
    rospy.spin()
