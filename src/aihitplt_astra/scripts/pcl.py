#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import pcl
import numpy as np
import ctypes
import struct
import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField

def create_cloud_xyz32(header, points):
    fields = [
        PointField('x', 0, PointField.FLOAT32, 1),
        PointField('y', 4, PointField.FLOAT32, 1),
        PointField('z', 8, PointField.FLOAT32, 1),
        PointField('rgb', 16, PointField.UINT32, 1)
    ]
    return pc2.create_cloud(header, fields, points)

def pcl_to_ros(pcl_array):
    ros_msg = PointCloud2()
    ros_msg.header.stamp = rospy.Time.now()
    ros_msg.header.frame_id = "camera_depth_optical_frame"
    ros_msg.height = pcl_array.shape[0]
    ros_msg.width = pcl_array.shape[1]
    ros_msg.fields = [
        PointField('x', 0, PointField.FLOAT32, 1),
        PointField('y', 4, PointField.FLOAT32, 1),
        PointField('z', 8, PointField.FLOAT32, 1),
        PointField('rgb', 16, PointField.UINT32, 1)
    ]
    ros_msg.is_bigendian = False
    ros_msg.point_step = 20
    ros_msg.row_step = ros_msg.point_step * ros_msg.width
    ros_msg.data = pcl_array.tostring()
    return ros_msg

def ros_to_pcl(ros_cloud):
    points_list = []
    for data in pc2.read_points(ros_cloud, skip_nans=True):
        points_list.append([data[0], data[1], data[2], data[3]])
    pcl_data = pcl.PointCloud_PointXYZRGB()
    pcl_data.from_list(points_list)
    return pcl_data
