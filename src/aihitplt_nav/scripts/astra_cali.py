#!/usr/bin/env python3
# encoding: utf-8
import rospy
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Header

def fix_camera_info():
    rospy.init_node('camera_info_fixer')
    pub = rospy.Publisher('/camera/rgb/camera_info', CameraInfo, queue_size=10)
    
    camera_info = CameraInfo()
    camera_info.header = Header()
    camera_info.header.frame_id = "camera_color_optical_frame"
    camera_info.height = 480
    camera_info.width = 640
    camera_info.distortion_model = "plumb_bob"
    camera_info.D = [0.0, 0.0, 0.0, 0.0, 0.0]
    
    # 内参矩阵 K
    camera_info.K = [554.3827128226441, 0.0, 320.5, 
                     0.0, 554.3827128226441, 240.5, 
                     0.0, 0.0, 1.0]
    
    # 旋转矩阵 R (单位矩阵)
    camera_info.R = [1.0, 0.0, 0.0, 
                     0.0, 1.0, 0.0, 
                     0.0, 0.0, 1.0]
    
    # 投影矩阵 P
    camera_info.P = [554.3827128226441, 0.0, 320.5, 0.0,
                     0.0, 554.3827128226441, 240.5, 0.0,
                     0.0, 0.0, 1.0, 0.0]
    
    camera_info.binning_x = 0
    camera_info.binning_y = 0
    camera_info.roi.x_offset = 0
    camera_info.roi.y_offset = 0
    camera_info.roi.height = 0
    camera_info.roi.width = 0
    camera_info.roi.do_rectify = False
    
    rate = rospy.Rate(30)  # 30Hz
    
    while not rospy.is_shutdown():
        camera_info.header.stamp = rospy.Time.now()
        pub.publish(camera_info)
        rate.sleep()

if __name__ == '__main__':
    try:
        fix_camera_info()
    except rospy.ROSInterruptException:
        pass
