#!/usr/bin/env python3
# coding:utf-8
import rospy
import cv2 as cv
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs import point_cloud2
from sensor_msgs.msg import LaserScan, Image
from laser_geometry import LaserProjection


class pt2brid_eye:
    def __init__(self):
        self.bridge = CvBridge()
        self.laserProj = LaserProjection()
        self.laserSub = rospy.Subscriber("/scan", LaserScan, self.laserCallback)
        self.image_pub = rospy.Publisher('/laserImage', Image, queue_size=1)
        
        # 获取是否显示图像的参数，默认显示
        self.display_image = rospy.get_param("~display_image", True)
        
        # 图像尺寸参数
        self.img_width = 1200
        self.img_height = 1600
        self.scale = 80  # 缩放因子
        self.offset = 500  # 偏移量

    def laserCallback(self, data):
        try:
            cloud_out = self.laserProj.projectLaser(data)
            lidar = point_cloud2.read_points(cloud_out)
            points = np.array(list(lidar))
            
            if len(points) == 0:
                rospy.logwarn("No points received")
                return
                
            img = self.pointcloud_to_laserImage(points)
            img = cv.resize(img, (640, 480))
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(img, encoding="mono8"))

            if self.display_image:
                cv.imshow("Laser Image", img)
                cv.waitKey(1)
        except Exception as e:
            rospy.logerr("Error in laserCallback: %s", str(e))

    def pointcloud_to_laserImage(self, points):
        try:
            # 提取XYZ坐标
            x_points = points[:, 0]
            y_points = points[:, 1]
            z_points = points[:, 2]
            
            # 过滤点云范围
            x_filter = np.logical_and(x_points > -50, x_points < 50)
            y_filter = np.logical_and(y_points > -50, y_points < 50)
            valid_filter = np.logical_and(x_filter, y_filter)
            
            # 应用过滤
            x_points = x_points[valid_filter]
            y_points = y_points[valid_filter]
            z_points = z_points[valid_filter]
            
            if len(x_points) == 0:
                return np.zeros((self.img_height, self.img_width), dtype=np.uint8)
            
            # 转换到图像坐标
            # 注意：图像坐标中x是列，y是行
            x_img = (-y_points * self.scale).astype(np.int32) + self.offset  # 列坐标
            y_img = (-x_points * self.scale).astype(np.int32) + self.offset  # 行坐标
            
            # 将z坐标转换为像素值 (0-255)
            pixel_values = np.clip(z_points, -2, 2)
            pixel_values = ((pixel_values + 2) / 4.0) * 255  # 映射到0-255
            pixel_values = pixel_values.astype(np.uint8)
            
            # 创建空白图像
            img = np.zeros((self.img_height, self.img_width), dtype=np.uint8)
            
            # 绘制点
            for i in range(len(x_img)):
                x = x_img[i]
                y = y_img[i]
                val = pixel_values[i]
                
                # 确保坐标在图像范围内
                if 0 <= x < self.img_width and 0 <= y < self.img_height:
                    # 绘制圆形点
                    cv.circle(img, (x, y), radius=3, color=int(val), thickness=-1)
            
            return img
            
        except Exception as e:
            rospy.logerr("Error in pointcloud_to_laserImage: %s", str(e))
            import traceback
            rospy.logerr(traceback.format_exc())
            return np.zeros((self.img_height, self.img_width), dtype=np.uint8)


if __name__ == '__main__':
    print("OpenCV version: {}".format(cv.__version__))
    rospy.init_node('laser_to_image', anonymous=False)
    
    try:
        pt2img = pt2brid_eye()
        rospy.loginfo("Laser to Image node started successfully")
        rospy.spin()
    except rospy.ROSInterruptException:
        cv.destroyAllWindows()
        pass