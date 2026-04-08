#!/usr/bin/env python3
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class ImageConverter:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_pub = rospy.Publisher("/camera/rgb/image_rgb", Image, queue_size=10)
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.callback)
        
    def callback(self, data):
        try:
            # 转换 BGR 到 RGB
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            
            # 发布 RGB 图像
            ros_image = self.bridge.cv2_to_imgmsg(rgb_image, "rgb8")
            ros_image.header = data.header
            self.image_pub.publish(ros_image)
            
        except CvBridgeError as e:
            rospy.logerr("CV Bridge error: %s", e)

if __name__ == '__main__':
    rospy.init_node('image_converter')
    ic = ImageConverter()
    rospy.spin()