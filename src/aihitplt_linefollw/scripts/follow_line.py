#!/usr/bin/env python3
# encoding: utf-8
import os
import threading
import math
import rospkg
import rospy
import cv2 as cv
import numpy as np
import time
from follow_common import *
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from dynamic_reconfigure.server import Server
from dynamic_reconfigure.client import Client
from sensor_msgs.msg import CompressedImage, LaserScan, Image
from aihitplt_linefollw.cfg import LineDetectPIDConfig

# RED: 0, 85, 126, 9, 253, 255
RAD2DEG = 180 / math.pi

class LineDetect:
    def __init__(self):
        rospy.on_shutdown(self.cancel)
        rospy.init_node("LineDetect", anonymous=False)
        self.img = None
        self.circle = ()
        self.hsv_range = ()
        self.Roi_init = ()
        self.warning = 1
        self.Start_state = True
        self.dyn_update = False
        self.Buzzer_state = False
        self.select_flags = False
        self.Track_state = 'identify'
        self.windows_name = 'frame'
        self.ros_ctrl = ROSCtrl()
        self.color = color_follow()
        self.cols, self.rows = 0, 0
        self.Mouse_XY = (0, 0)
        self.img_flip = rospy.get_param("~img_flip", False)
        self.hsv_text = rospkg.RosPack().get_path("aihitplt_linefollw")+"/scripts/LineFollowHSV.text"
        
        Server(LineDetectPIDConfig, self.dynamic_reconfigure_callback)
        self.dyn_client = Client("LineDetect", timeout=60)
        self.scale = 1000
        self.FollowLinePID = (60, 0, 20)
        self.linear = 0.4
        self.LaserAngle = 30
        self.ResponseDist = 0.55
        
        self.PID_init()
        self.sub_scan = rospy.Subscriber('/scan', LaserScan, self.registerScan, queue_size=1)
        self.pub_rgb = rospy.Publisher("/linefollw/rgb", Image, queue_size=1)
        self.pub_Buzzer = rospy.Publisher('/Buzzer', Bool, queue_size=1)

        from cv_bridge import CvBridge
        self.bridge = CvBridge()
        # 使用深度相机的RGB话题
        self.sub_img = rospy.Subscriber("/camera/rgb/image_raw", Image, self.image_callback)

    def cancel(self):
        self.Reset()
        self.ros_ctrl.cancel()
        self.sub_scan.unregister()
        self.sub_img.unregister()
        self.pub_rgb.unregister()
        self.pub_Buzzer.unregister()
        print("Shutting down this node.")
        cv.destroyAllWindows()
        rospy.signal_shutdown("User pressed 'q'")
    
    def image_callback(self, msg):
        if not isinstance(msg, Image): 
            return
        
        start = time.time()
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logerr("Error converting image: %s", str(e))
            return
            
        action = cv.waitKey(1) & 0xFF
        rgb_img, binary = self.process(frame, action)
        end = time.time()
        fps = 1 / (end - start)
        text = "FPS : " + str(int(fps))
        cv.putText(rgb_img, text, (30, 30), cv.FONT_HERSHEY_SIMPLEX, 0.6, (100, 200, 200), 1)
        
        if len(binary) != 0: 
            cv.imshow(self.windows_name, ManyImgs(1, ([rgb_img, binary])))
        else: 
            cv.imshow(self.windows_name, rgb_img)
            
        try:
            self.pub_rgb.publish(self.bridge.cv2_to_imgmsg(rgb_img, "bgr8"))
        except Exception as e:
            rospy.logerr("Error publishing image: %s", str(e))

    def process(self, rgb_img, action):
        binary = []
        rgb_img = cv.resize(rgb_img, (640, 480))
        
        if self.img_flip: 
            rgb_img = cv.flip(rgb_img, 1)
            
        if action == 32: 
            self.Track_state = 'tracking'
        elif action == ord('i') or action == 105: 
            self.Track_state = "identify"
        elif action == ord('r') or action == 114: 
            self.Reset()
        elif action == ord('q') or action == 113: 
            self.cancel()
            
        if self.Track_state == 'init':
            cv.namedWindow(self.windows_name, cv.WINDOW_AUTOSIZE)
            cv.setMouseCallback(self.windows_name, self.onMouse, 0)
            if self.select_flags:
                cv.line(rgb_img, self.cols, self.rows, (255, 0, 0), 2)
                cv.rectangle(rgb_img, self.cols, self.rows, (0, 255, 0), 2)
                if self.Roi_init[0] != self.Roi_init[2] and self.Roi_init[1] != self.Roi_init[3]:
                    rgb_img, self.hsv_range = self.color.Roi_hsv(rgb_img, self.Roi_init)
                    self.dyn_update = True
                else: 
                    self.Track_state = 'init'
                    
        elif self.Track_state == "identify":
            if os.path.exists(self.hsv_text): 
                self.hsv_range = read_HSV(self.hsv_text)
            else: 
                self.Track_state = 'init'
                
        if self.Track_state != 'init' and len(self.hsv_range) != 0:
            rgb_img, binary, self.circle = self.color.line_follow(rgb_img, self.hsv_range)
            if self.dyn_update:
                write_HSV(self.hsv_text, self.hsv_range)
                params = {'Hmin': self.hsv_range[0][0], 'Hmax': self.hsv_range[1][0],
                          'Smin': self.hsv_range[0][1], 'Smax': self.hsv_range[1][1],
                          'Vmin': self.hsv_range[0][2], 'Vmax': self.hsv_range[1][2]}
                self.dyn_client.update_configuration(params)
                self.dyn_update = False
                
        if self.Track_state == 'tracking':
            if len(self.circle) != 0:
                threading.Thread(target=self.execute, args=(self.circle[0], self.circle[2])).start()
        else:
            if self.Start_state:
                self.ros_ctrl.pub_cmdVel.publish(Twist())
                self.Start_state = False
                
        return rgb_img, binary

    def onMouse(self, event, x, y, flags, param):
        if event == 1:  # CV_EVENT_LBUTTONDOWN
            self.Track_state = 'init'
            self.select_flags = True
            self.Mouse_XY = (x, y)
        if event == 4:  # CV_EVENT_LBUTTONUP
            self.select_flags = False
            self.Track_state = 'mouse'
        if self.select_flags:
            self.cols = (min(self.Mouse_XY[0], x), min(self.Mouse_XY[1], y))
            self.rows = (max(self.Mouse_XY[0], x), max(self.Mouse_XY[1], y))
            self.Roi_init = (self.cols[0], self.cols[1], self.rows[0], self.rows[1])

    def execute(self, point_x, color_radius):
        if self.ros_ctrl.Joy_active:
            if self.Start_state:
                self.PID_init()
                self.Start_state = False
            return
        self.Start_state = True
        if color_radius == 0: 
            self.ros_ctrl.pub_cmdVel.publish(Twist())
        else:
            twist = Twist()
            b = Bool()
            [z_Pid, _] = self.PID_controller.update([(point_x - 320)/16, 0])
            if self.img_flip: 
                twist.angular.z = -z_Pid
            else: 
                twist.angular.z = z_Pid
            twist.linear.x = self.linear
            if self.warning > 10:
                rospy.loginfo("Obstacles ahead !!!")
                self.ros_ctrl.pub_cmdVel.publish(Twist())
                self.Buzzer_state = True
                b.data = True
                self.pub_Buzzer.publish(b)
            else:
                if self.Buzzer_state:
                    b.data = False
                    for i in range(3): 
                        self.pub_Buzzer.publish(b)
                    self.Buzzer_state = False
                self.ros_ctrl.pub_cmdVel.publish(twist)

    def Reset(self):
        self.PID_init()
        self.Track_state = 'init'
        self.hsv_range = ()
        self.ros_ctrl.Joy_active = False
        self.Mouse_XY = (0, 0)
        self.ros_ctrl.pub_cmdVel.publish(Twist())
        rospy.loginfo("Reset succes!!!")

    def PID_init(self):
        self.PID_controller = simplePID(
            [0, 0],
            [self.FollowLinePID[0] / 1.0 / (self.scale), 0],
            [self.FollowLinePID[1] / 1.0 / (self.scale), 0],
            [self.FollowLinePID[2] / 1.0 / (self.scale), 0])

    def dynamic_reconfigure_callback(self, config, level):
        self.scale = config['scale']
        self.linear = config['linear']
        self.LaserAngle = config['LaserAngle']
        self.ResponseDist = config['ResponseDist']
        self.FollowLinePID = (config['Kp'], config['Ki'], config['Kd'])
        self.hsv_range = ((config['Hmin'], config['Smin'], config['Vmin']),
                          (config['Hmax'], config['Smax'], config['Vmax']))
        write_HSV(self.hsv_text, self.hsv_range)
        print("HSV: ", self.hsv_range)
        self.PID_init()
        return config

    def registerScan(self, scan_data):
        self.warning = 1
        if not isinstance(scan_data, LaserScan): 
            return
        if self.ros_ctrl.Joy_active: 
            return
        ranges = np.array(scan_data.ranges)
        for i in range(len(ranges)):
            angle = (scan_data.angle_min + scan_data.angle_increment * i) * RAD2DEG
            if abs(angle) > (180 - self.LaserAngle):
                if ranges[i] < self.ResponseDist: 
                    self.warning += 1

if __name__ == '__main__':
    line_detect = LineDetect()
    rospy.spin()
    cv.destroyAllWindows()
