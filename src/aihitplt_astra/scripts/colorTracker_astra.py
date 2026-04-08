#!/usr/bin/env python3
# coding: utf-8
import time
import rospy
import cv2 as cv
import numpy as np
from astra_common import simplePID
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from aihitplt_msgs.msg import Position
from dynamic_reconfigure.server import Server
from aihitplt_astra.cfg import ColorTrackerPIDConfig

class color_Tracker:
    def __init__(self):
        rospy.on_shutdown(self.cleanup)
        self.minDist = 1000
        self.Center_x = 0
        self.Center_y = 0
        self.Center_r = 0
        self.Center_prevx = 0
        self.Center_prevr = 0
        self.prev_time = 0
        self.prev_dist = 0
        self.prev_angular = 0
        self.Robot_Run = False
        self.dist = []
        self.sub_depth = rospy.Subscriber("/camera/depth/image_raw", Image, self.depth_img_Callback, queue_size=1)
        self.sub_position = rospy.Subscriber("/Current_point", Position, self.positionCallback)
        self.pub_cmdVel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        Server(ColorTrackerPIDConfig, self.AstraFollowPID_callback)
        self.linear_PID = (3.0, 0.0, 1.0)
        self.angular_PID = (0.5, 0.0, 2.0)
        self.scale = 1000
        self.PID_init()

    def AstraFollowPID_callback(self, config, level):
        self.linear_PID = (config['linear_Kp'], config['linear_Ki'], config['linear_Kd'])
        self.angular_PID = (config['angular_Kp'], config['angular_Ki'], config['angular_Kd'])
        self.minDist = config['minDist'] * 1000
        print("linear_PID: ", self.linear_PID)
        print("angular_PID: ", self.angular_PID)
        self.PID_init()
        return config

    def PID_init(self):
        self.linear_pid = simplePID(self.linear_PID[0] / 1000.0, self.linear_PID[1] / 1000.0, self.linear_PID[2] / 1000.0)
        self.angular_pid = simplePID(self.angular_PID[0] / 100.0, self.angular_PID[1] / 100.0, self.angular_PID[2] / 100.0)

    def depth_img_Callback(self, msg):
        if not isinstance(msg, Image): 
            return

        # 直接解析深度数据（16UC1 格式，单位：毫米）
        depth_data = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width)

        self.action = cv.waitKey(1)
        if self.Center_r != 0:
            now_time = time.time()
            if now_time - self.prev_time > 5:
                if self.Center_prevx == self.Center_x and self.Center_prevr == self.Center_r: 
                    self.Center_r = 0
                self.prev_time = now_time
            
            # 边界检查
            if (0 <= int(self.Center_y) - 3 < msg.height and 
                0 <= int(self.Center_y) + 3 < msg.height and 
                0 <= int(self.Center_x) - 3 < msg.width and 
                0 <= int(self.Center_x) + 3 < msg.width):
                
                distance = [
                    depth_data[int(self.Center_y) - 3, int(self.Center_x) - 3],
                    depth_data[int(self.Center_y) + 3, int(self.Center_x) - 3],
                    depth_data[int(self.Center_y) - 3, int(self.Center_x) + 3],
                    depth_data[int(self.Center_y) + 3, int(self.Center_x) + 3],
                    depth_data[int(self.Center_y), int(self.Center_x)]
                ]
                
                # 计算有效距离
                distance_ = 0.0
                num_depth_points = 0
                for d in distance:
                    if 40 < d < 80000:  # 过滤无效值
                        distance_ += d
                        num_depth_points += 1
                
                if num_depth_points == 0:
                    distance_ = self.minDist
                else:
                    distance_ /= num_depth_points
                
                print(f"Center_x: {self.Center_x}, Center_y: {self.Center_y}, distance_: {distance_}mm")
                self.execute(self.Center_x, distance_)
                self.Center_prevx = self.Center_x
                self.Center_prevr = self.Center_r
        else:
            if self.Robot_Run:
                self.pub_cmdVel.publish(Twist())
                self.Robot_Run = False
        
        if self.action == ord('q') or self.action == 113: 
            self.cleanup()

    def execute(self, point_x, dist):
        if abs(self.prev_dist - dist) > 300:
            self.prev_dist = dist
            return
        if abs(self.prev_angular - point_x) > 300:
            self.prev_angular = point_x
            return

        linear_x = self.linear_pid.compute(dist, self.minDist)
        angular_z = self.angular_pid.compute(320, point_x)
        if abs(dist - self.minDist) < 30: 
            linear_x = 0
        if abs(point_x - 320.0) < 30: 
            angular_z = 0
        twist = Twist()
        twist.angular.z = angular_z
        twist.linear.x = 0.3 * linear_x
        self.pub_cmdVel.publish(twist)
        self.Robot_Run = True

    def positionCallback(self, msg):
        if not isinstance(msg, Position): 
            return
        self.Center_x = msg.angleX
        self.Center_y = msg.angleY
        self.Center_r = msg.distance

    def cleanup(self):
        self.pub_cmdVel.publish(Twist())
        self.sub_depth.unregister()
        self.sub_position.unregister()
        self.pub_cmdVel.unregister()
        print("Shutting down this node.")
        cv.destroyAllWindows()

if __name__ == '__main__':
    rospy.init_node("color_Tracker", anonymous=False)
    tracker = color_Tracker()
    rospy.spin()
