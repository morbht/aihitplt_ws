#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import UInt32, Bool

class Estop:
    def __init__(self):
        rospy.init_node('e_stop_detector', anonymous=True)
        
        self.emergency_value = 2359296  
        self.e_stop_state = False 
        self.first_message = True 
        
        self.estop_pub = rospy.Publisher('/e_stop', Bool, queue_size=10, latch=True)
        
        rospy.Subscriber('/self_check_data', UInt32, self.self_check_callback)
        
        #初始发布一次状态
        self.publish_estop_state(False)
    
    def self_check_callback(self, msg):

        value = msg.data
        
        is_emergency = (value == self.emergency_value)
        
        if self.first_message:
            self.first_message = False
            self.e_sotp_state = is_emergency
            self.publish_estop_state(is_emergency)

        elif is_emergency != self.e_stop_state:
            self.e_stop_state = is_emergency
            self.publish_estop_state(is_emergency)
            
    
    def publish_estop_state(self, state):
        msg = Bool()
        msg.data = state
        self.estop_pub.publish(msg)
    
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        detector = Estop()
        detector.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("节点关闭")