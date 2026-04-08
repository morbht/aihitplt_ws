#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool

class aihitplt_spray_api:
    def __init__(self):
        
        self.spray_control_pub_data = rospy.Publisher("/spray_control", Bool, queue_size=10)
        
    def spray_control(self, data):
        """
        控制喷雾消毒模块
        Args:
            data：布尔值
        """
        try:
            if not isinstance(data, bool): #检测话题是否为bool
                rospy.logwarn(f"Invalid spray control data type: {type(data)}, expected bool")
                return False           
            
            rospy.sleep(0.2)
            msg = Bool()
            msg.data = data
            self.spray_control_pub_data.publish(msg)
            action = "开启" if data else "关闭"
            rospy.loginfo(f"喷雾消毒{action}指令已发送")            
            return True
        except Exception as e:
            rospy.logerr(f"喷雾控制失败: {e}")
            return False