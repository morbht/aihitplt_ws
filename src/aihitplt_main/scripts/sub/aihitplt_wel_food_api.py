#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool

class aihitplt_wel_food_api:
    def estop_state(self):
        """
        送餐与迎宾模块急停状态获取
        Returns:
            bool: 返回布尔值
        """
        try:
            estop_msg = rospy.wait_for_message("/guide_delivery_emergency_button",Bool,timeout=1.0)
            estop_data = estop_msg.data
            return estop_data
            
        except Exception as e:
            pass
        
    