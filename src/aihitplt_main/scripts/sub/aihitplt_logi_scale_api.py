#!/usr/bin/env python3 

import rospy
from std_msgs.msg import String,Bool,Float32


class aihitplt_logi_scale_api:
    def __init__(self) -> None:
        # 传感器控制话题发布
        self.control_pub = rospy.Publisher("/logi_scale/control",String,queue_size=10)

    
    
    
    def get_weight(self):
        """
        获取重量
        Returns:
            float:返回重量值
        """
        try:
            weight_msg = rospy.wait_for_message("/logi_scale/weight",Float32,timeout=1.0)
            weight_data = weight_msg.data
            return weight_data
            
        except Exception as e:
            pass
        
    def estop_state(self):
        
        try:
            estop_msg = rospy.wait_for_message("/logi_scale/emergency_stop",Bool,timeout=1.0)
            estop_data = estop_msg.data
            return estop_data
        except Exception as e:
            pass

    def tare_scale(self):
        """
        执行归零操作
        Returns:
            bool: 返回布尔值
        """
        try:
            pub_msg = "z"
            self.control_pub.publish(pub_msg)
            return True
        except Exception as e:
            return False
        
        
    def reset_scale(self):
        """
        执行设备重置操作
        Returns:
            bool: 返回布尔值
        """
        try:
            msg = "r"
            self.control_pub.publish(msg)
            return True
        except Exception as e:
            return False
        
    def cali_scare(self,cali_weight):
        """
        校准称重传感器
        Args:
            cali_weight (float): 校准正确重量

        Returns:
            bool: 返回布尔值
        """
        try:
            cali_msg = Float32()
            cali_msg.data = cali_weight
            # cali_data = f"c{cali_msg.data}"
            self.control_pub.publish(f"c{cali_msg.data}")
            
            return True
        except Exception as e:
            return False    
        
        
    