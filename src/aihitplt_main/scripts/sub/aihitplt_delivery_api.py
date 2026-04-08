#!/usr/bin/env python3

import rospy
from std_msgs.msg import String,Int32,Bool

class aihitplt_delivery_api:
    def __init__(self):
        # 上舱门话题
        self.upper_control_pub = rospy.Publisher("/upper_control_cmd",String,queue_size=10)
        # 下舱门话题
        self.lower_control_pub = rospy.Publisher("/lower_control_cmd",String,queue_size=10)
        # 上舱门复位话题
        self.upper_reset_pub = rospy.Publisher("/upper_reset_cmd",String,queue_size=10)
        # 下舱门复位话题
        self.lower_reset_pub = rospy.Publisher("/lower_reset_cmd",String,queue_size=10)
        # 初始化话题
        self.init_sys_pub = rospy.Publisher("/delivery_init_cmd",String,queue_size=10)
        
        
        # rospy.Subscriber("/delivery_device_state",Int32,self._door_state_callback)
        
    def upper_control(self,command):
        """
        上舱门控制操作
        Args:
            command (str): 输入方向与距离，例如:"up,10.0"
        Returns:
            bool:布尔值
        """
        try:
            control_upper_msg = String()
            control_upper_msg.data = command
            
            self.upper_control_pub.publish(control_upper_msg)
            return True
        
        except Exception as e:
            return False
        
    def upper_reset(self,command):
        """
        上舱门复位操作
        Args:
            command (str): 输入方向与距离，例如:"up,220"
        Returns:
            bool:布尔值
        """
        try:
            control_upper_msg = String()
            control_upper_msg.data = command
            
            self.upper_reset_pub.publish(control_upper_msg)
            return True
        
        except Exception as e:
            return False   
    
    def lower_control(self,command):
        """
        控制下舱门
        Args:
            command (str): 输入方向与距离，例如:"up,10.0"
        """
        try:
            control_lower_msg = String()
            control_lower_msg.data = command
            
            self.lower_control_pub.publish(control_lower_msg)
            return True
        
        except Exception as e:
            return False   
        
    def lower_reset(self,command):
        """_summary_

        Args:
            command (str): 输入方向与距离，例如:"up,220"
        """
        try:
            lower_reset_msg = String()
            lower_reset_msg.data = command
            
            self.lower_reset_pub.publish(lower_reset_msg)
            return True
        
        except Exception as e:
            return False       
        
    def initialize_system(self,command):
        """
        上下舱门初始化
        Args:
            command (str): 需要输入两个参数，上下舱门的距离，例如"220,220"
        Return:
            bool:布尔值
        """
        try:
            data_msg = String()
            data_msg.data = command  
            self.init_sys_pub.publish(data_msg)          
            return True
        except Exception as e:
            return False
    
    def get_door_state(self, door_type):
        """
        获取上下舱门状态
        Args:
            door_type (str): "upper" - 上舱门, "lower" - 下舱门
        Returns:
            dict: 包含位置、限位状态、完成状态等信息的状态字典
        """
        try:
            if door_type.lower() == "upper":
                # 上舱门状态
                return {
                    'motor_enabled': self._get_topic_state('upper_motor_state', Bool),
                    'up_limit': self._get_topic_state('upper_up_limit_state', Bool),
                    'down_limit': self._get_topic_state('upper_down_limit_state', Bool),
                    'type': 'upper'
                }
            
            elif door_type.lower() == "lower":
                # 下舱门状态
                return {
                    'motor_enabled': self._get_topic_state('lower_motor_state', Bool),
                    'up_limit': self._get_topic_state('lower_up_limit_state', Bool),
                    'down_limit': self._get_topic_state('lower_down_limit_state', Bool),
                    'type': 'lower'
                }
            else:
                rospy.logwarn(f"未知舱门类型: {door_type}")
                return {'error': 'invalid_door_type'}
                
        except Exception as e:
            rospy.logerr(f"获取舱门状态失败: {e}")
            return {'error': str(e)}
        
    def get_system_state(self):
        """
        获取系统状态
        Returns:
            int: 状态码 (0-11)
            状态码对应关系:
            0: 正常
            1: 初始化中
            2: 急停中
            3: 电机重置中
            4: 上门上复位
            5: 上门下复位
            6: 下门上复位
            7: 下门下复位
            8: 上门向上移动
            9: 上门向下移动
            10: 下门向上移动
            11: 下门向下移动
        """        
        try:
            state_msg = rospy.wait_for_message("/delivery_device_state",Int32,timeout=1.0)
            state_code = state_msg.data
            if 0 <= state_code <=11:
                return state_code
            else:
                rospy.logwarn(f"获取到异常状态码: {state_code}")
                return -1
        except Exception as e:
            return -3 
    
    def estop_state(self):
        """
        获取急停状态
        Returns:
            bool: 布尔值
        """
        
        try:
            estop_msg = rospy.wait_for_message("/emergency_stop",Bool,timeout=1.0)
            estop_state = estop_msg.data
            if estop_state == True:
                return True
            else:
                return False
    
        except Exception as e:
            pass
        
        
    # 以下为处理函数
    def _get_topic_state(self, topic_name, msg_type):
        """
        获取指定话题的最新状态值
        Args:
            topic_name (str): 话题名称
            msg_type: 消息类型
        Returns:
            话题的最新状态值，如果获取失败返回None
        """
        try:
            # 等待话题数据（最多等待1秒）
            msg = rospy.wait_for_message(topic_name, msg_type, timeout=1.0)
            return msg.data
        except rospy.ROSException as e:
            rospy.logwarn(f"获取话题 {topic_name} 状态超时: {e}")
            return None
        except Exception as e:
            rospy.logerr(f"获取话题 {topic_name} 状态失败: {e}")
            return None

        










