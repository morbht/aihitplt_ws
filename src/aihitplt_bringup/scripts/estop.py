#!/usr/bin/env python3
import rospy
from std_msgs.msg import UInt32, Bool

rospy.init_node('emergency_stop')
pub = rospy.Publisher('/emergency_active', Bool, queue_size=1)

def callback(msg):
    is_emergency = bool(msg.data & (1 << 21))  # 检查第21位
    pub.publish(Bool(data=is_emergency))
    if is_emergency:
        rospy.logwarn("急停激活")

rospy.Subscriber("/self_check_data", UInt32, callback)
rospy.spin()