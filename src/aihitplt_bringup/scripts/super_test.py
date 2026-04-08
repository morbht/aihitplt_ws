#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Bool

class ButtonTester:
    def __init__(self):
        rospy.init_node('button_tester', anonymous=True)
        
        # 订阅按钮话题
        self.button1_sub = rospy.Subscriber("/user_button", Bool, self.button1_callback)
        self.button2_sub = rospy.Subscriber("/collision_sensor", Bool, self.button2_callback)
        
        # 按钮状态
        self.button1_state = False
        self.button2_state = False
        
        print("按钮测试程序启动...")
        print("等待按钮数据...")
        print("按 Ctrl+C 退出")
        
    def button1_callback(self, msg):
        if msg.data != self.button1_state:
            self.button1_state = msg.data
            status = "按下" if msg.data else "释放"
            print(f"🎛️  用户按钮: {status}")
    
    def button2_callback(self, msg):
        if msg.data != self.button2_state:
            self.button2_state = msg.data
            status = "触发" if msg.data else "正常"
            print(f"🛡️  防撞传感器: {status}")
    
    def run(self):
        # 每秒显示一次当前状态
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            print(f"\r当前状态 - 用户按钮: {'按下' if self.button1_state else '释放'}, "
                  f"防撞传感器: {'触发' if self.button2_state else '正常'}", end="")
            rate.sleep()

if __name__ == '__main__':
    try:
        tester = ButtonTester()
        tester.run()
    except rospy.ROSInterruptException:
        print("\n测试结束")