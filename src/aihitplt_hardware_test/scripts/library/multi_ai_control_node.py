#!/usr/bin/env python3
"""
pan_control.py
通过话题控制云台
"""
import rospy
import serial
import time
import threading
from std_msgs.msg import String

class PanTiltTopicController:
    def __init__(self):
        rospy.init_node('multi_ai_pan_control', anonymous=True)

        port  = rospy.get_param('~port',  '/dev/ttyUSB2')
        baud  = rospy.get_param('~baud',  115200)

        # 初始化串口
        self.ser = serial.Serial(port, baud, timeout=1)
        time.sleep(2)
        rospy.loginfo(f"串口打开: {port} @ {baud}")

        # 当前角度缓存
        self.current_h = 90
        self.current_v = 90

        # 订阅控制命令
        rospy.Subscriber('/multi_ai/pan_control', String, self.cmd_callback)

        rospy.loginfo("等待 /multi_ai/pan_control 话题命令 (格式: 'h:90 v:45' 或 '90,45')")

    # ---------- 话题回调 ----------
    def cmd_callback(self, msg):
        """解析字符串命令并控制舵机"""
        try:
            h, v = None, None
            raw = msg.data.strip()

            # 格式1: "90,45"
            if ',' in raw:
                h_str, v_str = raw.split(',', 1)
                h, v = int(h_str), int(v_str)
            # 格式2: "h:90 v:45" 或单独 "h:90"
            else:
                parts = raw.split()
                for p in parts:
                    if p.startswith('h:'):
                        h = int(p[2:])
                    elif p.startswith('v:'):
                        v = int(p[2:])

            # 缺省值用当前角度
            h = h if h is not None else self.current_h
            v = v if v is not None else self.current_v

            # 限幅
            h = max(0, min(180, h))
            v = max(0, min(180, v))

            # 发串口
            cmd_str = f"P:{h},{v}\n"
            self.ser.write(cmd_str.encode())
            self.current_h, self.current_v = h, v
            rospy.loginfo(f"云台命令: {cmd_str.strip()}")
        except Exception as e:
            rospy.logwarn(f"控制命令解析失败: {e}")

    # ---------- 节点关闭 ----------
    def on_shutdown(self):
        rospy.loginfo("节点关闭，回到中位")
        self.ser.write(b"P:90,90\n")
        time.sleep(0.2)
        self.ser.close()


if __name__ == '__main__':
    try:
        controller = PanTiltTopicController()
        rospy.on_shutdown(controller.on_shutdown)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass