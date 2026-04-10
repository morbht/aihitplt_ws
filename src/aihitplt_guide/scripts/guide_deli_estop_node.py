#!/usr/bin/env python3
import rospy
import serial
from std_msgs.msg import Bool

def aihitplt_guide_delivery_emergency_node():
    """急停按钮ROS节点：读取串口急停状态并发布"""
    rospy.init_node('aihitplt_guide_delivery_emergency_node')

    port = rospy.get_param('~serial_port', '/dev/ttyUSB2')
    baudrate = rospy.get_param('~baudrate', 115200)
    rospy.loginfo(f"使用串口: {port} @ {baudrate}")

    # 创建发布者
    pub = rospy.Publisher('/e_stop', Bool, queue_size=10)

    try:
        # 连接串口
        ser = serial.Serial(port, baudrate, timeout=1)
        rospy.loginfo(f"已连接到 {port}")

        # 主循环
        while not rospy.is_shutdown():
            if ser.in_waiting:
                data = ser.readline().decode('utf-8').strip()

                if data == "P":  # 急停按下
                    pub.publish(Bool(True))
                elif data == "R":  # 急停释放
                    pub.publish(Bool(False))

    except serial.SerialException as e:
        rospy.logerr(f"串口错误: {e}")
    except rospy.ROSInterruptException:
        pass
    finally:
        if 'ser' in locals():
            ser.close()

if __name__ == '__main__':
    aihitplt_guide_delivery_emergency_node()