#!/usr/bin/env python3
import rospy
import serial
import threading
from std_msgs.msg import Bool, String

class  RelayBridge:
    def __init__(self):
        # ROS参数
        self.port = rospy.get_param('~port', '/dev/ttyUSB1')
        self.baudrate = rospy.get_param('~baudrate', 115200)
        
        # 初始化串口连接
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=1)
            rospy.loginfo("Connected  on %s", self.port)
        except serial.SerialException as e:
            rospy.logerr("Failed to connect: %s", e)
            return
        
        # 发布器和订阅器
        self.status_pub = rospy.Publisher('spray_status', String, queue_size=10)
        self.control_sub = rospy.Subscriber('spray_control', Bool, self.control_callback)
        
        # 启动串口读取线程
        self.read_thread = threading.Thread(target=self.read_serial)
        self.read_thread.daemon = True
        self.read_thread.start()
        
        rospy.loginfo("  Relay Bridge started")
    
    def control_callback(self, msg):
        """处理继电器控制命令"""
        try:
            if msg.data:
                self.ser.write(b'on\n')
                rospy.loginfo("Sent command: ON")
            else:
                self.ser.write(b'off\n')
                rospy.loginfo("Sent command: OFF")
        except Exception as e:
            rospy.logerr("Error sending command to  : %s", e)
    
    def read_serial(self):
        """读取串口数据并发布状态"""
        while not rospy.is_shutdown() and self.ser.is_open:
            try:
                if self.ser.in_waiting > 0:
                    line = self.ser.readline().decode('utf-8').strip()
                    if line:
                        rospy.loginfo(" : %s", line)
                        
                        # 发布状态消息
                        status_msg = String()
                        status_msg.data = line
                        self.status_pub.publish(status_msg)
            except Exception as e:
                rospy.logwarn("Error reading from serial: %s", e)
                rospy.sleep(1)
    
    def cleanup(self):
        """清理资源"""
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()
            rospy.loginfo("Serial port closed")

def main():
    rospy.init_node('spray_bridge')
    bridge =  RelayBridge()
    
    # 设置关闭节点时的清理函数
    rospy.on_shutdown(bridge.cleanup)
    
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        bridge.cleanup()

if __name__ == '__main__':
    main()
