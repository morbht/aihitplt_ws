#!/usr/bin/env python3
import rospy
import serial
import threading
import yaml
from std_msgs.msg import Bool, String

class RelayBridge:
    def __init__(self):
        # 从YAML读取端口配置
        with open('/home/aihit/aihitplt_ws/src/aihitplt_hardware_test/config/spray_port.yaml', 'r') as f:
            config = yaml.safe_load(f)
            self.port = config['port']
            self.baudrate = config['baudrate']
        
        # 初始化串口
        self.ser = serial.Serial(self.port, self.baudrate, timeout=1)
        rospy.loginfo("Connected on %s", self.port)
        
        # 发布器和订阅器
        self.status_pub = rospy.Publisher('spray_status', String, queue_size=10)
        self.control_sub = rospy.Subscriber('spray_control', Bool, self.control_callback)
        
        # 启动串口读取线程
        self.read_thread = threading.Thread(target=self.read_serial)
        self.read_thread.daemon = True
        self.read_thread.start()
        
        rospy.loginfo("Relay Bridge started")
    
    def control_callback(self, msg):
        try:
            self.ser.write(b'on\n' if msg.data else b'off\n')
        except Exception as e:
            rospy.logerr("Error sending command: %s", e)
    
    def read_serial(self):
        while not rospy.is_shutdown() and self.ser.is_open:
            try:
                if self.ser.in_waiting > 0:
                    line = self.ser.readline().decode('utf-8').strip()
                    if line:
                        self.status_pub.publish(String(data=line))
            except:
                rospy.sleep(1)
    
    def cleanup(self):
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()

def main():
    rospy.init_node('spray_bridge')
    bridge = RelayBridge()
    rospy.on_shutdown(bridge.cleanup)
    rospy.spin()

if __name__ == '__main__':
    main()