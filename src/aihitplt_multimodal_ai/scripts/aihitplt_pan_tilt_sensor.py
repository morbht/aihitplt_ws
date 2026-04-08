#!/usr/bin/env python3
import rospy
import serial
import struct
import time

FRAME_FORMAT = '<2sI8H2f6fB'
FRAME_SIZE = 55

class ESP32Node:
    def __init__(self, port, baudrate):
        rospy.init_node('esp32_sensor_node', anonymous=True)
        try:
            self.ser = serial.Serial(port, baudrate, timeout=1)
            time.sleep(2)
            rospy.loginfo(f"成功连接串口: {port} @ {baudrate}")
        except serial.SerialException as e:
            rospy.logerr(f"串口连接失败: {e}")
            exit(1)

    def read_data(self):
        """寻找帧头并解析一帧数据"""
        while not rospy.is_shutdown():
            if self.ser.read(1) == b'\xAA':
                if self.ser.read(1) == b'\x55':
                    data = self.ser.read(FRAME_SIZE - 2)
                    if len(data) == 53:
                        frame = b'\xAA\x55' + data

                        if (sum(frame[:-1]) & 0xFF) == frame[-1]:
                            return struct.unpack(FRAME_FORMAT, frame)
        return None

    def set_servo(self, angle):
        """发送舵机控制指令"""
        if 0 <= angle <= 180:
            self.ser.write(f"P:{angle},{angle}\n".encode())

    def run(self):
        angle = 90
        rate = rospy.Rate(0.5)  
        
        while not rospy.is_shutdown():
   
            u = self.read_data()
            if u:

                rospy.loginfo(f"温湿度: {u[10]:.1f}℃, {u[11]:.1f}% | "
                              f"PM2.5: {u[8]} | CO2: {u[5]} | TVOC: {u[7]}")
            
 
            angle = 45 if angle == 90 else 90
            self.set_servo(angle)
            rospy.loginfo(f"已发送舵机角度: {angle}°")
            
            rate.sleep()

    def cleanup(self):
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()

if __name__ == '__main__':
    # 从 ROS 参数服务器获取串口配置，若无则使用默认值
    port = rospy.get_param('~port', '/dev/ttyUSB2')
    baud = rospy.get_param('~baud', 115200)
    
    node = ESP32Node(port, baud)
    try:
        node.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        node.cleanup()