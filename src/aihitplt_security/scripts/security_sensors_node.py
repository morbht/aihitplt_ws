#!/usr/bin/env python3
import rospy
import serial
import struct
import json
from std_msgs.msg import String

FRAME_FORMAT = '<2sI 4H B 5H 2f B'
FRAME_SIZE = 34

def security_sensors_node():
    rospy.init_node('security_sensors_node')
    
    # 
    port = rospy.get_param('~port', '/dev/ttyUSB2')
    baudrate = 115200
    
    pub = rospy.Publisher('/security_sensors', String, queue_size=10)
    
    try:
        ser = serial.Serial(port, baudrate, timeout=0.01)
        rospy.loginfo(f"Connected to {port} @ {baudrate}")
        
        buffer = bytearray()
        
        while not rospy.is_shutdown():
            # 读取串口数据
            if ser.in_waiting:
                new_data = ser.read(ser.in_waiting)
                buffer.extend(new_data)
            
            # 解析完整数据帧
            while len(buffer) >= FRAME_SIZE:
                # 查找帧头
                if buffer[0:2] != b'\xAA\x55':
                    buffer.pop(0)
                    continue
                
                frame = bytes(buffer[:FRAME_SIZE])
                
                try:
                    data = struct.unpack(FRAME_FORMAT, frame)
                    checksum = sum(frame[:-1]) & 0xFF
                    
                    if checksum == data[14]:
                        # 构建传感器数据字典
                        sensor_dict = {
                            'alcohol': int(data[2]),
                            'smoke': int(data[3]),
                            'light': int(data[4]),
                            'sound': int(data[5]),
                            'emergency_stop': int(data[6]),
                            'eCO2': int(data[7]),
                            'eCH2O': float(data[8] / 100.0),
                            'TVOC': float(data[9] / 100.0),
                            'PM25': int(data[10]),
                            'PM10': int(data[11]),
                            'temperature': float(data[12]),
                            'humidity': float(data[13])
                        }
                        
                        # 发布JSON数据
                        msg = String()
                        msg.data = json.dumps(sensor_dict)
                        pub.publish(msg)
                        
                        # 可选：打印调试信息
                        rospy.logdebug(f"Published: {sensor_dict}")
                    
                    # 移除已处理的帧
                    buffer = buffer[FRAME_SIZE:]
                    
                except struct.error:
                    # 解析失败，移除一个字节
                    buffer.pop(0)
                    
    except serial.SerialException as e:
        rospy.logerr(f"Serial error: {e}")
    except Exception as e:
        rospy.logerr(f"Error: {e}")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()
            rospy.loginfo("Serial port closed")

if __name__ == '__main__':
    try:
        security_sensors_node()
    except rospy.ROSInterruptException:
        pass