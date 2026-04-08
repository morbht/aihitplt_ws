#!/usr/bin/env python3
import rospy
import serial
import struct
import json
import yaml
import os
from std_msgs.msg import String

FRAME_FORMAT = '<2sI 4H B 5H 2f B'
FRAME_SIZE = 34

def security_sensors_node():
    rospy.init_node('security_sensors_node')
    
    # 从YAML文件读取端口号
    config_path = os.path.join(os.path.dirname(__file__), '../config/security_sensors_port.yaml')
    try:
        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)
            port = config.get('security_sensor_port', '/dev/ttyUSB1')
    except Exception as e:
        rospy.logerr(f"Failed to read YAML config: {e}")
        port = '/dev/ttyUSB1'
    
    # 使用String类型发布JSON
    pub = rospy.Publisher('/security_sensors', String, queue_size=10)
    
    try:
        ser = serial.Serial(port, 115200, timeout=0.01)
        rospy.loginfo(f"Connected to {port}")
        rospy.sleep(2)
        
        buffer = bytearray()
        
        while not rospy.is_shutdown():
            new_data = ser.read(ser.in_waiting or 1)
            buffer.extend(new_data)
            
            while len(buffer) >= FRAME_SIZE:
                if buffer[0:2] != b'\xAA\x55':
                    buffer.pop(0)
                    continue
                
                frame = bytes(buffer[:FRAME_SIZE])
                
                try:
                    data = struct.unpack(FRAME_FORMAT, frame)
                    checksum = sum(frame[:-1]) & 0xFF
                    
                    if checksum == data[14]:
                        # 构建字典
                        sensor_dict = {
                            'alcohol': int(data[2]),
                            'smoke': int(data[3]),
                            'light': int(data[4]),
                            'sound': int(data[5]),
                            'emergency_stop': int(data[6]),
                            'eCO2': int(data[7]),
                            'eCH2O': float(data[8]/100.0),
                            'TVOC': float(data[9]/100.0),
                            'PM25': int(data[10]),
                            'PM10': int(data[11]),
                            'temperature': float(data[12]),
                            'humidity': float(data[13])
                        }
                        
                        # 发布JSON字符串
                        msg = String()
                        msg.data = json.dumps(sensor_dict)
                        pub.publish(msg)
                        
                    buffer = buffer[FRAME_SIZE:]
                    
                except:
                    buffer.pop(0)
                
    except Exception as e:
        rospy.logerr(f"Error: {e}")
    finally:
        if 'ser' in locals():
            ser.close()

if __name__ == '__main__':
    security_sensors_node()