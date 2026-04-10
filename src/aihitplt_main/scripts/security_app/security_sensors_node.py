#!/usr/bin/env python3
import rospy
import serial
import struct
import json
from std_msgs.msg import String, Bool

FRAME_FORMAT = '<2sI 4H B 5H 2f B'
FRAME_SIZE = 34

def security_sensors_node():
    rospy.init_node('security_sensors_node')
    
    # 直接指定端口号
    port = rospy.get_param("~/port","/dev/ttyUSB2") 
    
    # 发布器
    pub = rospy.Publisher('/security_sensors', String, queue_size=10)
    e_stop_pub = rospy.Publisher('/e_stop', Bool, queue_size=10)
    
    # 记录上一次急停状态
    last_e_stop_state = None
    
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
                        # 急停状态：True=触发, False=正常
                        emergency_stop = data[6]
                        e_stop_active = (emergency_stop == 0)
                        
                        # 构建字典
                        sensor_dict = {
                            'alcohol': int(data[2]),
                            'smoke': int(data[3]),
                            'light': int(data[4]),
                            'sound': int(data[5]),
                            'emergency_stop': int(emergency_stop),
                            'eCO2': int(data[7]),
                            'eCH2O': float(data[8]/100.0),
                            'TVOC': float(data[9]/100.0),
                            'PM25': int(data[10]),
                            'PM10': int(data[11]),
                            'temperature': float(data[12]),
                            'humidity': float(data[13])
                        }
                        
                        msg = String()
                        msg.data = json.dumps(sensor_dict)
                        pub.publish(msg)
                        
                        if last_e_stop_state != e_stop_active:
                            last_e_stop_state = e_stop_active
                            e_stop_msg = Bool()
                            e_stop_msg.data = e_stop_active
                            e_stop_pub.publish(e_stop_msg)
                            
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