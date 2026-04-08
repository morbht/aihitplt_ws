#!/usr/bin/env python3
"""
Security Sensors ROS Node

This node reads data from security sensors via serial port and publishes
the sensor data as JSON string to /security_sensors topic.
"""

import rospy
import serial
import struct
import json
from std_msgs.msg import String

# Serial frame format and size
FRAME_FORMAT = '<2sI 4H B 5H 2f B'
FRAME_SIZE = 34

# Serial port configuration
SERIAL_PORT = '/dev/ttyUSB2'
BAUDRATE = 115200
TIMEOUT = 0.01


def security_sensors_node():
    """
    Main function for security sensors ROS node.
    Reads sensor data from serial port and publishes as JSON.
    """
    # Initialize ROS node
    rospy.init_node('security_sensors_node', anonymous=True)
    
    # ROS publisher for sensor data
    pub = rospy.Publisher('/security_sensors', String, queue_size=10)
    
    try:
        # Open serial connection
        ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=TIMEOUT)
        rospy.loginfo(f"Connected to {SERIAL_PORT} at {BAUDRATE} baud")
        
        # Allow time for serial connection to stabilize
        rospy.sleep(2)
        
        # Buffer for accumulating serial data
        buffer = bytearray()
        
        while not rospy.is_shutdown():
            # Read available data from serial port
            new_data = ser.read(ser.in_waiting or 1)
            buffer.extend(new_data)
            
            # Process complete frames in buffer
            while len(buffer) >= FRAME_SIZE:
                # Check frame header
                if buffer[0:2] != b'\xAA\x55':
                    buffer.pop(0)
                    continue
                
                # Extract one frame
                frame = bytes(buffer[:FRAME_SIZE])
                
                try:
                    # Unpack frame data
                    data = struct.unpack(FRAME_FORMAT, frame)
                    
                    # Verify checksum
                    checksum = sum(frame[:-1]) & 0xFF
                    if checksum == data[14]:
                        # Build sensor data dictionary
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
                        
                        # Publish as JSON string
                        msg = String()
                        msg.data = json.dumps(sensor_dict)
                        pub.publish(msg)
                        
                    # Remove processed frame from buffer
                    buffer = buffer[FRAME_SIZE:]
                    
                except (struct.error, IndexError) as e:
                    # Invalid frame, remove first byte and continue
                    rospy.logdebug(f"Frame parsing error: {e}")
                    buffer.pop(0)
                
    except serial.SerialException as e:
        rospy.logerr(f"Serial port error: {e}")
    except Exception as e:
        rospy.logerr(f"Unexpected error: {e}")
    finally:
        # Close serial port if open
        if 'ser' in locals() and ser.is_open:
            ser.close()
            rospy.loginfo("Serial port closed")


if __name__ == '__main__':
    try:
        security_sensors_node()
    except rospy.ROSInterruptException:
        pass