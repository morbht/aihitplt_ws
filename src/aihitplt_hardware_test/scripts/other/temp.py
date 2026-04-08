#!usr/bin/env python3
import rospy
import serial
from std_msgs.msg import Float32

rospy.init_node('temp_read')
pub = rospy.Publisher("/Temperature_measurement_topic",Float32,queue_size=10)
ser = serial.Serial('/dev/ttyUSB2',115200,timeout=1)

while not rospy.is_shutdown():
    try:
        line = ser.readline().decode().strip()
        if line:
            pub.publish(float(line))
    except:
        pass

    
    
    
    
    
    