#!/usr/bin/env python3
import rospy
import json
from std_msgs.msg import String

class SensorDisplay:
    def __init__(self):
        rospy.init_node('sensor_display')
        rospy.Subscriber('/security_sensors', String, self.callback)
        
        print("\n" + "="*50)
        print("安防传感器监控系统")
        print("="*50)
        
    def callback(self, msg):
        try:
            data = json.loads(msg.data)
            self.display(data)
        except:
            pass
    
    def display(self, data):
        # 清屏并显示最新数据
        print("\033[2J\033[H")  # 清屏命令
        
        print("="*50)
        print("实时传感器数据")
        print("="*50)
        
        print(f"酒精传感器: {data['alcohol']:>4d}")
        print(f"烟雾传感器: {data['smoke']:>4d}") 
        print(f"光照强度:   {data['light']:>4d}")
        print(f"声音强度:   {data['sound']:>4d}")
        print(f"急停状态:   {'🔴 触发' if data['emergency_stop'] == 0 else '🟢 正常'}")
        print("-"*30)
        print(f"CO2浓度:    {data['eCO2']:>4d} ppm")
        print(f"甲醛浓度:    {data['eCH2O']:.2f} mg/m³")
        print(f"TVOC浓度:   {data['TVOC']:.2f} mg/m³")
        print(f"PM2.5:      {data['PM25']:>4d} μg/m³")
        print(f"PM10:       {data['PM10']:>4d} μg/m³")
        print(f"温度:       {data['temperature']:.1f} °C")
        print(f"湿度:       {data['humidity']:.1f} %")
        
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    SensorDisplay().run()