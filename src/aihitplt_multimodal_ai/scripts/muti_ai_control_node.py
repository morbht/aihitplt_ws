#!/usr/bin/env python3
import rospy
import serial
import time
import threading
from std_msgs.msg import String

class ESP32PanTiltController:
    def __init__(self, port='/dev/ttyUSB2', baudrate=115200):
        rospy.init_node('pan_tilt_node', anonymous=True)
        
        self.current_h, self.current_v = 90, 90
        self.sensor_thread_running = True

        # 1. 初始化串口
        try:
            self.ser = serial.Serial(port, baudrate, timeout=1)
            time.sleep(2)  # 等待 ESP32 复位
            rospy.loginfo(f"成功连接到串口: {port}")
        except serial.SerialException as e:
            rospy.logerr(f"串口连接失败: {e}")
            exit(1)

        # 2. 启动传感器接收线程
        self.sensor_thread = threading.Thread(target=self.read_sensor_loop)
        self.sensor_thread.daemon = True
        self.sensor_thread.start()

        # 3. 订阅控制话题
        rospy.Subscriber('/muti_ai/pan_control', String, self.cmd_callback)

    def send_cmd(self, h, v):
        """发送控制指令并更新当前角度"""
        if 0 <= h <= 180 and 0 <= v <= 180:
            self.ser.write(f"P:{h},{v}\n".encode())
            self.current_h, self.current_v = h, v
        else:
            rospy.logwarn(f"角度越界: h={h}, v={v}")

    def cmd_callback(self, msg):
        """处理来自话题的控制指令"""
        cmd = msg.data.strip().lower()
        try:
            if cmd == 'c': self.send_cmd(90, 90)
            elif cmd == 'a': self.send_cmd(max(0, self.current_h - 15), self.current_v)
            elif cmd == 'd': self.send_cmd(min(180, self.current_h + 15), self.current_v)
            elif cmd == 'w': self.send_cmd(self.current_h, min(180, self.current_v + 15))
            elif cmd == 's': self.send_cmd(self.current_h, max(0, self.current_v - 15))
            elif ',' in cmd:
                h, v = map(int, cmd.split(','))
                self.send_cmd(h, v)
            else:
                rospy.logwarn(f"未知指令: {cmd}")
        except ValueError:
            rospy.logwarn(f"指令格式解析错误: {cmd}")

    def read_sensor_loop(self):
        """后台读取传感器数据"""
        while self.sensor_thread_running and not rospy.is_shutdown():
            try:
                if self.ser.in_waiting > 0:
                    if self.ser.read(2) == b'\xAA\x55':  # 匹配帧头
                        data = self.ser.read(53)
            except Exception:
                pass
            time.sleep(0.05)

    def cleanup(self):
        """安全释放资源"""
        self.sensor_thread_running = False
        if hasattr(self, 'ser') and self.ser.is_open:
            self.send_cmd(90, 90)  # 退出前云台居中
            time.sleep(0.1)
            self.ser.close()
            rospy.loginfo("串口已关闭。")

def main():
    port = rospy.get_param('~port', '/dev/ttyUSB2')
    baud = rospy.get_param('~baud', 115200)
    
    controller = ESP32PanTiltController(port, baud)
    try:
        # 替代原本的死循环，由 ROS 接管线程阻塞
        rospy.spin() 
    except KeyboardInterrupt:
        pass
    finally:
        controller.cleanup()

if __name__ == '__main__':
    main()
