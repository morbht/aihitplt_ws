#!/usr/bin/env python3
import rospy
import serial
import time
import threading
from std_msgs.msg import String

class ESP32PanTiltController:
    def __init__(self, port='/dev/ttyUSB2', baudrate=115200):
        rospy.init_node('pan_tilt_interactive', anonymous=True)
        
        self.current_h, self.current_v = 90, 90
        self.sensor_pub = rospy.Publisher('pan_tilt_sensor', String, queue_size=10)
        self.sensor_thread_running = True

        # 初始化串口
        try:
            self.ser = serial.Serial(port, baudrate, timeout=1)
            time.sleep(2)  
            rospy.loginfo(f"成功连接到串口: {port}")
        except serial.SerialException as e:
            rospy.logerr(f"串口连接失败: {e}")
            exit(1)

        # 启动接收线程
        self.sensor_thread = threading.Thread(target=self.read_sensor_loop)
        self.sensor_thread.daemon = True
        self.sensor_thread.start()

    def send_cmd(self, h, v):
        """发送控制指令并更新当前角度"""
        if 0 <= h <= 180 and 0 <= v <= 180:
            self.ser.write(f"P:{h},{v}\n".encode())
            self.current_h, self.current_v = h, v
        else:
            print("错误：角度范围必须在 0-180 之间")

    def read_sensor_loop(self):
        """后台读取传感器数据"""
        while self.sensor_thread_running and not rospy.is_shutdown():
            try:
                if self.ser.in_waiting > 0:
                    if self.ser.read(2) == b'\xAA\x55':  # 匹配帧头
                        data = self.ser.read(53)
                        if len(data) == 53:
                            self.sensor_pub.publish(f"Sensor data received: {len(data)} bytes")
            except Exception:
                pass
            time.sleep(0.05)

    def run_interactive(self):
        print("\n=== 云台控制终端 ===")
        print("操作: w(上) s(下) a(左) d(右) c(居中) | quit(退出)")
        print("绝对角度: 输入 '水平,垂直' (例如: 90,45)")
        
        while not rospy.is_shutdown():
            try:
                cmd = input("\n指令: ").strip().lower()
                if cmd == 'quit': break
                elif cmd == 'c': self.send_cmd(90, 90)
                elif cmd == 'a': self.send_cmd(max(0, self.current_h - 15), self.current_v)
                elif cmd == 'd': self.send_cmd(min(180, self.current_h + 15), self.current_v)
                elif cmd == 'w': self.send_cmd(self.current_h, min(180, self.current_v + 15))
                elif cmd == 's': self.send_cmd(self.current_h, max(0, self.current_v - 15))
                elif ',' in cmd:
                    h, v = map(int, cmd.split(','))
                    self.send_cmd(h, v)
                else:
                    print("未知指令。")
            except ValueError:
                print("输入格式错误。")
            except KeyboardInterrupt:
                break

    def cleanup(self):
        """安全释放资源"""
        self.sensor_thread_running = False
        if hasattr(self, 'ser') and self.ser.is_open:
            self.send_cmd(90, 90)  # 退出前云台居中
            time.sleep(0.1)
            self.ser.close()
            print("\n串口已关闭。")

def main():
    # 优先读取 ROS 参数，否则使用默认值
    port = rospy.get_param('~port', '/dev/ttyUSB2')
    baud = rospy.get_param('~baud', 115200)
    
    controller = ESP32PanTiltController(port, baud)
    try:
        controller.run_interactive()
    finally:
        controller.cleanup()

if __name__ == '__main__':
    main()