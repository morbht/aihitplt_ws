#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
送物模块ROS节点
功能：串口通信，话题发布与订阅，控制上下舱门
"""

import rospy
import serial
import threading
import time
import os
from std_msgs.msg import Int32, Bool, String


class DeliveryModuleNode:
    def __init__(self):
        """初始化ROS节点"""
        rospy.init_node('delivery_module_node', anonymous=True)

        # 串口相关变量
        self.ser = None
        self.serial_connected = False
        self.serial_thread = None
        self.stop_serial_thread = False

        # 状态变量
        self.sys_state = 0
        self.upper_motor_enabled = False
        self.lower_motor_enabled = False
        self.c1u_complete = False
        self.c1d_complete = False
        self.c2u_complete = False
        self.c2d_complete = False
        self.emergency_stop = False
        self.c1u_limit = False
        self.c1d_limit = False
        self.c2u_limit = False
        self.c2d_limit = False

        # 配置参数
        self.serial_port = rospy.get_param('~serial_port', '/dev/ttyUSB1')
        self.baudrate = rospy.get_param('~baudrate', 115200)

        # 初始化发布器和订阅器
        self.init_publishers()
        self.init_subscribers()

        # 连接串口
        self.connect_serial()

        rospy.loginfo("送物模块ROS节点初始化完成")
        self.publish_states()

    def init_publishers(self):
        """初始化所有话题发布器"""
        self.state_pub = rospy.Publisher('delivery_device_state', Int32, queue_size=10)
        self.upper_motor_pub = rospy.Publisher('upper_motor_state', Bool, queue_size=10)
        self.upper_up_limit_pub = rospy.Publisher('upper_up_limit_state', Bool, queue_size=10)
        self.upper_down_limit_pub = rospy.Publisher('upper_down_limit_state', Bool, queue_size=10)
        self.lower_motor_pub = rospy.Publisher('lower_motor_state', Bool, queue_size=10)
        self.lower_up_limit_pub = rospy.Publisher('lower_up_limit_state', Bool, queue_size=10)
        self.lower_down_limit_pub = rospy.Publisher('lower_down_limit_state', Bool, queue_size=10)
        self.sys_state_pub = rospy.Publisher('delivery_system_state', Int32, queue_size=10)
        self.emergency_pub = rospy.Publisher('e_stop', Bool, queue_size=10)
        rospy.loginfo("话题发布器初始化完成")

    def init_subscribers(self):
        """初始化所有话题订阅器"""
        rospy.Subscriber('upper_motor_state_cmd', Bool, self.upper_motor_callback)
        rospy.Subscriber('upper_reset_cmd', String, self.upper_reset_callback)
        rospy.Subscriber('upper_control_cmd', String, self.upper_control_callback)
        rospy.Subscriber('lower_motor_state_cmd', Bool, self.lower_motor_callback)
        rospy.Subscriber('lower_reset_cmd', String, self.lower_reset_callback)
        rospy.Subscriber('lower_control_cmd', String, self.lower_control_callback)
        rospy.Subscriber('delivery_init_cmd', String, self.system_init_callback)
        rospy.Subscriber('motor_reset_cmd', Bool, self.motor_reset_callback)
        rospy.loginfo("话题订阅器初始化完成")

    def connect_serial(self):
        """连接串口"""
        if not os.path.exists(self.serial_port):
            rospy.logerr(f"串口不存在: {self.serial_port}")
            return

        try:
            self.ser = serial.Serial(
                port=self.serial_port,
                baudrate=self.baudrate,
                timeout=0.1
            )
            self.serial_connected = True
            self.stop_serial_thread = False

            self.serial_thread = threading.Thread(target=self.read_serial_data, daemon=True)
            self.serial_thread.start()

            rospy.loginfo(f"已连接到串口: {self.serial_port} ({self.baudrate} baud)")
            self.ser.write(b"HELLO\n")
        except Exception as e:
            rospy.logerr(f"串口连接失败: {e}")

    def read_serial_data(self):
        """读取串口数据线程"""
        while not rospy.is_shutdown() and self.serial_connected and not self.stop_serial_thread:
            try:
                if self.ser and self.ser.in_waiting > 0:
                    data = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    if data:
                        self.parse_serial_data(data)
            except Exception as e:
                rospy.logwarn(f"串口读取异常: {e}")
                time.sleep(0.1)
            time.sleep(0.01)
        rospy.loginfo("串口读取线程结束")

    def parse_serial_data(self, data):
        """解析串口数据"""
        if data.startswith("S:"):
            try:
                parts = data[2:].split(',')
                if len(parts) >= 12:
                    self.sys_state = int(parts[0])
                    self.upper_motor_enabled = bool(int(parts[1]))
                    self.lower_motor_enabled = bool(int(parts[2]))
                    self.c1u_complete = bool(int(parts[3]))
                    self.c1d_complete = bool(int(parts[4]))
                    self.c2u_complete = bool(int(parts[5]))
                    self.c2d_complete = bool(int(parts[6]))
                    self.emergency_stop = bool(int(parts[7]))
                    self.c1u_limit = bool(int(parts[8]))
                    self.c1d_limit = bool(int(parts[9]))
                    self.c2u_limit = bool(int(parts[10]))
                    self.c2d_limit = bool(int(parts[11]))

                    self.publish_states()
                    self.publish_device_state()
            except Exception as e:
                rospy.logwarn(f"解析状态数据失败: {e}")
        elif data.startswith("OK:"):
            rospy.loginfo(f"命令成功: {data}")
        elif data.startswith("ERR:"):
            rospy.logwarn(f"命令失败: {data}")

    def publish_states(self):
        """发布状态到ROS话题"""
        self.upper_motor_pub.publish(self.upper_motor_enabled)
        self.upper_up_limit_pub.publish(self.c1u_limit)
        self.upper_down_limit_pub.publish(self.c1d_limit)
        self.lower_motor_pub.publish(self.lower_motor_enabled)
        self.lower_up_limit_pub.publish(self.c2u_limit)
        self.lower_down_limit_pub.publish(self.c2d_limit)
        self.sys_state_pub.publish(self.sys_state)
        self.emergency_pub.publish(self.emergency_stop)

    def publish_device_state(self):
        """发布设备状态码"""
        state_code = min(self.sys_state, 99)  # 限制在0-99
        if rospy.get_time() % 5 < 0.1:
            state_text = {
                0: "正常", 1: "初始化中", 2: "急停中", 3: "电机重置中",
                4: "上门上复位", 5: "上门下复位", 6: "下门上复位", 7: "下门下复位",
                8: "上门向上移动", 9: "上门向下移动", 10: "下门向上移动", 11: "下门向下移动"
            }.get(self.sys_state, f"未知({self.sys_state})")
            if self.emergency_stop:
                state_text = "⚠ 急停已触发 ⚠"
            rospy.loginfo(f"状态码: {state_code} -> {state_text}")
        self.state_pub.publish(Int32(state_code))

    def send_serial_command(self, cmd, desc=""):
        """发送串口命令"""
        if not self.serial_connected or not self.ser:
            rospy.logwarn(f"串口未连接，无法发送: {desc}")
            return False
        try:
            self.ser.write(f"{cmd}\n".encode())
            rospy.loginfo(f"发送命令: {cmd} ({desc})")
            return True
        except Exception as e:
            rospy.logerr(f"命令发送失败: {e} ({desc})")
            return False

    # 回调函数
    def upper_motor_callback(self, msg):
        if msg.data:
            self.send_serial_command("E1", "上舱门使能")
        else:
            self.send_serial_command("D1", "上舱门禁用")

    def lower_motor_callback(self, msg):
        if msg.data:
            self.send_serial_command("E2", "下舱门使能")
        else:
            self.send_serial_command("D2", "下舱门禁用")

    def upper_reset_callback(self, msg):
        try:
            parts = msg.data.split(',')
            direction = parts[0].strip().lower()
            if len(parts) > 1:
                dist = float(parts[1].strip())
                if dist <= 0 or dist > 1000:
                    dist = 220.0
                cmd = f"C1U,{dist}" if direction == "up" else f"C1D,{dist}"
            else:
                cmd = "C1U" if direction == "up" else "C1D"
            self.send_serial_command(cmd, f"上舱门{'上' if direction=='up' else '下'}复位")
        except Exception as e:
            rospy.logerr(f"上舱门复位解析失败: {e}")

    def lower_reset_callback(self, msg):
        try:
            parts = msg.data.split(',')
            direction = parts[0].strip().lower()
            if len(parts) > 1:
                dist = float(parts[1].strip())
                if dist <= 0 or dist > 1000:
                    dist = 220.0
                cmd = f"C2U,{dist}" if direction == "up" else f"C2D,{dist}"
            else:
                cmd = "C2U" if direction == "up" else "C2D"
            self.send_serial_command(cmd, f"下舱门{'上' if direction=='up' else '下'}复位")
        except Exception as e:
            rospy.logerr(f"下舱门复位解析失败: {e}")

    def upper_control_callback(self, msg):
        try:
            parts = msg.data.split(',')
            direction = parts[0].strip().lower()
            if len(parts) > 1:
                dist = float(parts[1].strip())
                if dist <= 0:
                    dist = 10.0
                cmd = f"M1U,{dist}" if direction == "up" else f"M1D,{dist}"
            else:
                cmd = "M1U" if direction == "up" else "M1D"
            self.send_serial_command(cmd, f"上舱门向{'上' if direction=='up' else '下'}移动")
        except Exception as e:
            rospy.logerr(f"上舱门移动解析失败: {e}")

    def lower_control_callback(self, msg):
        try:
            parts = msg.data.split(',')
            direction = parts[0].strip().lower()
            if len(parts) > 1:
                dist = float(parts[1].strip())
                if dist <= 0:
                    dist = 10.0
                cmd = f"M2U,{dist}" if direction == "up" else f"M2D,{dist}"
            else:
                cmd = "M2U" if direction == "up" else "M2D"
            self.send_serial_command(cmd, f"下舱门向{'上' if direction=='up' else '下'}移动")
        except Exception as e:
            rospy.logerr(f"下舱门移动解析失败: {e}")

    def system_init_callback(self, msg):
        try:
            if msg.data.lower() == "init":
                self.send_serial_command("INIT", "系统初始化")
            else:
                parts = msg.data.split(',')
                if len(parts) >= 2:
                    upper = float(parts[0].strip())
                    lower = float(parts[1].strip())
                    upper = 220.0 if upper <= 0 or upper > 1000 else upper
                    lower = 220.0 if lower <= 0 or lower > 1000 else lower
                    self.send_serial_command(f"INIT,{upper},{lower}", f"系统初始化({upper}mm,{lower}mm)")
        except Exception as e:
            rospy.logerr(f"初始化命令解析失败: {e}")

    def motor_reset_callback(self, msg):
        if msg.data:
            self.send_serial_command("R", "电机重置")

    def cleanup(self):
        """清理资源"""
        rospy.loginfo("清理资源...")
        self.stop_serial_thread = True
        self.serial_connected = False
        if self.ser:
            self.ser.close()
        if self.serial_thread and self.serial_thread.is_alive():
            self.serial_thread.join(timeout=1.0)

    def run(self):
        """主循环"""
        rospy.loginfo("送物模块ROS节点启动")
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            try:
                self.publish_device_state()
                if not self.serial_connected:
                    rospy.logwarn("串口未连接，尝试重连...")
                    self.connect_serial()
                rate.sleep()
            except rospy.ROSInterruptException:
                break
            except Exception as e:
                rospy.logerr(f"主循环异常: {e}")
                rate.sleep()


def main():
    node = None
    try:
        node = DeliveryModuleNode()
        node.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("节点中断")
    except Exception as e:
        rospy.logerr(f"节点异常: {e}")
    finally:
        if node:
            node.cleanup()
        rospy.loginfo("节点已停止")


if __name__ == '__main__':
    main()