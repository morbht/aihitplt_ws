#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import serial
import threading
import re
import time
from std_msgs.msg import String, Float32, Int32, Bool


class ScaleData:
    """存储传感器状态"""
    def __init__(self):
        self.emergency_stop = False      # 急停状态
        self.calibration_factor = 110.0  # 校准因子
        self.weight = 0.0                # 当前重量
        self.device_state = 0             # 设备状态（0-正常，1-归零中，2-校准中，3-初始化中，4-通信异常）


class LogiScaleNode:
    def __init__(self):
        rospy.init_node('logi_scale_node', anonymous=True)

        # 参数配置
        self.publish_raw_data = rospy.get_param('~publish_raw_data', False)
        self.port = rospy.get_param('~serial_port', '/dev/ttyUSB1')
        self.baudrate = rospy.get_param('~baudrate', 115200)
        self.timeout = rospy.get_param('~timeout', 0.1)
        self.cmd_topic = rospy.get_param('~cmd_topic', '/logi_scale/control')

        rospy.loginfo(f"使用串口: {self.port} @ {self.baudrate}")

        # 串口变量
        self.ser = None
        self.is_connected = False
        self.serial_lock = threading.Lock()
        self.scale_data = ScaleData()

        # 发布者
        self.weight_pub = rospy.Publisher('/logi_scale/weight', Float32, queue_size=10)
        self.cal_factor_pub = rospy.Publisher('/logi_scale/calibration_factor', Float32, queue_size=10)
        self.emergency_stop_pub = rospy.Publisher('/logi_scale/emergency_stop', Bool, queue_size=10)
        self.device_state_pub = rospy.Publisher('/logi_scale/device_state', Int32, queue_size=10)

        if self.publish_raw_data:
            self.raw_data_pub = rospy.Publisher('/logi_scale/raw_data', String, queue_size=10)
        else:
            self.raw_data_pub = None

        # 订阅者
        self.cmd_sub = rospy.Subscriber(self.cmd_topic, String, self.cmd_callback)

        # 启动串口线程
        self.stop_thread = False
        self.serial_thread = threading.Thread(target=self.serial_listener, daemon=True)
        self.connect_serial()
        self.serial_thread.start()

        # 定时发布状态
        self.pub_timer = rospy.Timer(rospy.Duration(0.1), self.publish_state)

        rospy.loginfo(f"LogiScale节点已启动，监听话题: {self.cmd_topic}")

    def connect_serial(self):
        """连接串口"""
        with self.serial_lock:
            try:
                self.ser = serial.Serial(
                    port=self.port,
                    baudrate=self.baudrate,
                    timeout=self.timeout,
                    parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE,
                    bytesize=serial.EIGHTBITS
                )
                self.is_connected = True
                rospy.loginfo(f"串口已连接: {self.port} ({self.baudrate} baud)")
            except Exception as e:
                self.is_connected = False
                rospy.logerr(f"串口连接失败: {e}")

    def serial_listener(self):
        """串口监听线程"""
        while not rospy.is_shutdown() and not self.stop_thread:
            if self.is_connected and self.ser and self.ser.is_open:
                try:
                    with self.serial_lock:
                        if self.ser.in_waiting > 0:
                            line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                            if line:
                                if self.publish_raw_data and self.raw_data_pub:
                                    self.raw_data_pub.publish(line)
                                self.parse_protocol_data(line)
                except Exception as e:
                    rospy.logwarn(f"串口读取错误: {e}")
                    self.is_connected = False
                    rospy.loginfo("尝试重连串口...")
                    self.connect_serial()
            time.sleep(0.05)

    def parse_protocol_data(self, data_str):
        """解析协议数据: DATA:ES=0,CAL=110.0,WEIGHT=0.0,STATE=0"""
        pattern = r"DATA:ES=(\d+),CAL=([\d.]+),WEIGHT=([\d.]+),STATE=(\d+)"
        match = re.match(pattern, data_str)
        if match:
            self.scale_data.emergency_stop = bool(int(match.group(1)))
            self.scale_data.calibration_factor = float(match.group(2))
            self.scale_data.weight = float(match.group(3))
            self.scale_data.device_state = int(match.group(4))
        elif data_str.startswith("ES:"):
            if data_str == "ES:1":
                self.scale_data.emergency_stop = True
                rospy.logwarn("⚠️ 急停已触发 ⚠️")
            elif data_str == "ES:0":
                self.scale_data.emergency_stop = False
                rospy.loginfo("急停已释放")
        elif data_str.startswith("CMD:") or data_str.startswith("CALIB:"):
            rospy.loginfo(f"传感器响应: {data_str}")

    def cmd_callback(self, msg):
        """处理控制指令"""
        cmd = msg.data.strip()
        if not self.is_connected:
            rospy.logwarn(f"串口未连接，忽略指令: {cmd}")
            return

        try:
            with self.serial_lock:
                self.ser.write(f"{cmd}\n".encode('utf-8'))
                rospy.loginfo(f"发送指令到传感器: {cmd}")
        except Exception as e:
            rospy.logerr(f"指令发送失败: {cmd}, 错误: {e}")
            self.is_connected = False

    def publish_state(self, event):
        """定时发布状态"""
        self.weight_pub.publish(self.scale_data.weight)
        self.cal_factor_pub.publish(self.scale_data.calibration_factor)
        self.emergency_stop_pub.publish(self.scale_data.emergency_stop)
        self.device_state_pub.publish(self.scale_data.device_state)

    def shutdown(self):
        """清理资源"""
        self.stop_thread = True
        with self.serial_lock:
            if self.ser and self.ser.is_open:
                self.ser.close()
        rospy.loginfo("LogiScale节点已关闭")


if __name__ == '__main__':
    node = None
    try:
        node = LogiScaleNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logfatal(f"节点错误: {e}")
    finally:
        if node:
            node.shutdown()