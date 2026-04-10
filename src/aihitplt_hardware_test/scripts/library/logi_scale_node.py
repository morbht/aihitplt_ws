#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import serial
import serial.tools.list_ports
import threading
import re
import time
import os
import yaml
from std_msgs.msg import String, Float32, Int32, Bool

# 自定义数据结构体（存储传感器状态）
class ScaleData:
    def __init__(self):
        self.emergency_stop = False  # 急停状态
        self.calibration_factor = 110.0  # 校准因子
        self.weight = 0.0  # 当前重量
        self.device_state = 0  # 设备状态（0-正常，1-归零中，2-校准中，3-初始化中，4-通信异常）

class LogiScaleNode:
    def __init__(self):
        # ROS节点初始化
        rospy.init_node('logi_scale_node', anonymous=True)
        
        self.publish_raw_data = rospy.get_param('~publish_raw_data', False)  # 默认不发布
        
        # 参数配置：先从YAML配置文件加载，如果没有则使用ROS参数或默认值
        self.port, self.baudrate = self._load_config_from_yaml()
        
        # 如果配置文件没有找到或参数不完整，使用ROS参数或默认值
        if not self.port:
            self.port = rospy.get_param('~serial_port', '/dev/ttyUSB2')
        if not self.baudrate:
            self.baudrate = rospy.get_param('~baudrate', 115200)
            
        self.timeout = rospy.get_param('~timeout', 0.1)
        self.cmd_topic = rospy.get_param('~cmd_topic', '/logi_scale/control')
        
        rospy.loginfo(f"使用串口: {self.port} @ {self.baudrate}")
        
        # 串口初始化
        self.ser = None
        self.is_connected = False
        self.serial_lock = threading.Lock()  # 串口操作互斥锁
        self.scale_data = ScaleData()
        
        # ROS发布者（发布传感器状态）
        self.weight_pub = rospy.Publisher('/logi_scale/weight', Float32, queue_size=10)
        self.cal_factor_pub = rospy.Publisher('/logi_scale/calibration_factor', Float32, queue_size=10)
        self.emergency_stop_pub = rospy.Publisher('/e_stop', Bool, queue_size=10)
        self.device_state_pub = rospy.Publisher('/logi_scale/device_state', Int32, queue_size=10)
        
        self.raw_data_pub = None
        if self.publish_raw_data:
            self.raw_data_pub = rospy.Publisher('/logi_scale/raw_data', String, queue_size=10)
        
        # ROS订阅者（接收控制指令）
        self.cmd_sub = rospy.Subscriber(self.cmd_topic, String, self.cmd_callback, queue_size=10)
        
        # 串口连接线程
        self.stop_thread = False
        self.serial_thread = threading.Thread(target=self.serial_listener)
        self.serial_thread.daemon = True
        self.connect_serial()
        self.serial_thread.start()
        
        # 状态发布定时器（10Hz）
        self.pub_timer = rospy.Timer(rospy.Duration(0.1), self.publish_state)
        
        rospy.loginfo("LogiScale node initialized. Listening on topic: %s", self.cmd_topic)

    def _load_config_from_yaml(self):
        """从YAML配置文件加载串口参数"""
        # 默认值
        port = None
        baudrate = None
        
        # 尝试多个可能的配置文件路径
        config_paths = [
            os.path.join(
                os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                'config', 'weight_sensor_config.yaml'
            ),
            '/home/aihit/aihitplt_ws/src/aihitplt_hardware_test/config/weight_sensor_config.yaml',
            os.path.expanduser("~/aihitplt_ws/src/aihitplt_hardware_test/config/weight_sensor_config.yaml")
        ]
        
        config_file = None
        for path in config_paths:
            if os.path.exists(path):
                config_file = path
                rospy.loginfo(f"找到配置文件: {path}")
                break
        
        if config_file:
            try:
                with open(config_file, 'r') as f:
                    cfg = yaml.safe_load(f) or {}
                
                # 从配置文件获取参数
                if 'serial_port' in cfg:
                    port = cfg['serial_port']
                    rospy.loginfo(f"从配置文件加载串口号: {port}")
                if 'baudrate' in cfg:
                    baudrate = cfg['baudrate']
                    rospy.loginfo(f"从配置文件加载波特率: {baudrate}")
                    
            except Exception as e:
                rospy.logerr(f"读取配置文件失败: {e}")
                rospy.loginfo(f"将使用ROS参数或默认值")
        else:
            rospy.logwarn(f"未找到配置文件，将使用ROS参数或默认值")
        
        return port, baudrate

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
                rospy.loginfo("Serial port connected: %s (baudrate: %d)", self.port, self.baudrate)
            except Exception as e:
                self.is_connected = False
                rospy.logerr("Failed to connect serial port: %s", str(e))

    def serial_listener(self):
        """串口监听线程：持续读取传感器数据"""
        while not rospy.is_shutdown() and not self.stop_thread:
            if self.is_connected and self.ser.is_open:
                try:
                    with self.serial_lock:
                        if self.ser.in_waiting > 0:
                            line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                            if line:
                                if self.publish_raw_data and self.raw_data_pub is not None:                            
                                    # 发布原始数据
                                    self.raw_data_pub.publish(line)
                                # 解析协议数据
                                self.parse_protocol_data(line)
                except Exception as e:
                    rospy.logwarn("Serial read error: %s", str(e))
                    self.is_connected = False
                    # 尝试重连
                    rospy.loginfo("Trying to reconnect serial port...")
                    self.connect_serial()
            time.sleep(0.05)  # 降低CPU占用

    def parse_protocol_data(self, data_str):
        """解析ESP8266发送的协议数据：DATA:ES=0,CAL=110.0,WEIGHT=0.0,STATE=0"""
        pattern = r"DATA:ES=(\d+),CAL=([\d.]+),WEIGHT=([\d.]+),STATE=(\d+)"
        match = re.match(pattern, data_str)
        if match:
            self.scale_data.emergency_stop = bool(int(match.group(1)))
            self.scale_data.calibration_factor = float(match.group(2))
            self.scale_data.weight = float(match.group(3))
            self.scale_data.device_state = int(match.group(4))
            
            # 调试日志（可选）
            # rospy.logdebug(f"解析到数据: ES={self.scale_data.emergency_stop}, CAL={self.scale_data.calibration_factor}, "
            #               f"WEIGHT={self.scale_data.weight}, STATE={self.scale_data.device_state}")
            
        # 解析急停单独消息
        elif data_str.startswith("ES:"):
            if data_str == "ES:1":
                self.scale_data.emergency_stop = True
                rospy.logwarn("Emergency stop triggered!")
            elif data_str == "ES:0":
                self.scale_data.emergency_stop = False
                rospy.loginfo("Emergency stop released.")
        # 解析命令响应消息
        elif data_str.startswith("CMD:") or data_str.startswith("CALIB:"):
            rospy.loginfo(f"传感器响应: {data_str}")

    def cmd_callback(self, msg):
        """处理ROS控制指令"""
        cmd = msg.data.strip()
        if not self.is_connected:
            rospy.logwarn("Serial port not connected. Ignore command: %s", cmd)
            return
        
        # 指令解析
        try:
            with self.serial_lock:
                full_cmd = cmd + "\n"
                self.ser.write(full_cmd.encode('utf-8'))
                rospy.loginfo("Send command to scale: %s", cmd)
        except Exception as e:
            rospy.logerr("Failed to send command: %s, error: %s", cmd, str(e))
            self.is_connected = False

    def publish_state(self, event):
        """发布传感器状态（定时器回调）"""
        self.weight_pub.publish(self.scale_data.weight)
        self.cal_factor_pub.publish(self.scale_data.calibration_factor)
        self.emergency_stop_pub.publish(self.scale_data.emergency_stop)
        self.device_state_pub.publish(self.scale_data.device_state)

    def shutdown(self):
        """节点关闭时的清理工作"""
        self.stop_thread = True
        with self.serial_lock:
            if self.ser and self.ser.is_open:
                self.ser.close()
        rospy.loginfo("LogiScale node shutdown.")

if __name__ == '__main__':
    try:
        node = LogiScaleNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        node.shutdown()
    except Exception as e:
        rospy.logfatal("Node error: %s", str(e))