#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
送物模块ROS节点
功能：串口通信，话题发布与订阅，控制上下舱门
"""

import rospy
import serial
import serial.tools.list_ports
import threading
import time
import os
import sys
from std_msgs.msg import Int32, Bool, Float32, String
from geometry_msgs.msg import Twist
import yaml

class DeliveryModuleNode:
    def __init__(self):
        """初始化ROS节点"""
        rospy.init_node('delivery_module_node', anonymous=True)
        
        # 串口相关变量
        self.ser = None
        self.serial_connected = False
        self.serial_thread = None
        self.stop_serial_thread = False
        
        # 状态变量（与下位机对应）
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
        self.config_dir = rospy.get_param('~config_dir', '/home/aihit/aihitplt_ws/src/aihitplt_hardware_test/config')
        self.config_file = os.path.join(self.config_dir, 'delivery_module_config.yaml')
        self.default_serial_port = rospy.get_param('~serial_port', '/dev/ttyUSB0')
        self.baudrate = rospy.get_param('~baudrate', 115200)
        
        # 从配置文件加载串口
        self.serial_port = self.load_serial_port()
        
        # 初始化发布器
        self.init_publishers()
        
        # 初始化订阅器
        self.init_subscribers()
        
        # 连接串口
        self.connect_serial()
        
        # 初始化完成
        rospy.loginfo("送物模块ROS节点初始化完成")
        
        # 发布初始状态
        self.publish_states()
        
    def load_serial_port(self):
        """从配置文件加载串口"""
        if os.path.exists(self.config_file):
            try:
                with open(self.config_file, 'r') as f:
                    config = yaml.safe_load(f)
                if config and 'serial_port' in config:
                    rospy.loginfo(f"从配置文件加载串口: {config['serial_port']}")
                    return config['serial_port']
            except Exception as e:
                rospy.logwarn(f"加载配置文件失败: {e}")
        
        rospy.loginfo(f"使用默认串口: {self.default_serial_port}")
        return self.default_serial_port
    
    def init_publishers(self):
        """初始化所有话题发布器"""
        # 设备状态话题（发布数字状态码）
        self.state_pub = rospy.Publisher('delivery_device_state', Int32, queue_size=10)
        
        # 上舱门状态话题
        self.upper_motor_pub = rospy.Publisher('upper_motor_state', Bool, queue_size=10)
        self.upper_up_limit_pub = rospy.Publisher('upper_up_limit_state', Bool, queue_size=10)
        self.upper_down_limit_pub = rospy.Publisher('upper_down_limit_state', Bool, queue_size=10)
        
        # 下舱门状态话题
        self.lower_motor_pub = rospy.Publisher('lower_motor_state', Bool, queue_size=10)
        self.lower_up_limit_pub = rospy.Publisher('lower_up_limit_state', Bool, queue_size=10)
        self.lower_down_limit_pub = rospy.Publisher('lower_down_limit_state', Bool, queue_size=10)
        
        # 系统状态话题
        self.sys_state_pub = rospy.Publisher('delivery_system_state', Int32, queue_size=10)
        self.emergency_pub = rospy.Publisher('emergency_stop', Bool, queue_size=10)
        
        rospy.loginfo("话题发布器初始化完成")
    
    def init_subscribers(self):
        """初始化所有话题订阅器"""
        # 上舱门控制话题
        rospy.Subscriber('upper_motor_state_cmd', Bool, self.upper_motor_callback)
        rospy.Subscriber('upper_reset_cmd', String, self.upper_reset_callback)
        rospy.Subscriber('upper_control_cmd', String, self.upper_control_callback)
        
        # 下舱门控制话题
        rospy.Subscriber('lower_motor_state_cmd', Bool, self.lower_motor_callback)
        rospy.Subscriber('lower_reset_cmd', String, self.lower_reset_callback)
        rospy.Subscriber('lower_control_cmd', String, self.lower_control_callback)
        
        # 系统控制话题
        rospy.Subscriber('delivery_init_cmd', String, self.system_init_callback)
        rospy.Subscriber('motor_reset_cmd', Bool, self.motor_reset_callback)
        
        rospy.loginfo("话题订阅器初始化完成")
    
    def connect_serial(self):
        """连接串口"""
        if not self.serial_port:
            rospy.logerr("未配置串口，无法连接")
            return
        
        if not os.path.exists(self.serial_port):
            rospy.logerr(f"串口不存在: {self.serial_port}")
            return
        
        try:
            self.ser = serial.Serial(
                port=self.serial_port,
                baudrate=self.baudrate,
                timeout=0.1,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE
            )
            
            self.serial_connected = True
            self.stop_serial_thread = False
            
            # 启动串口读取线程
            self.serial_thread = threading.Thread(target=self.read_serial_data, daemon=True)
            self.serial_thread.start()
            
            rospy.loginfo(f"成功连接到串口: {self.serial_port} (波特率: {self.baudrate})")
            
            # 发送连接成功消息
            self.ser.write(b"HELLO\n")
            
        except Exception as e:
            error_msg = str(e)
            if "Permission denied" in error_msg:
                rospy.logerr(f"权限不足，请运行: sudo chmod 666 {self.serial_port}")
            else:
                rospy.logerr(f"连接串口失败: {error_msg}")
    
    def read_serial_data(self):
        """读取串口数据"""
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
        # 解析状态数据格式: S:0,1,0,0,0,0,0,0,0,0,0,0
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
                    
                    # 发布状态到ROS话题
                    self.publish_states()
                    
                    # 发布设备状态数字码
                    self.publish_device_state()
                    
            except Exception as e:
                rospy.logwarn(f"解析状态数据失败: {e}, 数据: {data}")
        
        # 解析其他类型的数据
        elif data.startswith("OK:"):
            rospy.loginfo(f"命令执行成功: {data}")
        elif data.startswith("ERR:"):
            rospy.logwarn(f"命令执行失败: {data}")
        else:
            # 忽略其他数据
            pass
    
    def publish_states(self):
        """发布所有状态到ROS话题"""
        try:
            # 发布上舱门状态
            self.upper_motor_pub.publish(self.upper_motor_enabled)
            self.upper_up_limit_pub.publish(self.c1u_limit)
            self.upper_down_limit_pub.publish(self.c1d_limit)
            
            # 发布下舱门状态
            self.lower_motor_pub.publish(self.lower_motor_enabled)
            self.lower_up_limit_pub.publish(self.c2u_limit)
            self.lower_down_limit_pub.publish(self.c2d_limit)
            
            # 发布系统状态
            self.sys_state_pub.publish(self.sys_state)
            self.emergency_pub.publish(self.emergency_stop)
            
        except Exception as e:
            rospy.logwarn(f"发布状态失败: {e}")
    
    def publish_device_state(self):
        """发布设备状态数字码到delivery_device_state话题"""
        try:
            # 构建复合状态码
            # 格式：前2位: sys_state (0-99)
            #       后10位: 各个状态位 (每个状态1位)
            state_code = self.sys_state  # 基本系统状态 (0-99)
            
            # 如果系统状态超过99，则限制在99
            if state_code > 99:
                state_code = 99
                rospy.logwarn(f"系统状态码{self.sys_state}超过99，已限制为99")
            
            # 在日志中显示详细状态信息（便于调试）
            if rospy.get_time() % 5 < 0.1:  # 每5秒打印一次详细状态
                state_text = {
                    0: "正常",
                    1: "初始化中",
                    2: "急停中",
                    3: "电机重置中",
                    4: "上门上复位",
                    5: "上门下复位",
                    6: "下门上复位",
                    7: "下门下复位",
                    8: "上门向上移动",
                    9: "上门向下移动",
                    10: "下门向上移动",
                    11: "下门向下移动"
                }.get(self.sys_state, f"未知({self.sys_state})")
                
                if self.emergency_stop:
                    state_text = "⚠ 急停已触发 ⚠"
                
                rospy.loginfo(f"当前状态码: {state_code}, 状态描述: {state_text}")
            
            # 发布数字状态码
            msg = Int32()
            msg.data = state_code
            self.state_pub.publish(msg)
            
        except Exception as e:
            rospy.logwarn(f"发布设备状态失败: {e}")
    
    def send_serial_command(self, cmd, log_msg=""):
        """发送串口命令"""
        if not self.serial_connected or not self.ser:
            rospy.logwarn(f"发送命令失败: 串口未连接 ({log_msg})")
            return False
        
        try:
            self.ser.write(f"{cmd}\n".encode())
            if log_msg:
                rospy.loginfo(f"发送命令: {cmd} ({log_msg})")
            return True
        except Exception as e:
            rospy.logerr(f"发送命令失败: {e} ({log_msg})")
            return False
    
    # ====== 回调函数 ======
    
    def upper_motor_callback(self, msg):
        """上舱门电机控制回调"""
        if msg.data:  # True: 使能电机
            self.send_serial_command("E1", "上舱门使能电机")
            rospy.loginfo("收到命令: 上舱门使能电机")
        else:  # False: 禁用电机
            self.send_serial_command("D1", "上舱门禁用电机")
            rospy.loginfo("收到命令: 上舱门禁用电机")
    
    def lower_motor_callback(self, msg):
        """下舱门电机控制回调"""
        if msg.data:  # True: 使能电机
            self.send_serial_command("E2", "下舱门使能电机")
            rospy.loginfo("收到命令: 下舱门使能电机")
        else:  # False: 禁用电机
            self.send_serial_command("D2", "下舱门禁用电机")
            rospy.loginfo("收到命令: 下舱门下禁电机")
    
    def upper_reset_callback(self, msg):
        """上舱门复位控制回调"""
        # 消息格式: "up,220" 或 "down,220"
        try:
            parts = msg.data.split(',')
            direction = parts[0].strip().lower()
            
            if len(parts) > 1:
                distance = float(parts[1].strip())
                if distance <= 0 or distance > 1000:
                    rospy.logwarn(f"复位距离无效: {distance}mm，使用默认值220mm")
                    cmd = "C1U" if direction == "up" else "C1D"
                else:
                    cmd = f"C1U,{distance}" if direction == "up" else f"C1D,{distance}"
            else:
                cmd = "C1U" if direction == "up" else "C1D"
            
            operation = f"上舱门{'上' if direction == 'up' else '下'}复位"
            self.send_serial_command(cmd, operation)
            rospy.loginfo(f"收到命令: {operation}")
            
        except Exception as e:
            rospy.logerr(f"解析上舱门复位命令失败: {e}, 消息: {msg.data}")
    
    def lower_reset_callback(self, msg):
        """下舱门复位控制回调"""
        # 消息格式: "up,220" 或 "down,220"
        try:
            parts = msg.data.split(',')
            direction = parts[0].strip().lower()
            
            if len(parts) > 1:
                distance = float(parts[1].strip())
                if distance <= 0 or distance > 1000:
                    rospy.logwarn(f"复位距离无效: {distance}mm，使用默认值220mm")
                    cmd = "C2U" if direction == "up" else "C2D"
                else:
                    cmd = f"C2U,{distance}" if direction == "up" else f"C2D,{distance}"
            else:
                cmd = "C2U" if direction == "up" else "C2D"
            
            operation = f"下舱门{'上' if direction == 'up' else '下'}复位"
            self.send_serial_command(cmd, operation)
            rospy.loginfo(f"收到命令: {operation}")
            
        except Exception as e:
            rospy.logerr(f"解析下舱门复位命令失败: {e}, 消息: {msg.data}")
    
    def upper_control_callback(self, msg):
        """上舱门移动控制回调"""
        # 消息格式: "up,10" 或 "down,10"
        try:
            parts = msg.data.split(',')
            direction = parts[0].strip().lower()
            
            if len(parts) > 1:
                distance = float(parts[1].strip())
                if distance <= 0:
                    rospy.logwarn(f"移动距离无效: {distance}mm，使用默认值10mm")
                    cmd = "M1U" if direction == "up" else "M1D"
                else:
                    cmd = f"M1U,{distance}" if direction == "up" else f"M1D,{distance}"
            else:
                cmd = "M1U" if direction == "up" else "M1D"
            
            operation = f"上舱门向{'上' if direction == 'up' else '下'}移动"
            self.send_serial_command(cmd, operation)
            rospy.loginfo(f"收到命令: {operation}")
            
        except Exception as e:
            rospy.logerr(f"解析上舱门移动命令失败: {e}, 消息: {msg.data}")
    
    def lower_control_callback(self, msg):
        """下舱门移动控制回调"""
        # 消息格式: "up,10" 或 "down,10"
        try:
            parts = msg.data.split(',')
            direction = parts[0].strip().lower()
            
            if len(parts) > 1:
                distance = float(parts[1].strip())
                if distance <= 0:
                    rospy.logwarn(f"移动距离无效: {distance}mm，使用默认值10mm")
                    cmd = "M2U" if direction == "up" else "M2D"
                else:
                    cmd = f"M2U,{distance}" if direction == "up" else f"M2D,{distance}"
            else:
                cmd = "M2U" if direction == "up" else "M2D"
            
            operation = f"下舱门向{'上' if direction == 'up' else '下'}移动"
            self.send_serial_command(cmd, operation)
            rospy.loginfo(f"收到命令: {operation}")
            
        except Exception as e:
            rospy.logerr(f"解析下舱门移动命令失败: {e}, 消息: {msg.data}")
    
    def system_init_callback(self, msg):
        """系统初始化回调"""
        # 消息格式: "220,220" 或 "INIT"
        try:
            if msg.data.lower() == "init":
                self.send_serial_command("INIT", "系统初始化（默认距离）")
                rospy.loginfo("收到命令: 系统初始化（默认距离）")
            else:
                parts = msg.data.split(',')
                if len(parts) >= 2:
                    upper_dist = float(parts[0].strip())
                    lower_dist = float(parts[1].strip())
                    
                    if upper_dist <= 0 or upper_dist > 1000:
                        rospy.logwarn(f"上门初始化距离无效: {upper_dist}mm，使用默认值220mm")
                        upper_dist = 220.0
                    if lower_dist <= 0 or lower_dist > 1000:
                        rospy.logwarn(f"下门初始化距离无效: {lower_dist}mm，使用默认值220mm")
                        lower_dist = 220.0
                    
                    cmd = f"INIT,{upper_dist},{lower_dist}"
                    self.send_serial_command(cmd, f"系统初始化（上门:{upper_dist}mm, 下门:{lower_dist}mm）")
                    rospy.loginfo(f"收到命令: 系统初始化（上门:{upper_dist}mm, 下门:{lower_dist}mm）")
                else:
                    self.send_serial_command("INIT", "系统初始化（默认距离）")
                    rospy.loginfo("收到命令: 系统初始化（默认距离）")
                    
        except Exception as e:
            rospy.logerr(f"解析系统初始化命令失败: {e}, 消息: {msg.data}")
    
    def motor_reset_callback(self, msg):
        """电机重置回调"""
        if msg.data:  # True: 执行重置
            self.send_serial_command("R", "电机重置")
            rospy.loginfo("收到命令: 电机重置")
    
    def cleanup(self):
        """清理资源"""
        rospy.loginfo("正在清理资源...")
        
        self.stop_serial_thread = True
        self.serial_connected = False
        
        if self.ser:
            try:
                self.ser.close()
            except:
                pass
        
        if self.serial_thread and self.serial_thread.is_alive():
            self.serial_thread.join(timeout=1.0)
        
        rospy.loginfo("资源清理完成")
    
    def run(self):
        """运行主循环"""
        rospy.loginfo("送物模块ROS节点启动")
        
        rate = rospy.Rate(10)  # 10Hz
        
        while not rospy.is_shutdown():
            try:
                # 定期发布状态
                self.publish_device_state()
                
                # 检查串口连接
                if not self.serial_connected:
                    rospy.logwarn("串口未连接，尝试重新连接...")
                    self.connect_serial()
                
                rate.sleep()
                
            except rospy.ROSInterruptException:
                break
            except Exception as e:
                rospy.logerr(f"主循环异常: {e}")
                rate.sleep()


def main():
    """主函数"""
    node = None
    
    try:
        node = DeliveryModuleNode()
        node.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("ROS节点被中断")
    except Exception as e:
        rospy.logerr(f"节点运行异常: {e}")
    finally:
        if node:
            node.cleanup()
        rospy.loginfo("送物模块ROS节点已停止")


if __name__ == '__main__':
    main()