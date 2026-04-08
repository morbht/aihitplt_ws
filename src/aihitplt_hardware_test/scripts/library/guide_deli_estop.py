#!/usr/bin/env python3
import rospy
import serial
import yaml
import os
from std_msgs.msg import Bool

def aihitplt_guide_delivery_emergency_node():
    # 初始化节点
    rospy.init_node('aihitplt_guide_delivery_emergency_node')
    
    # 获取串口参数
    port, baudrate = _load_config_from_yaml()
    rospy.loginfo(f"使用串口: {port} @ {baudrate}")
    
    # 创建发布者
    pub = rospy.Publisher('/e_stop', Bool, queue_size=10)
    
    try:
        # 连接串口
        ser = serial.Serial(port, baudrate, timeout=1)
        rospy.loginfo(f"Connected to {port}")
        
        # 主循环
        while not rospy.is_shutdown():
            if ser.in_waiting:
                data = ser.readline().decode('utf-8').strip()
                
                if data == "P":  # 急停按下
                    pub.publish(Bool(True))
                    rospy.logwarn("EMERGENCY STOP ACTIVATED!")
                elif data == "R":  # 急停释放
                    pub.publish(Bool(False))
                    rospy.loginfo("Emergency stop released")
                    
    except serial.SerialException as e:
        rospy.logerr(f"Serial error: {e}")
    except rospy.ROSInterruptException:
        pass
    finally:
        if 'ser' in locals():
            ser.close()

def _load_config_from_yaml():
    """从YAML配置文件加载串口参数"""
    # 默认值
    port = '/dev/ttyUSB2'
    baudrate = 115200
    
    # 尝试多个可能的配置文件路径
    config_paths = [
        os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
            'config', 'emergency_stop_port.yaml'
        ),
        '/home/aihit/aihitplt_ws/src/aihitplt_hardware_test/config/emergency_stop_port.yaml'
    ]
    
    config_file = None
    for path in config_paths:
        if os.path.exists(path):
            config_file = path
            break
    
    if config_file:
        try:
            with open(config_file, 'r') as f:
                cfg = yaml.safe_load(f) or {}
            
            # 从配置文件获取参数，使用默认值作为后备
            port = cfg.get('emergency_stop_port') or cfg.get('port') or port
            baudrate = cfg.get('baudrate') or baudrate
            
            rospy.loginfo(f"从配置文件加载: {config_file}")
        except Exception as e:
            rospy.logerr(f"读取配置文件失败: {e}")
            rospy.loginfo(f"使用默认值: {port} @ {baudrate}")
    else:
        rospy.logwarn(f"未找到配置文件，使用默认值: {port} @ {baudrate}")
    
    return port, baudrate

if __name__ == '__main__':
    aihitplt_guide_delivery_emergency_node()