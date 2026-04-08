#!/usr/bin/env python3
import math
import rospy
import os
import yaml
from sensor_msgs.msg import JointState
import pymycobot
from packaging import version

# min low version require
MAX_REQUIRE_VERSION = '3.9.1'
current_verison = pymycobot.__version__
print('current pymycobot library version: {}'.format(current_verison))

if version.parse(current_verison) > version.parse(MAX_REQUIRE_VERSION):
    from pymycobot.ultraArmP340 import ultraArmP340
    class_name = 'new'
else:
    from pymycobot.ultraArm import ultraArm
    class_name = 'old'
    print("Note: This class is no longer maintained since v3.6.0, please refer to the project documentation: https://github.com/elephantrobotics/pymycobot/blob/main/README.md")

ua = None


def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "%s", data.position)
    # print(data.position)
    data_list = []
    for index, value in enumerate(data.position):
        radians_to_angles = round(math.degrees(value), 2)
        data_list.append(radians_to_angles)
        
    rospy.loginfo(rospy.get_caller_id() + "%s", data_list)
    ua.set_angles(data_list, 25)


def _load_config_from_yaml():
    """从YAML配置文件加载串口参数"""
    # 默认值
    port = '/dev/ttyUSB0'
    baudrate = 115200
    
    # 尝试多个可能的配置文件路径
    config_paths = [
        os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
            'config', 'arm_config.yaml'
        ),
        '/home/aihit/aihitplt_ws/src/aihitplt_hardware_test/config/arm_config.yaml'
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
            port = cfg.get('arm_port') or cfg.get('port') or port
            baudrate = cfg.get('baudrate') or baudrate
            
            rospy.loginfo(f"从配置文件加载: {config_file}")
            rospy.loginfo(f"端口: {port}, 波特率: {baudrate}")
        except Exception as e:
            rospy.logerr(f"读取配置文件失败: {e}")
            rospy.loginfo(f"使用默认值: {port} @ {baudrate}")
    else:
        rospy.logwarn(f"未找到配置文件，使用默认值: {port} @ {baudrate}")
    
    return port, baudrate


def listener():
    global ua
    rospy.init_node("control_slider", anonymous=True)

    # 从YAML配置文件加载端口和波特率
    port, baud = _load_config_from_yaml()
    
    print(f"使用端口: {port}, 波特率: {baud}")
    
    # 连接机械臂
    try:
        if class_name == 'old':
            ua = ultraArm(port, baud)
        else:
            ua = ultraArmP340(port, baud)
            
        rospy.loginfo(f"成功连接到机械臂: {port}")
        ua.power_on()
        ua.go_zero()
        
    except Exception as e:
        rospy.logerr(f"连接机械臂失败: {e}")
        rospy.signal_shutdown("机械臂连接失败")
        return
    
    # 订阅关节状态话题
    rospy.Subscriber("joint_states", JointState, callback)

    # spin() simply keeps python from exiting until this node is stopped
    # spin() 只是阻止python退出，直到该节点停止
    print("spin ...")
    rospy.spin()


if __name__ == "__main__":
    listener()