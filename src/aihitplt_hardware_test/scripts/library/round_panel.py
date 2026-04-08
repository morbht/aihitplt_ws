#!/usr/bin/env python3
import rospy, serial, struct, yaml, os
from std_msgs.msg import Bool

class RoundPanelDetector:
    def __init__(self):
        rospy.init_node('round_panel', anonymous=True)
        self.pub = rospy.Publisher('round_panel', Bool, queue_size=10)

        self.port, self.baudrate = self._load_config_from_yaml()
        
        rospy.loginfo('使用串口: %s @ %d', self.port, self.baudrate)

        self.click_detected = False
        self.setup_serial()

    def _load_config_from_yaml(self):
        """从YAML配置文件加载串口参数"""
        # 默认值
        port = '/dev/ttyUSB0'
        baudrate = 115200
        
        # 配置文件路径
        config_file = '/home/aihit/aihitplt_ws/src/aihitplt_hardware_test/config/knob_port.yaml'
        
        if os.path.exists(config_file):
            try:
                with open(config_file, 'r') as f:
                    cfg = yaml.safe_load(f) or {}
                
                # 从配置文件获取参数，使用默认值作为后备
                port = cfg.get('port') or cfg.get('security_sensor_port') or port
                baudrate = cfg.get('baudrate') or baudrate
                
                rospy.loginfo(f"从配置文件加载: {config_file}")
            except Exception as e:
                rospy.logerr(f"读取配置文件失败: {e}")
                rospy.loginfo(f"使用默认值: {port} @ {baudrate}")
        else:
            rospy.logwarn(f"未找到配置文件 {config_file}，使用默认值: {port} @ {baudrate}")
        
        return port, baudrate

    def setup_serial(self):
        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                bytesize=serial.EIGHTBITS,
                timeout=0.1
            )
            rospy.loginfo('成功连接圆形屏: %s', self.port)
        except serial.SerialException as e:
            rospy.logerr('无法打开串口 %s: %s', self.port, e)
            rospy.signal_shutdown('串口连接失败')

    def read_panel_data(self):
        try:
            data = self.ser.read(2)
            if len(data) == 2:
                value = struct.unpack('>H', data)[0]
                if value == 0x0001:
                    return True
        except serial.SerialException as e:
            rospy.logwarn('串口读取错误: %s', e)
            self.setup_serial()
        return False

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.click_detected = self.read_panel_data()
            msg = Bool(data=self.click_detected)
            self.pub.publish(msg)
            if self.click_detected:
                rospy.loginfo('检测到点击事件: True')
            rate.sleep()
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()

if __name__ == '__main__':
    try:
        RoundPanelDetector().run()
    except rospy.ROSInterruptException:
        pass