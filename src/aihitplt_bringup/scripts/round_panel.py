#!/usr/bin/env python3
import rospy, serial, struct
from std_msgs.msg import Bool

class RoundPanelDetector:
    def __init__(self):
        rospy.init_node('round_panel', anonymous=True)
        self.pub = rospy.Publisher('round_panel', Bool, queue_size=10)

        self.port = rospy.get_param('~port', '/dev/ttyUSB1')   # 缺省与 launch 保持一致
        self.baudrate = rospy.get_param('~baudrate', 115200)

        rospy.loginfo('使用串口: %s @ %d', self.port, self.baudrate)

        self.click_detected = False
        self.setup_serial()

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