#!/usr/bin/env python3
import rospy
import serial
import struct
import time
import yaml
import os
from typing import Optional, Dict

# 唯一需要的 ROS 消息
from std_msgs.msg import String

# 协议常量
FRAME_HEADER = b'\xAA\x55'
FRAME_SIZE   = 55
BAUDRATE     = 115200
FRAME_FORMAT = '<2sI 8H 2f 6f B'


class PanTiltSensorNode:

    def __init__(self):
        rospy.init_node('multi_ai_sensor', anonymous=True)

        # 加载串口参数
        self.port, self.baudrate = self._load_config_from_yaml()
        rospy.loginfo(f"使用串口: {self.port} @ {self.baudrate}")

        # 串口
        self.ser = None
        self.setup_serial()

        
        self.pub_all = rospy.Publisher('/multi_ai/sensor_data', String, queue_size=10)

        # 10 Hz 循环
        self.rate = rospy.Rate(10)

    # ---------- 配置加载 ----------
    def _load_config_from_yaml(self):
        config_file = os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
            'config', 'pan_tilt_port.yaml')
        if not os.path.exists(config_file):
            config_file = '/home/aihit/aihitplt_ws/src/aihitplt_hardware_test/config/multi_ai_port.yaml'

        port, baud = '/dev/ttyUSB2', BAUDRATE
        if os.path.exists(config_file):
            try:
                with open(config_file) as f:
                    cfg = yaml.safe_load(f) or {}
                port = cfg.get('port') or cfg.get('device') or port
                baud = cfg.get('baudrate') or baud
            except Exception as e:
                rospy.logerr(f"读取配置文件失败: {e}")
        return port, baud

    # ---------- 串口初始化 ----------
    def setup_serial(self):
        self.ser = serial.Serial(port=self.port,
                                 baudrate=self.baudrate,
                                 timeout=1)
        time.sleep(2)  # 等稳定
        rospy.loginfo(f"成功连接到端口: {self.port}")

    # ---------- 帧解析 ----------
    def parse_sensor_frame(self, frame: bytes) -> Optional[Dict]:
        try:
            u = struct.unpack(FRAME_FORMAT, frame)
            return {
                'header'     : u[0],
                'timestamp'  : u[1],
                'alcohol'    : u[2],
                'smoke'      : u[3],
                'light'      : u[4],
                'eCO2'       : u[5],
                'eCH2O'      : u[6],
                'TVOC'       : u[7],
                'PM25'       : u[8],
                'PM10'       : u[9],
                'temperature': u[10],
                'humidity'   : u[11],
                'accel'      : u[12:15],
                'gyro'       : u[15:18],
                'checksum'   : u[18]
            }
        except struct.error:
            return None

    def verify_checksum(self, frame: bytes, checksum: int) -> bool:
        return (sum(frame[:-1]) & 0xFF) == checksum

    # ---------- 串口读取 ----------
    def read_sensor_data(self) -> Optional[Dict]:
        if not self.ser or not self.ser.is_open:
            return None
        try:
            header = self.ser.read(2)
            if header != FRAME_HEADER:
                return None
            rest = self.ser.read(FRAME_SIZE - 2)
            if len(rest) != FRAME_SIZE - 2:
                return None
            full = header + rest
            data = self.parse_sensor_frame(full)
            if not data or not self.verify_checksum(full, data['checksum']):
                return None
            return data
        except serial.SerialException as e:
            rospy.logerr(f"串口读取错误: {e}")
            return None

    # ---------- 拼单行字符串 ----------
    def create_oneline(self, data: Dict) -> String:
        msg = String()
        msg.data = (
            f'timestamp:"{data["timestamp"]}" '
            f'temperature:"{data["temperature"]:.2f}" '
            f'humidity:"{data["humidity"]:.2f}" '
            f'alcohol:"{data["alcohol"]}" '
            f'smoke:"{data["smoke"]}" '
            f'light:"{data["light"]}" '
            f'eCO2:"{data["eCO2"]}" '
            f'eCH2O:"{data["eCH2O"]}" '
            f'TVOC:"{data["TVOC"]}" '
            f'PM25:"{data["PM25"]}" '
            f'PM10:"{data["PM10"]}" '
            f'accel:"{data["accel"][0]:.3f},{data["accel"][1]:.3f},{data["accel"][2]:.3f}" '
            f'gyro:"{data["gyro"][0]:.3f},{data["gyro"][1]:.3f},{data["gyro"][2]:.3f}"'
        )
        return msg

    # ---------- 主循环 ----------
    def run(self):
        if not self.ser or not self.ser.is_open:
            rospy.logerr("串口未连接，无法启动")
            return

        while not rospy.is_shutdown():
            data = self.read_sensor_data()
            if data:
                self.pub_all.publish(self.create_oneline(data))
            self.rate.sleep()

    # ---------- 清理 ----------
    def cleanup(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
            rospy.loginfo("串口连接已关闭")


# -------------------- 入口 --------------------
def main():
    node = None
    try:
        node = PanTiltSensorNode()
        node.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("ROS 中断")
    except Exception as e:
        rospy.logerr(f"程序异常: {e}")
    finally:
        if node:
            node.cleanup()


if __name__ == '__main__':
    main()