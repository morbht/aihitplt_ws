#!/usr/bin/env python3
import serial
import struct
import time
from typing import Optional, Dict

# 常量定义
FRAME_HEADER = b'\xAA\x55'  # 帧头标识
FRAME_SIZE = 55             # 数据帧总字节数
BAUDRATE = 115200           # 串口波特率
SERVO_CMD_PREFIX = "P:"     # 舵机控制命令前缀

# 数据帧格式定义 (使用小端字节序)
# 格式说明:
# < : 小端字节序
# 2s: 2字节帧头(AA 55)
# I : 4字节无符号整数(时间戳)
# 8H: 8个2字节无符号整数(酒精、烟雾、光照、eCO2、eCH2O、TVOC、PM25、PM10)
# 2f: 2个4字节浮点数(温度、湿度)
# 6f: 6个4字节浮点数(3轴加速度+3轴陀螺仪)
# B : 1字节校验和
FRAME_FORMAT = '<2sI 8H 2f 6f B'

class ESP32Controller:
    """ESP32 传感器数据读取与舵机控制类
    
    功能:
    - 通过串口读取ESP32发送的传感器数据帧
    - 解析并验证传感器数据
    - 发送舵机控制命令
    
    使用方法:
    1. 创建实例: controller = ESP32Controller(port='COM3')
    2. 读取数据: data = controller.read_sensor_data()
    3. 控制舵机: controller.set_servo_angle(horizontal=90, vertical=90)
    4. 关闭连接: controller.close()
    """
    
    def __init__(self, port: str, baudrate: int = BAUDRATE):
        """初始化串口连接
        
        参数:
            port: 串口设备路径 (如 'COM3' 或 '/dev/ttyUSB0')
            baudrate: 串口波特率 (默认115200)
        """
        self.ser = serial.Serial(port, baudrate, timeout=1)
        time.sleep(2)  # 等待串口稳定
        print(f"已连接到 {port}，波特率 {baudrate}")

    def parse_sensor_frame(self, frame: bytes) -> Optional[Dict]:
        """解析传感器数据帧
        
        参数:
            frame: 完整的55字节数据帧
            
        返回:
            解析后的传感器数据字典，如果解析失败返回None
        """
        try:
            # 解包二进制数据
            unpacked = struct.unpack(FRAME_FORMAT, frame)
            
            return {
                'header': unpacked[0],                # 帧头(AA 55)
                'timestamp': unpacked[1],             # 时间戳(ms)
                'alcohol': unpacked[2],               # 酒精浓度
                'smoke': unpacked[3],                 # 烟雾浓度
                'light': unpacked[4],                 # 光照强度
                'eCO2': unpacked[5],                 # CO2浓度(ppm)
                'eCH2O': unpacked[6],                # 甲醛浓度(μg/m³)
                'TVOC': unpacked[7],                 # TVOC浓度(μg/m³)
                'PM25': unpacked[8],                 # PM2.5浓度(μg/m³)
                'PM10': unpacked[9],                 # PM10浓度(μg/m³)
                'temperature': unpacked[10],          # 温度(℃)
                'humidity': unpacked[11],             # 湿度(%)
                'accel': unpacked[12:15],            # 3轴加速度(m/s²) [X,Y,Z]
                'gyro': unpacked[15:18],             # 3轴角速度(rad/s) [X,Y,Z]
                'checksum': unpacked[18]             # 校验和
            }
        except struct.error as e:
            print(f"帧解析错误: {e}")
            return None

    def verify_checksum(self, frame: bytes, checksum: int) -> bool:
        """验证数据帧校验和
        
        参数:
            frame: 完整的数据帧(包括帧头)
            checksum: 接收到的校验和
            
        返回:
            bool: 校验和是否有效
        """
        calculated = sum(frame[:-1]) & 0xFF  # 计算前54字节的和，取最低8位
        return calculated == checksum

    def read_sensor_data(self) -> Optional[Dict]:
        """从串口读取并解析一帧传感器数据
        
        返回:
            解析后的传感器数据字典，如果没有有效数据返回None
        """
        # 查找帧头
        while True:
            header = self.ser.read(2)
            if not header:
                return None
            if header == FRAME_HEADER:
                break
        
        # 读取帧剩余部分 (55-2=53字节)
        frame_data = self.ser.read(FRAME_SIZE - 2)
        if len(frame_data) != FRAME_SIZE - 2:
            print(f"数据不完整，收到 {len(frame_data)} 字节，需要 {FRAME_SIZE-2} 字节")
            return None
        
        # 组合完整帧
        full_frame = header + frame_data
        
        # 解析帧数据
        data = self.parse_sensor_frame(full_frame)
        if not data:
            return None
        
        # 验证校验和
        if not self.verify_checksum(full_frame, data['checksum']):
            print("校验和验证失败")
            return None
        
        return data

    def set_servo_angle(self, horizontal: int, vertical: int) -> bool:
        """设置舵机角度
        
        参数:
            horizontal: 上舵机角度 (0-180)
            vertical: 下舵机角度 (0-180)
            
        返回:
            bool: 命令是否成功发送
        """
        # 验证角度范围
        if not (0 <= horizontal <= 180 and 0 <= vertical <= 180):
            print("错误: 舵机角度必须在0-180之间")
            return False
        
        # 构造控制命令 (格式: "P:上角度,下角度\n")
        cmd = f"{SERVO_CMD_PREFIX}{horizontal},{vertical}\n"
        
        try:
            self.ser.write(cmd.encode())
            return True
        except serial.SerialException as e:
            print(f"发送舵机命令失败: {e}")
            return False

    def close(self):
        """关闭串口连接"""
        self.ser.close()
        print("串口连接已关闭")

def print_sensor_data(data: Dict):
    """格式化打印传感器数据
    
    参数:
        data: 传感器数据字典
    """
    print("\n=== 传感器数据 ===")
    print(f"时间戳: {data['timestamp']}ms")
    print(f"环境 - 酒精: {data['alcohol']} 烟雾: {data['smoke']} 光照: {data['light']}")
    print(f"空气质量 - CO2: {data['eCO2']}ppm  甲醛: {data['eCH2O']}μg/m³  TVOC: {data['TVOC']}μg/m³")
    print(f"颗粒物 - PM2.5: {data['PM25']}μg/m³  PM10: {data['PM10']}μg/m³")
    print(f"温湿度 - 温度: {data['temperature']:.1f}°C  湿度: {data['humidity']:.1f}%")
    print("加速度 (m/s²): X={:.2f}  Y={:.2f}  Z={:.2f}".format(*data['accel']))
    print("角速度 (rad/s): X={:.4f}  Y={:.4f}  Z={:.4f}".format(*data['gyro']))

def main():
    """主函数示例"""
    # 替换为你的实际串口设备
    # PORT = 'COM3'  # Windows示例
    PORT = '/dev/ttyUSB2'  # Linux示例
    
    # 创建控制器实例
    controller = ESP32Controller(port=PORT)
    
    try:
        # 示例: 每2秒读取一次数据并交替设置舵机角度
        angle = 90
        while True:
            # 读取传感器数据
            data = controller.read_sensor_data()
            if data:
                print_sensor_data(data)
            
            # 控制舵机 (在90度和45度之间交替)
            angle = 45 if angle == 90 else 90
            if controller.set_servo_angle(horizontal=angle, vertical=angle):
                print(f"\n设置舵机角度: 上={angle}°, 下={angle}°")
            
            time.sleep(2)  # 等待2秒
            
    except KeyboardInterrupt:
        print("\n用户中断程序")
    finally:
        controller.close()

if __name__ == '__main__':
    main()