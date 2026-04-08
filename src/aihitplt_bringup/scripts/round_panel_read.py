#!/usr/bin/env python3
import serial
import time

def read_round_panel():
    try:
        # 打开圆形屏串口
        ser = serial.Serial(
            port='/dev/ttyUSB0',
            baudrate=115200,      # 根据您的屏幕设置波特率
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=1           # 读取超时时间
        )
        
        print("等待圆形屏点击事件...")
        
        while True:
            # 读取数据
            data = ser.read(2)  # 读取2字节，因为0x0001是2字节
            if data:
                # 将字节数据转换为整数
                value = int.from_bytes(data, byteorder='big')
                print(f"收到点击事件: 0x{value:04x} (十进制: {value})")
                
                # 判断是否是点击事件
                if value == 0x0001:
                    print("✅ 检测到屏幕点击！执行相应操作...")
                    # 在这里添加您的点击处理逻辑
                    
    except serial.SerialException as e:
        print(f"串口错误: {e}")
    except KeyboardInterrupt:
        print("程序退出")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()

if __name__ == "__main__":
    read_round_panel()
