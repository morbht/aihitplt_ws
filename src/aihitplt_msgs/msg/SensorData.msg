Header header

# 基础传感器数据
uint32 timestamp
uint16 alcohol      # 酒精浓度
uint16 smoke        # 烟雾浓度  
uint16 light        # 光照强度
uint16 eco2         # CO2浓度(ppm)
uint16 ech2o        # 甲醛浓度(μg/m³)
uint16 tvoc         # TVOC浓度(μg/m³)
uint16 pm25         # PM2.5浓度(μg/m³)
uint16 pm10         # PM10浓度(μg/m³)
float32 temperature # 温度(℃)
float32 humidity    # 湿度(%)

# IMU数据
float32 accel_x     # X轴加速度(m/s²)
float32 accel_y     # Y轴加速度(m/s²)  
float32 accel_z     # Z轴加速度(m/s²)
float32 gyro_x      # X轴角速度(rad/s)
float32 gyro_y      # Y轴角速度(rad/s)
float32 gyro_z      # Z轴角速度(rad/s)
