#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32, Bool
from nav_msgs.msg import Odometry
from aihitplt_bringup.msg import supersonic  # 根据您的包名调整

class SensorTester:
    def __init__(self):
        rospy.init_node('sensor_tester', anonymous=True)
        self.sensors = {}
        self.rate = rospy.Rate(10)  # 1Hz
        
    def add_sensor(self, sensor_name, topic_name, msg_type, data_processor=None):
        """添加传感器到测试列表"""
        self.sensors[sensor_name] = {
            'topic': topic_name,
            'msg_type': msg_type,
            'data_processor': data_processor,
            'latest_data': None,
            'subscriber': None
        }
    
    def start_test(self):
        """开始传感器测试"""
        print("----开始传感器测试----")
        
        # 为每个传感器创建订阅者
        for sensor_name, config in self.sensors.items():
            config['subscriber'] = rospy.Subscriber(
                config['topic'], 
                config['msg_type'], 
                self._create_callback(sensor_name)
            )
            print(f"{sensor_name}：等待数据...")
        
        print()  # 空行
        
        try:
            while not rospy.is_shutdown():
                self._print_sensor_data()
                self.rate.sleep()
        except KeyboardInterrupt:
            print("\n----传感器测试结束----")
    
    def _create_callback(self, sensor_name):
        """为每个传感器创建回调函数"""
        def callback(data):
            if self.sensors[sensor_name]['data_processor']:
                processed_data = self.sensors[sensor_name]['data_processor'](data)
            else:
                processed_data = data.data
            self.sensors[sensor_name]['latest_data'] = processed_data
        return callback
    
    def _print_sensor_data(self):
        """打印所有传感器的当前数据"""
        for sensor_name, config in self.sensors.items():
            data = config['latest_data']
            if data is not None:
                if sensor_name == "圆形屏":
                    print(f"圆形屏：{data}")
                elif sensor_name == "防碰撞传感器":
                    print(f"防碰撞传感器：{data}")
                elif sensor_name == "充电状态":
                    print(f"充电状态：{data}")
                elif sensor_name == "电压值":
                    print(f"电压值：{data:.2f} V")
                elif sensor_name == "线速度":
                    print(f"线速度：{data:.6f} m/s")
                elif sensor_name == "红外距离":
                    print(f"红外距离：{data:.3f}m")
                elif sensor_name == "超声波传感器":
                    # 超声波数据是字典，单独处理
                    for us_key, us_value in data.items():
                        print(f"{us_key}：{us_value:.3f}m")
                else:
                    print(f"{sensor_name}：{data}")
            else:
                print(f"{sensor_name}：无数据")
        print()  # 每组数据后空一行

def extract_linear_velocity(odom_msg):
    """从Odometry消息中提取线速度的x分量"""
    return odom_msg.twist.twist.linear.x

def extract_ultrasonic_data(distance_msg):
    """从supersonic消息中提取超声波ABCDEF数据"""
    return {
        "超声波A": distance_msg.distanceA,
        "超声波B": distance_msg.distanceB,
        "超声波C": distance_msg.distanceC,
        "超声波D": distance_msg.distanceD,
        "超声波E": distance_msg.distanceE,
        "超声波F": distance_msg.distanceF
    }

def main():
    tester = SensorTester()
    
    # 添加圆形屏数据（布尔值）
    tester.add_sensor("圆形屏", "/round_panel", Bool)
    
    # 添加防碰撞传感器
    tester.add_sensor("防碰撞传感器", "/collision_sensor", Bool)
    
    # 添加充电状态
    tester.add_sensor("充电状态", "/robot_charging_flag", Bool)
    
    # 添加电压值
    tester.add_sensor("电压值", "/PowerVoltage", Float32)
    
    # 添加线速度（从odom中提取）
    tester.add_sensor("线速度", "/odom", Odometry, extract_linear_velocity)
    
    # 添加红外距离传感器
    tester.add_sensor("红外距离", "/ir_distance", Float32)
    
    # 添加超声波传感器（ABCDEF），使用正确的消息类型
    tester.add_sensor("超声波传感器", "/Distance", supersonic, extract_ultrasonic_data)
    
    # 启动测试
    tester.start_test()

if __name__ == '__main__':
    main()