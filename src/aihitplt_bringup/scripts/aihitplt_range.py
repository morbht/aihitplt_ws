#!/usr/bin/env python3
# aihit_range_converter.py
import rospy
import math
from sensor_msgs.msg import Range
from std_msgs.msg import Float32, Header
from aihitplt_bringup.msg import Supersonic

class AihitRangeConverter:
    def __init__(self):
        rospy.init_node('aihit_range_converter')
        
        # === 红外传感器 ===
        rospy.Subscriber('/ir_distance', Float32, self.ir_callback)
        self.ir_range_pub = rospy.Publisher('/ir_range', Range, queue_size=10)
        
        # 红外传感器参数
        self.ir_frame_id = "anti_fall_link"  # 根据URDF
        self.ir_fov = math.radians(5)  # 视场角5度（弧度）
        self.ir_min_range = 0.02  # 最小测距（米）
        self.ir_max_range = 2.0   # 最大测距（米）
        
        # === 超声波传感器 ===
        rospy.Subscriber('/Distance', Supersonic, self.ultrasonic_callback)
        
        # 6个超声波发布器 (A-F)
        self.ultrasonic_pubs = []
        self.ultrasonic_frame_ids = [
            "ultrasonic_left_front_link",    # A - 左前
            "ultrasonic_right_front_link",   # B - 右前
            "ultrasonic_left_link",          # C - 左侧
            "ultrasonic_right_link",         # D - 右侧
            "ultrasonic_left_back_link",     # E - 左后
            "ultrasonic_right_back_link"     # F - 右后
        ]
        
        for i in range(6):
            pub = rospy.Publisher(f'/ultrasonic_range_{i}', Range, queue_size=10)
            self.ultrasonic_pubs.append(pub)
        
        # 超声波参数
        self.ultrasonic_fov = math.radians(15)  # 15度视场角
        self.ultrasonic_min_range = 0.02  # 最小测距
        self.ultrasonic_max_range = 3.0   # 最大测距
        
        rospy.spin()
    
    def create_range_msg(self, distance, frame_id, fov_rad, min_range, max_range, radiation_type):
        """创建Range消息"""
        range_msg = Range()
        range_msg.header = Header()
        range_msg.header.stamp = rospy.Time.now()
        range_msg.header.frame_id = frame_id
        
        range_msg.radiation_type = radiation_type  # 辐射类型
        range_msg.field_of_view = fov_rad          # 视场角（弧度）
        range_msg.min_range = min_range            # 最小距离
        range_msg.max_range = max_range            # 最大距离
        
        # 限制距离在有效范围内
        if distance < min_range:
            range_msg.range = min_range
        elif distance > max_range:
            range_msg.range = max_range
        else:
            range_msg.range = distance
        
        return range_msg
    
    def ir_callback(self, msg):
        """处理红外传感器数据"""
        try:
            # 红外传感器通常测量正前方距离
            range_msg = self.create_range_msg(
                distance=msg.data,
                frame_id=self.ir_frame_id,
                fov_rad=self.ir_fov,
                min_range=self.ir_min_range,
                max_range=self.ir_max_range,
                radiation_type=Range.INFRARED
            )
            
            self.ir_range_pub.publish(range_msg)
            

            
        except Exception as e:
            rospy.logerr(f"红外数据转换错误: {e}")
    
    def ultrasonic_callback(self, msg):
        """处理超声波传感器数据"""
        try:
            # 提取6个超声波传感器的距离值
            distances = [
                msg.distanceA,  # A
                msg.distanceB,  # B
                msg.distanceC,  # C
                msg.distanceD,  # D
                msg.distanceE,  # E
                msg.distanceF   # F
            ]
            
            # 传感器名称对应
            sensor_names = ['A', 'B', 'C', 'D', 'E', 'F']
            
            # 发布每个传感器的Range消息
            for i in range(6):
                range_msg = self.create_range_msg(
                    distance=distances[i],
                    frame_id=self.ultrasonic_frame_ids[i],
                    fov_rad=self.ultrasonic_fov,
                    min_range=self.ultrasonic_min_range,
                    max_range=self.ultrasonic_max_range,
                    radiation_type=Range.ULTRASOUND
                )
                
                # 修复：使用列表中的发布器
                self.ultrasonic_pubs[i].publish(range_msg)
            
            # 可选：调试输出（每2秒打印一次）
            current_time = rospy.get_time()
            if not hasattr(self, 'last_print_time'):
                self.last_print_time = current_time
            
            if current_time - self.last_print_time > 2.0:
                self.last_print_time = current_time
                distances_str = ", ".join([f"{n}:{d:.3f}m" for n, d in zip(sensor_names, distances)])
            
        except Exception as e:
            rospy.logerr(f"超声波数据转换错误: {e}")
            import traceback
            rospy.logerr(traceback.format_exc())

if __name__ == "__main__":
    try:
        AihitRangeConverter()
    except rospy.ROSInterruptException:
        rospy.loginfo("传感器转换器关闭")