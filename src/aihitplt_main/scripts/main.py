#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import sys,os,rospkg,rospy,time   # 导入所需的库
sys.path.append(os.path.join(rospkg.RosPack().get_path('aihitplt_main'), 'scripts'))   # 将脚本所在目录添加到系统路径中
from sub.aihitplt_motion_api import *
from sub.aihitplt_function_api import *
from sub.aihitplt_arm_api import *
from sub.aihitplt_wel_food_api import *
from sub.aihitplt_delivery_api import *
from sub.aihitplt_security_api import * 
from sub.aihitplt_spray_api import *
from sub.aihitplt_logi_scale_api import *
from sub.aihitplt_ai_api import *
class test():
    def __init__(self): 
        # 创建 类的实例
        self.motion = aihitplt_motion_api()
        self.function = aihitplt_function_api()
        self.arm = aihitplt_arm_api()
        self.security = aihitplt_security_api()
        self.spray = aihitplt_spray_api()
        self.deli = aihitplt_delivery_api()
        self.logi = aihitplt_logi_scale_api()
        self.wel_deli = aihitplt_wel_food_api()
        self.muti_ai = aihitplt_ai_api()
        # 定义 AR 标记名称列表
        self.marker_name = ["ar_marker_0","ar_marker_1","ar_marker_2","ar_marker_3","ar_marker_4","ar_marker_5","ar_marker_6","ar_marker_7"]
        # 等待 0.1 秒，以确保初始化完成
        rospy.sleep(0.1)
        # 调用 start 方法开始执行任务
        self.start()

    def start(self):
        
        # AGV运动控制API
        # target_pose = [0.139, -2.02, 0, 0, 0, 0.57, 0.81]
        # self.motion.navigation_target(target_pose)

        # target_pose = [0.139, -2.02, 0, 0, 0, 0.57, 0.81]
        # self.motion.nav_to_target(target_pose)
        
        # self.motion.set_speed(0.1,0)
        
        # self.motion.ultrasonic_cal(0.1, 10)
        
        # args = [0,0,0,0,0,0,1.0]
        # self.motion.rviz_pose_setting(args)
        
        # self.motion.get_target_pose()
        
        # self.motion.leave_target("back")
        
        # self.motion.find_AR_marker("ar_marker_1")
        
        # self.motion.approach("ar_marker_1",3)
        
        # target = [0.891,0.649,0]
        # self.motion.turn_to_target(target)
        
        # self.motion.linear_move(0.5,0.1)
        
        # self.motion.rotate_to_angle(180,0.3)
        
        # AGV功能API
        # self.function.pan_cam_image_save("/home/aihit/aihitplt_ws/src/aihitplt_hardware_test/img/3-1.jpeg","/pan_tilt_camera/image")
        
        # robot = aihitplt_function_api()    
        # success = robot.yolo_data(1)  
        # print(f"发送指令成功: {success}")
        # result = robot.get_yolo_result()  
        # print(f"YOLO检测结果: {result}")
        # rospy.sleep(5)
        # robot.yolo_data(0)
        
        # self.function.voice_off_line()
        # result = self.function.voice_off_line()
        # if result == True:
        #     print(f"获取到语音结果: {result}")
            
        # self.function.play_sound("/home/aihit/aihitplt_test/test.wav")
        
        # self.function.wait_for_round_panel_click()
        
        # 机械臂模块API
        # self.arm.arm_joint_deg_control(50,60,70,0)    
        
        # self.arm.arm_joint_rad_control(0.5,0.5,0,0)
        
        # self.arm.arm_pose_control(150,100,50)
        
        # self.arm.close_gripper()
        
        # self.arm.go_to_target()
        
        # self.arm.open_gripper()
        
        # self.arm.gripper_control()
        
        # 安防模块API
        # self.security.cam_move("left",speed=7)
        
        # rospy.sleep(2)
        
        # self.security.cam_move("right",speed=3)
        
        # rospy.sleep(2)
        
        # self.security.cam_stop()
        
        # self.security.light_control(True)
        
        # self.security.wiper_control(True)
        
        # self.security.image_capture("/home/aihit/aihitplt_ws/src/aihitplt_main/img/image01.jpg","/pan_tilt_camera/image")
        
        # self.security.adjust_focus("1")
        
        # self.security.adjust_aperture("7")
        
        # self.security.preset_point_control("go,1")

        # self.security.get_sensor_data() 
        
        # self.security.get_estop_state()
        
        # 送物模块API
        # self.deli.upper_control("up,50")
        
        # self.deli.upper_reset("up")
        
        # self.deli.upper_reset("down")
        
        # self.deli.lower_control("up ,50")
        
        # self.deli.lower_reset("down")
        
        # self.deli.initialize_system("220,220")

        # self.deli.get_door_state("upper")

        # self.deli.get_system_state()

        # self.deli.estop_state()

        # 工业物流模块API
        # self.logi.get_weight()
    
        # self.logi.estop_state()

        # self.logi.tare_scale()
        
        # self.logi.cali_scare(200)
        
        # e = self.wel_deli.estop_state()
        # print(f"{e}")
        
        # 喷雾模块API
        # self.spray.spray_control(True)
        
        # 迎宾、送餐模块API
        # self.wel_deli.estop_state()
        
        # AI开发套件API
        # self.muti_ai.start_voice_system()

        # self.muti_ai.stop_voice_system()

        # self.muti_ai.get_sensor_data()
        
        # self.muti_ai.send_pan_tilt_cmd("w")
        
        # self.muti_ai.set_pan_tilt_angle("45","45")
        
        # self.muti_ai.reset_pan_tilt()
        
        # self.muti_ai.start_multi_camera()
        
        # self.muti_ai.stop_multi_camera()
        
        
        pass


# 程序入口
if __name__ == "__main__":
    try:
        # 初始化 ROS 节点，节点名称为 'aihitplt_main'，匿名化处理
        # 'anonymous=True' 意味着如果该节点已经存在，则会自动生成一个唯一名称的新节点
        rospy.init_node('aihitplt_main', anonymous=True)  

        # 创建 test 类的实例并调用其构造函数
        # 该实例会启动类中的功能，例如传感器数据获取、机器人运动控制等
        test()
        
    except rospy.ROSInterruptException:   # 捕捉 ROS 中断异常
        # 发生异常时，不执行任何操作，程序正常退出
        pass

