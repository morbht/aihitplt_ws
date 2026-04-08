#!/usr/bin/env python3 

from pymycobot.ultraArmP340 import ultraArmP340

class aihitArm:
    def __init__(self, port='/dev/ttyUSB2', baud=115200):
        """
        初始化机械臂API
        :param port: 串口设备路径 
        :param baud: 波特率
        """
        self.arm = ultraArmP340(port, baud)
        print(f"UltraArm P340 initialized on {port} with baudrate {baud}")
        
    def go_zero(self):
        """
        将机械臂回零
        """
        self.arm.go_zero()
    def power_on(self):
        """
        将机械臂所有关节上电
        """
        self.arm.power_on()

    def release_all_servers(self):
        """
        将机械臂所有关节上电
        """
        self.arm.release_all_servos()
    
    def is_moving_end(self):
        """
        机械臂运动结束标志

        返回值：
            1: 运动结束
            0: 运动未结束
        """
        moving_end_flag = self.arm.is_moving_end()
        return moving_end_flag
        
    
    def set_system_value(self, id, address, value, mode=None):
        """
        设置系统参数

        参数说明：
            id: int, 电机ID，4 或者 7
            address: int, 参数寄存器地址，7 ~ 69
            value: int, 对应寄存器参数取值
            mode: int, 1 或者 2，可以为空，默认模式为1
                1: 设置范围为0-255，可使用地址21（P值）
                2: 设置取值范围0-65535，可使用地址56（设置位置）
        """
        self.arm.set_system_value(id, address, value, mode)
        
    def get_system_value(self,id, address, mode=None):
        """
        功能： 读取系统参数
        参数说明：
        id: int, 电机ID，4 或者 7
        address: int, 参数寄存器地址，0 ~ 69
        mode: int, 1 或者 2，可以为空，默认模式为1
            1: 读取范围为0-255，可使用地址21（P值）
            2: 读取取值范围0-65535，可使用地址56（读取位置）
        返回值： int, 对应寄存器参数取值
        """
        system_value = self.arm.get_system_value(id,address,mode)
        return system_value

    def get_system_version(self):
        """
        功能： 读取固件主次版本

        返回值： float, 固件版本号
        """
        system_version = self.arm.get_system_version()
        return system_version

    def get_modify_version(self):
        """
        功能： 读取固件更正版本号

        返回值： int, 更正版本号
        """
        modify_version = self.arm.get_modify_version()
        return modify_version

    def get_angles_info(self):
        """
        获取机械臂所有关节的当前角度
        
        return: 包含4个关节角度的列表 [j1, j2, j3, j4]
        """
        angles = self.arm.get_angles_info()
        # print(f"Current angles: {angles}")
        return angles
    
    def set_angle(self, id, degree, speed):
        """
        功能： 发送指定的单个关节运动至指定的角度
        参数说明：

        id: 代表机械臂的关节，三轴有三个关节，可以用数字1-3来表示。
        degree: 表示关节的角度

        关节 Id	范围
        1	-150 ~ 170
        2	-20 ~ 90
        3	-5 ~ 110
        4（配件）	-179 ~ 179
        speed：表示机械臂运动的速度，范围0~200 (单位：mm/s)。

        返回值： 无
        """
        self.arm.set_angle(id, degree, speed)

    def set_angles(self, degrees, speed):
        """
        功能： 发送所有角度给机械臂所有关节

        参数说明：
            degrees: (List[float])包含所有关节的角度 ,三轴机器人有三个关节，所以长度为3，表示方法为：[20,20,20]
            speed: 表示机械臂运动的速度，取值范围是0~200 (单位：mm/s)。

        功能： 发送所有角度给机械臂所有关节
        """
        self.arm.set_angles(degrees, speed)

    def get_coords_info(self):
        """
        功能： 获取机械臂当前坐标。

        返回值： list包含坐标的列表, θ 为末端的旋转角

        三轴：长度为 4，依次为 [x, y, z, θ]
        """
        coords_info = self.arm.get_coords_info()
        return coords_info
    
    def set_coord(self,id,coord,speed):
        """
        功能： 发送单个坐标值给机械臂进行移动
        参数说明：

            id:代表机械臂的坐标，三轴有三个坐标，有特定的表示方法。 X坐标的表示法："X".
            coord: 输入您想要到达的坐标值

            坐标 Id	范围
            X	-360 ~ 365.55
            Y	-365.55 ~ 365.55
            Z	-140 ~ 130
            speed: 表示机械臂运动的速度，范围是0-200 (单位：mm/s)。

            返回值： 无
        """
        self.arm.set_coord(id,coord,speed)

    def set_coords(self, coords, speed):
        """
        功能： 发送整体坐标,让机械臂头部从原来点移动到您指定点。
        参数说明：
            coords:
            三轴：[x,y,z] 的坐标值，长度为3
            speed: 表示机械臂运动的速度，范围是0-200 (单位：mm/s)。
            返回值： 无
        """
        self.arm.set_coords(coords, speed)

    def get_radians_info(self):
        """
        功能： 获取机械臂当前弧度值。
        返回值： list包含所有关节弧度值的列表.
        """
        radians_info = self.arm.get_radians_info()
        return radians_info
    
    def set_radians(self,radians, speed):
        """
        功能： 发送弧度值给机械臂所有关节
        参数说明：

            radians: 每个关节的弧度值列表( List[float])

            关节 Id	范围
            1	2.6179 ~ 2.9670
            2	-0.3490 ~ 1.5707
            3	-0.0872 ~ 1.9198
            4（配件）	-3.1241 ~ + 3.1241
            speed: 表示机械臂运动的速度，范围是0-200 (单位：mm/s)。

            返回值： 无
        
        """
        self.arm.set_radians(radians,speed)

    def set_mode(self):
        """
        功能： 设置坐标模式
            参数说明：
            0:绝对笛卡尔模式。
            1:相对笛卡尔模式。
        返回值： 无
        """
        self.arm.set_mode()

    def sleep(self):
        """
        功能： 延迟
            参数说明：
            Time: 延迟的时间( Int类型)，
        返回值： 无
        """
        self.arm.sleep()

    def set_init_pose(self):
        """
        功能： 设置当前位置为某个固定位置。如[0,0,0]就把这个位置设为零点
        参数说明：
            coords (list): 机械臂所有坐标，比如[0, 0, 0]
            speed: 表示机械臂运动的速度，范围是0-200 (单位：mm/s)。
        返回值： 无
        """
        self.arm.set_init_pose()

    def set_pwm(self):
        """
        功能： 设置PWM占空比
        参数说明： P：占空比，范围：0-255
        返回值： 无
        """
        self.arm.set_pwm()

    def set_speed_mode(self, mode):
        """
        功能： 设置速度模式
        参数说明：
            0: 匀速模式
            2: 加减速模式
        返回值： 无
        """
        self.arm.set_speed_mode(mode)

    def set_jog_angle(self, id, direction, speed):
        """"
        功能： 设置点动模式（角度）
        参数说明：
            id: 代表机械臂的关节，按照关节id给入1~3来表示
            direction: 主要控制机器臂移动的方向，0 - 正向移动，1 - 负向移动
            speed: 速度 0 ~ 200 (单位：mm/s)。
        返回值： 无
        """
        self.arm.set_jog_angle(id, direction, speed)

    
    def set_jog_coord(self, axis, direction, speed):
        """
        功能： 控制机器人按照指定的坐标或姿态值持续移动
        参数说明：
            axis: 代表机械臂的关节，范围 1 ~ 3
            direction: 主要控制机器臂移动的方向，0 - 正向移动，1 - 负向移动
            speed: 速度 0 ~ 200 (单位：mm/s)。
        返回值： 无
        """
        self.arm.set_jog_coord(axis, direction, speed)


    def set_jog_stop(self):
        """
        功能： 停止 jog 控制下的持续移动
        返回值： 无
        """
        self.arm.set_jog_stop()

    def set_gripper_zero(self):
        """
        功能： 设置夹爪零位（设置当前位置为零位）。
        返回值： 无
        """
        self.arm.set_gripper_zero()

    def set_gripper_state(self,gripper_value, gripper_speed):
        """
        功能： 设置夹爪张开位置
        参数说明:
            gripper_value： int, 0 ~ 100。
            gripper_speed: 0 ~ 1500 RPM/s
        返回值： 无
        """
        self.arm.set_gripper_state(gripper_value, gripper_speed)

    def get_gripper_angle(self):
        """
        功能： 获取夹爪角度
        返回值： 夹爪角度值
        """
        gripper_angle = self.arm.get_gripper_angle()
        return gripper_angle
    
    def gripper_release(self):
        """
        功能： 放松夹爪
        返回值： 无
        """
        self.arm.set_gripper_release()



if __name__ == "__main__":
    # 创建API实例
    arm_api = aihitArm()
    
    arm_api.arm.go_zero()

    # 获取角度信息
    # angles = arm_api.get_angles_info()
    
#     # 格式化打印角度
#     arm_api.print_angles()