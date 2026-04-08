# 功能包使用说明

## 1. 功能包说明

`aihiplt_deli` 功能包用于启动送物上装的相关节点

## 2. 使用说明

`roslaunch aihitplt_deli aihitplt_delivery_node.launch` #使用该命令启动送物上装节点

## 3. delivery_node.py参数解析

`self.serial_port = rospy.get_param('~serial_port', '/dev/ttyUSB0')` #该行代码用于指定送物上装串口设备路径，默认为`/dev/ttyUSB0`，如有需要，则根据实际情况更改为正确的设备路径

## 4. 话题说明

发布话题：

`delivery_device_state` 话题发布急停信号，发布设备状态码，用于表示系统当前运行状态，消息类型为`std_msgs/Int32`

状态码说明：
```
0 - 正常运行
1 - 初始化中
2 - 急停中
3 - 电机重置中
4 - 上门上复位中
5 - 上门下复位中
6 - 下门上复位中
7 - 下门下复位中
8 - 上门上移中
9 - 上门下移中
10 - 下门上移中
11 - 下门下移中
```

`delivery_system_state` 发布系统状态，与设备状态码对应，消息类型为`std_msgs/Int32`

`emergency_stop` 话题发布急停信号，消息类型为`std_msgs/Bool`，`True`表示急停状态，`False`表示非急停状态

`upper_motor_state` 话题发布上门电机状态，消息类型为`std_msgs/Bool`，`True`表示已使能，`False`表示未使能

`upper_up_limit_state` 话题发布上门上限位状态，消息类型为`std_msgs/Bool`，`True`表示上限位触发，`False`表示未触发

`uppper_down_limit_state` 话题发布上门下限位状态，消息类型为`std_msgs/Bool`，`True`表示下限位触发，`False`表示未触发

`lower_motor_state` 话题发布下门电机状态，消息类型为`std_msgs/Bool`，`True`表示已使能，`False`表示未使能

`lower_up_limit_state` 话题发布下门上限位状态，消息类型为`std_msgs/Bool`，`True`表示上限位触发，`False`表示未触发

`lower_down_limit_state` 话题发布下门下限位状态，消息类型为`std_msgs/Bool`，`True`表示下限位触发，`False`表示未触发

订阅话题：

`upper_motor_state_cmd` 话题订阅上门电机控制指令，消息类型为`std_msgs/Bool`，接收控制指令以控制上门电机的使能状态

`upper_reset_cmd` 话题订阅上舱门复位或归零指令，消息类型为`std_msgs/String`，格式为`up`或`up,距离`，单位为毫米

`upper_control_cmd` 话题订阅上门移动指令，消息类型为`std_msgs/String`，格式为`up,距离`，单位为毫米

`lower_motor_state_cmd` 话题订阅下门电机控制指令，消息类型为`std_msgs/Bool`，接收控制指令以控制下门电机的使能状态

`lower_reset_cmd` 话题订阅下舱门复位或归零指令，消息类型为`std_msgs/String`，格式为`down`或`down,距离`，单位为毫米

`lower_control_cmd` 话题订阅下门移动指令，消息类型为`std_msgs/String`，格式为`down,距离`，单位为毫米

`deliver_init_cmd` 话题订阅送物上装初始化指令，消息类型为`std_msgs/String`，接收控制指令以初始化送物上装系统，指令内容为`init`或`上距离，下距离`，单位为毫米

`motor_reset_cmd` 话题订阅电机复位指令，消息类型为`std_msgs/Bool`


