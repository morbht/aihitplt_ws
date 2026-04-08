# 功能包使用说明

## 1. 功能包简介

`aihitplt_guide` 功能包主要用于迎宾与送餐上装测试，包含急停功能测试

## 2. 功能包使用

`roslaunch aihitplt_guide guide_deli_estop.launch` # 启动迎宾与送餐上装测试

## 3. guide_deli_estop.py程序说明

`port = rospy.get_param('~serial_port', '/dev/ttyUSB2')` # 获取串口参数，默认为`/dev/ttyUSB2`,可以通过修改参数来指定其他串口

## 4. 话题说明

发布话题：

`/guide_delivery_emergency_button` 话题发布急停按钮状态，话题类型为`std_msgs/Bool`，`True`表示急停状态，`False`表示非急停状态
