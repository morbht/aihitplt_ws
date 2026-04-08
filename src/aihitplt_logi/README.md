# 功能包使用说明

## 1. 功能包说明

`aihitplt_logi` 功能包主要用于工业物流上装相关节点启动

## 2. 功能包使用

`roslaunch aihitplt_logi logic_scale.launch` # 启动工业物流上装相关节点

## 3. logic_scale_node.py程序说明

`self.port = rospy.get_param('~serial_port', '/dev/ttyUSB2')` # 获取串口参数，默认为`/dev/ttyUSB2`,可以通过修改参数来指定其他串口

## 4. 话题说明

发布话题：

`logi_scale/weight` 话题发布当前重量数据，话题类型为`std_msgs/Float32`

`/logi_scale/raw_data` 话题发布原始数据，话题类型为`std_msgs/String`,可选发布（通过`~publish_raw_data`参数控制）
发布串口接收到的原始数据字符串

`/logi_scale/calibration_factor` 话题发布校准因子，用于重量校准，话题类型为`std_msgs/Float32`

`/logi_scale/emergency_stop` 话题发布紧急停止信号，话题类型为`std_msgs/Bool`

`/logi_scale/device_state` 话题发布设备状态，话题类型为`std_msgs/String`

``
状态说明：
```
0 - 正常
1 - 归零中
2 - 校准中
3 - 初始化中
4 - 通信异常

```

订阅话题：

`/logi_scale/control` 话题订阅控制指令，话题类型为`std_msgs/String`，接收控制指令，发送给传感器设备
