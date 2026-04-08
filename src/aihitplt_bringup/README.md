# 功能包使用说明

## 1. 功能包说明

`aihitplt_bringup`功能包，主要功能为机器人系统的启动和配置，包括机器人底盘、深度相机、激光雷达、磁导航、圆形屏等相关传感器的启动和参数配置，提供了一个统一的入口来启动整个机器人系统.

## 2. 使用说明

`roslaunch aihitplt_bringup aihitplt_bringup.launch` #使用该命令启动机器人系统，可通过调整参数启动多个传感器。

### aihitplt_bringup参数解析

`<arg name="smoother"  default="false"/>` #是否开启里程计平滑功能，默认为false，开启后可提升里程计精度，但可能增加系统延迟

`<arg name="odom_frame_id"  default="odom_combined"/>` #里程计坐标系ID，默认为odom_combined

`<arg name="Ultrasonic_Avoid" default="false"/>` #是否开启超声波避障功能，默认为false，开启后机器人将使用超声波传感器进行避障

`<arg name="is_cartographer" default="false"/>` #是否为cartographer建图，默认为false,开启后将调整激光雷达参数以适应cartographer建图需求

`<arg name="activate_pan_tilt_sensor" default="false"/>` #是否激活AI开发套件，默认为false，开启后将启动传感器相关节点

`<arg name="rviz_control" default="false" />` #是否启用RViz可视化控制界面，默认为false，开启后可在RViz中进行机器人控制和传感器数据监视

```
  <node pkg="aihitplt_bringup" type="aihitplt_robot_node" name="aihitplt_robot_node" output="screen" respawn="false">
    <param name="usart_port_name"    type="string" value="/dev/aihitplt_controller"/>  
    <param name="serial_baud_rate"   type="int"    value="115200"/>
    <param name="odom_frame_id"      type="string" value="$(arg odom_frame_id)"/> 
    <param name="robot_frame_id"     type="string" value="base_footprint"/> 
    <param name="gyro_frame_id"      type="string" value="gyro_link"/>
```

`usart_port_name`参数为机器人控制器串口设备路径，默认为`/dev/aihitplt_controller`，如有需要，则根据实际情况更改为正确的设备路径
