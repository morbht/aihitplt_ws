# 功能包使用说明

## 1. 功能包说明

`aihitplt_spray` 功能包主要用于启动AIHIT喷雾相关节点。

## 2. 功能包使用

`roslaunch aihitplt_spray aihitplt_spray.launch` # 启动AIHIT喷雾相关节点

### launch文件说明

```
<launch>
    <!-- 参数配置 -->
    <arg name="port" default="/dev/ttyUSB2" />
    <arg name="baudrate" default="115200" />
    
    <!-- 串口桥接节点 -->
    <node name="aihitplt_spray_bridge" pkg="aihitplt_spray" type="aihitplt_spray_bridge.py" output="screen">
        <param name="port" value="$(arg port)" />
        <param name="baudrate" value="$(arg baudrate)" />
    </node>
    
    <!-- 手动控制节点 -->
    <node name="spray_controller" pkg="aihitplt_spray" type="spray_controller.py" output="screen" />
    
    <!-- 状态监控节点 -->
    <node name="spray_status_monitor" pkg="rostopic" type="rostopic" args="echo /spray_status" output="screen" />
    
</launch>
```
其中，`port`参数指定了串口桥接节点使用的串口端口，默认为`/dev/ttyUSB2`，可以根据实际连接的串口设备进行修改。`baudrate`参数指定了串口通信的波特率，默认为115200，可以根据实际设备的要求进行调整。
