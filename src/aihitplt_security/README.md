# 功能包使用说明

## 1. 功能包说明

`aihitplt_security` 功能包主要包含安防模块上装的启动程序。

## 2. 功能包使用

`roslaunch aihitplt_security aihitplt_pan_tilt_camera.launch` # 启动安防模块的云台相机

`roslaunch aihitplt_security aihitplt_security_sensors.launch` # 启动安防模块传感器节点

### launch文件说明：

`aihitplt_security_sensors.launch` 文件中包含了安防模块的传感器节点串口配置：

```
    <node name="security_sensors_node" pkg="aihitplt_security" type="security_sensors_node.py">
        <param name="port" value="/dev/ttyUSB2"/>
    </node>
```
其中`port`参数指定了传感器节点使用的串口端口，默认为`/dev/ttyUSB2`，可以根据实际连接的串口设备进行修改。
