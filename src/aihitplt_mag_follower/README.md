# 功能包使用说明

## 1. 功能包说明

`aihitplt_mag_follower` 功能包主要用于AGV的磁条跟随功能测试，包含磁条跟随算法的实现和测试。

## 2. 功能包使用

`roslaunch aihitplt_mag_follower aihitplt_mag_follower.launch` # 启动磁条跟随功能

### aihitplt_mag_follower.launch 文件解析：

`<param name="Port" type="string" value="/dev/aihitplt_mag"/>` #设置磁条传感器的串口端口

`<param name="BaudRate" type="int" value="115200"/>` #设置串口通信的波特率

`<param name="ID" type="int" value="1"/>` #传感器ID

`<param name="Mode" type="int" value="0"/>` #消息模式，0表示发布标准`data`类型，1表示发布网络`netdata`类型




