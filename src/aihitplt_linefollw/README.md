# 功能包使用说明

## 1. 功能包说明

`aihitplt_linefollw` 功能包主要用于AGV的路径跟随功能测试，包含路径跟随算法的实现和测试。

## 2. 功能包使用

`roslaunch aihitplt_linefollw follow_line.launch` # 启动路径跟随功能

`rosrun rqt_reconfigure rqt_reconfigure` # 运行rqt_reconfigure工具，可以动态调整路径跟随算法的参数，以优化AGV的路径跟随性能

