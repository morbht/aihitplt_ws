# 功能包使用说明

## 1. 功能包说明

`aihitplt_nav` 功能包主要用于启动AIHIT导航相关功能，例如路径规划、导航控制等功能。

## 2. 功能包使用

`roslaunch aihitplt_nav astrapro_bringup.launch` # 启动深度相机与AGV底盘

`roslaunch aihitplt_nav laser_astrapro_bringup.launch` # 启动激光雷达、深度相机与AGV底盘

`roslaunch aihitplt_nav laser_bringup.launch` # 启动激光雷达AGV底盘

`roslaunch aihitplt_nav aihitplt_map.launch map_type` # 启动地图构建功能,其中 `<map_type>` 为建图算法，默认为gmapping，可选项包括：gmapping、hector、cartographer、karto

`roslaunch aihitplt_nav aihitplt_navigation.launch` # 启动导航功能

`roslaunch aihitplt_nav aihitplt_rtabmap.launch` # 启动rtabmap建图功能

`roslaunch aihitplt_nav aihitplt_rtabmap_nav.launch` # 启动rtabmap导航功能

## 3. 功能包结构

`bringup` 文件夹：包含启动深度相机、激光雷达和AGV底盘的launch文件

`combo` 文件夹：包含建图导航功能的launch文件，例如视觉建图launch文件

`configuration_files` 文件夹：包含Cartographer建图算法的配置文件

`library` 文件夹：包含路径规划和导航控制相关的launch文件

`rtabmap` 文件夹：包含rtabmap建图和导航相关的launch文件

`viewer` 文件夹：包含RViz可视化启动与配置文件，用于导航功能的可视化展示


