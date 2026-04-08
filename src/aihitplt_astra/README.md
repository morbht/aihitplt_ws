# 功能包使用说明

## 1. 功能包说明

`aihitplt_astra`功能包内含深度相机与AGV底盘视觉相关功能launch文件及python程序，可启动AGV底盘视觉或Astra深度相机功能，例如颜色追踪、目标检测、AR功能等。

## 2. 使用说明

`roslaunch aihitplt_astra astra_camera.launch` #使用该命令启动Astra深度相机

`roslaunch aihitplt_astra colorTracker.launch` #启动AGV颜色追踪功能

`roslaunch aihitplt_astra colorTracker_astra.launch` #启动Astra深度相机颜色追踪功能

`roslaunch aihitplt_astra KCFTracker.launch` #启动AGV KCF目标追踪功能

`roslaunch aihitplt_astra KCFTracker_astra.launch` #启动Astra深度相机KCF目标追踪功能
