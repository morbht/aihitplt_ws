# 功能包使用说明

## 1. 功能包说明

`aihitplt_visual` 功能包主要用于AIHIT视觉相关功能的实现和测试，例如PCL点云处理、视觉识别等功能。

## 2. 功能包使用

`roslaunch aihitplt_visual laser_to_image.launch` # 启动激光雷达点云转图像功能

`roslaunch aihitplt_visual astra_calibration.launch` # 启动Astra相机

`rosrun aihitplt_visual astra_rgb_image.launch` # 获取Astra相机彩色图像

`rosrun aihitplt_visual astra_depth_image.launch` # 获取Astra相机深度图像

`rosrun aihitplt_visual astra_image_flip.launch` # 获取Astra相机翻转图像

`roslaunch aihitplt_visual opencv_apps.launch` # 启动2D相机及转换节点

`roslaunch aihitplt_visual opencv_apps_deepcam.launch` # 启动深度相机及转换节点

`roslaunch aihitplt_visual simple_AR.launch display:=true flip:=false` # 启动AR视觉功能，display参数控制是否显示识别结果，flip参数控制是否翻转图像进行识别

`roslaunch aihitplt_visual ar_track.launch open_rviz:=true` # 启动二维码跟踪功能，open_rviz参数控制是否打开RViz进行可视化显示

`roslaunch aihitplt_visual pointCloud_visualize.launch cloud_topic:=/camera/depth_registered/points` # 启动点云可视化功能，cloud_topic参数指定点云数据的话题名称。

`roslaunch aihitplt_visual pointCloud_pub.launch use_rviz:=true` # 启动点云发布功能，use_rviz参数控制是否在RViz中可视化点云数据

