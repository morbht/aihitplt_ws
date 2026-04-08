# 功能包使用说明

## 1. 功能包说明

`aihitplt_yolo` 功能包主要用于启动YOLO相关功能，例如使用云台相机进行YOLO检测。

## 2. 功能包使用

`roslaunch aihitplt_yolo pan_cam_detect.launch` # 启动云台相机以及YOLO检测功能

`roslaunch aihitplt_yolo pan_cam_detect_gui.launch` # 启动云台相机以及YOLO检测功能，并打开GUI界面显示检测画面与结果

`roslaunch aihitplt_yolo yolodetect_deepcam.launch` # 启动深度相机以及YOLO检测功能，使用模型为yolov4-tiny

`roslaunch aihitplt_yolo yolodetect_pan_cam.launch` # 启动云台相机以及YOLO检测功能，使用模型为yolov4-tiny


