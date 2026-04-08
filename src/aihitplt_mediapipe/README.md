# 功能包使用说明

## 1. 功能包说明

`aihitplt_mediapipe` 功能包主要用于启动MediaPipe相关功能，例如MediaPipe手势识别、MediaPipe姿态估计、MediaPipe面部识别等功能。

## 2. 功能包使用

功能包中有多个功能，并且分为USB摄像头与深度相机两种类型，可以根据需要选择启动

USB摄像头相关功能：

`roslaunch aihitplt_mediapipe 01_HandDetector.launch` # 启动MediaPipe手部检测功能

`roslaunch aihtplt_mediapipe 02_PoseDetector.launch` # 启动MediaPipe姿态估计功能

`roslaunch aihitplt_mediapipe 03_Holistic.launch` # 启动MediaPipe全身识别功能

`roslaunch aihitplt_mediapipe 04_FaceMesh.launch` # 启动MediaPipe面部识别功能

`roslaunch aihitplt_mediapipe 05_FaceEyeDetection.launch` # 启动MediaPipe面部眼部识别功能

`roslaunch aihitplt_mediapipe 06_FaceLandmarks.launch` # 启动MediaPipe面部特征点识别功能

`roslaunch aihitplt_mediapipe 07_FaceDetection.launch` # 启动MediaPipe面部检测功能

`roslaunch aihitplt_mediapipe 08_Objectron.launch` # 启动MediaPipe物体识别功能

`roslaunch aihitplt_mediapipe 09_VirtualPaint.launch` # 启动MediaPipe虚拟绘画功能

`roslaunch aihitplt_mediapipe 10_HandCtrl.launch` # 启动MediaPipe手势控制功能

`roslaunch aihitplt_mediapipe 11_GestureRecognition.launch` # 启动MediaPipe手势识别功能

### launch文件说明

launch文件中包含了图像输入话题以及图像输出话题的设置，可以根据需要进行修改，以手部检测为例：
```
<launch>
    <arg name="input_image_topic" default="/usb_cam/image_raw"/>

    <include file="$(find usb_cam)/launch/usb_cam_no_display.launch"/>

    <node pkg="aihitplt_mediapipe" type="01_HandDetector.py" name="handDetector" required="true" output="screen">
        <!-- 重映射输入话题 -->
        <remap from="/camera/rgb/image_raw" to="$(arg input_image_topic)"/>
    </node>
 
</launch>
```
其中`input_image_topic`参数用于指定输入图像的话题名称，默认为`/usb_cam/image_raw`，可以根据实际情况修改为正确的话题名称。

`<remap from="/camera/rgb/image_raw" to="$(arg input_image_topic)"/>`这行代码用于将节点内部使用的图像输入话题`/camera/rgb/image_raw`重映射为`input_image_topic`参数指定的话题名称，以确保节点能够正确接收图像数据。

使用时需要确保自身深度相机或USB摄像头的图像话题名称与launch文件中设置的输入话题名称一致，或者通过修改launch文件中的参数来匹配实际使用的图像话题名称。也就是说需要将自己的相机话题正确映射到`input_image_topic`参数上，以确保节点能够正确接收图像数据进行处理。

深度相机相关功能：

`roslaunch aihitplt_mediapipe 01_HandDetector_deepcam.launch` # 启动MediaPipe手部检测功能（深度相机）

`roslaunch aihtplt_mediapipe 02_PoseDetector_deepcam.launch` # 启动MediaPipe姿态估计功能（深度相机）

`roslaunch aihitplt_mediapipe 03_Holistic_deepcam.launch` # 启动MediaPipe全身识别功能（深度相机）

`roslaunch aihitplt_mediapipe 04_FaceMesh_deepcam.launch` # 启动MediaPipe面部识别功能（深度相机）

`roslaunch aihitplt_mediapipe 05_FaceEyeDetection_deepcam.launch` # 启动MediaPipe面部眼部识别功能（深度相机）

`roslaunch aihitplt_mediapipe 06_FaceLandmarks_deepcam.launch` # 启动MediaPipe面部特征点识别功能（深度相机）

`roslaunch aihitplt_mediapipe 07_FaceDetection_deepcam.launch` # 启动MediaPipe面部检测功能（深度相机）

`roslaunch aihitplt_mediapipe 08_Objectron_deepcam.launch` # 启动MediaPipe物体识别功能（深度相机）

`roslaunch aihitplt_mediapipe 09_VirtualPaint_deepcam.launch` # 启动MediaPipe虚拟绘画功能（深度相机）

`roslaunch aihitplt_mediapipe 10_HandCtrl_deepcam.launch` # 启动MediaPipe手势控制功能（深度相机）

`roslaunch aihitplt_mediapipe 11_GestureRecognition_deepcam.launch` # 启动MediaPipe手势识别功能（深度相机）

