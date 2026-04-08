# 功能包使用说明

## 1. 功能包说明

`aihitplt_multimodal_ai` 功能包主要用于进行AI开发套件功能使用与测试。

## 2. 功能包使用

功能包中有多个功能模块，可以根据需要选择启动相应的功能。

`roslaunch aihitplt_multimodal_ai aihitplt_ai_cam.launch` # 启动AI开发套件工业相机

### launch文件说明：

```
<launch>
  <node name="usb_cam" pkg="usb_cam" type="usb_cam_node" output="screen" >
    <param name="video_device" value="/dev/video2" />
    <param name="image_width" value="640" />
    <param name="image_height" value="480" />
    <param name="pixel_format" value="yuyv" />
    <param name="camera_frame_id" value="usb_cam" />
    <param name="io_method" value="mmap"/>
  </node>
</launch>
```
其中`video_device`参数用于指定相机设备的路径，默认为`/dev/video2`，可以根据实际情况修改为正确的设备路径。

`roslaunch aihitplt_multimodal_ai aihitplt_ai_deepcam.launch` # 启动AI开发套件深度相机

`roslaunch aihitplt_multimodal_ai aihitplt_pan_sensor.launch` # 启动AI开发套件环境传感器

### launch文件说明：

```
<launch>
  <node pkg="aihitplt_multimodal_ai" type="aihitplt_pan_tilt_sensor.py" name="pan_tilt_sensor_node" output="screen">
        <param name="port" value="/dev/ttyUSB0"/>
  </node>
</launch>
```
其中`port`参数用于指定环境传感器的串口设备路径，默认为`/dev/ttyUSB0`，可以根据实际情况修改为正确的设备路径。

`roslaunch aihitplt_multimodal_ai multi_camera.launch` # 启动AI开发套件与底盘双深度相机








