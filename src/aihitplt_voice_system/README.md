# 功能包使用说明

## 1. 功能包说明

`aihitplt_voice_system` 功能包主要用于启动AIHIT语音系统，提供语音识别、语音合成等功能，方便用户与AIHIT进行语音交互。

## 2. 功能包使用

`roslaunch aihitplt_voice_system aihitplt_voice_off_line.launch` # 启动AIHIT离线语音系统

`roslaunch aihitplt_voice_system aihitplt_aisound.launch` # 启动AIHIT离线语音合成系统

### launch文件说明

```
<launch>
  <node pkg="aihitplt_voice_system" type="aihitplt_aisound.py" name="aihitplt_tts_node" output="screen">
    <param name="appid" value="0daad079" />
    <param name="voice" value="测试测试"/>
    <param name="work_dir" value="/home/aihit/aihitplt_ws/src/aihitplt_voice_system/bin/msc" />>
    <param name="speech_param" value="voice_name = xiaoyan, text_encoding = utf8, sample_rate = 16000, speed = 50, volume = 50, pitch = 50, rdn = 2" />
  </node>
</launch>
```
如果需要使用离线语音合成功能，可以通过调整参数`voice`来指定需要合成的文本内容。`work_dir`参数指定了离线语音合成所需的资源文件所在的目录路径。`appid`参数指定了离线语音合成所需的AppID。
