#!/usr/bin/env python3

from ultralytics import YOLO

# 加载模型
model = YOLO('/home/aihit/aihitplt_ws/src/aihitplt_yolo/param/fire_detect.pt')

# 对单张图像进行预测
results = model.predict('/home/aihit/aihitplt_test/yolov5/img/Test_fire/thumb (7).jpg', conf=0.1) # conf为置信度

# 可视化结果
results[0].show()
