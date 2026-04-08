#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import os
import time
import numpy as np
import cv2 as cv
import pyzbar.pyzbar as pyzbar
from cv_bridge import CvBridge
from sensor_msgs.msg import Image as ROSImage
from PIL import Image, ImageDraw, ImageFont

def decodeDisplay(image, font_path):
    gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
    # 需要先把输出的中文字符转换成Unicode编码形式
    # The output Chinese characters need to be converted to Unicode encoding first
    barcodes = pyzbar.decode(gray)
    for barcode in barcodes:
        # 提取二维码的边界框的位置
        # Extract the position of the boundary box of the TWO-DIMENSIONAL code
        # 画出图像中条形码的边界框
        # Draw the bounding box for the bar code in the image
        (x, y, w, h) = barcode.rect
        cv.rectangle(image, (x, y), (x + w, y + h), (225, 0, 0), 5)
        encoding = 'UTF-8'
        # 画出来，就需要先将它转换成字符串
        # to draw it, you need to convert it to a string
        barcodeData = barcode.data.decode(encoding)
        barcodeType = barcode.type
        # 绘出图像上数据和类型
        # Draw the data and type on the image
        pilimg = Image.fromarray(image)
        # 创建画笔
        # create brush
        draw = ImageDraw.Draw(pilimg)  # 图片上打印  Print on picture
        # 参数1：字体文件路径，参数2：字体大小
        # parameter 1: font file path, parameter 2: font size
        fontStyle = ImageFont.truetype(font_path, size=12, encoding=encoding)
        # # 参数1：打印坐标，参数2：文本，参数3：字体颜色，参数4：字体
        # Parameter 1: print coordinates, parameter 2: text, parameter 3: font color, parameter 4: font
        draw.text((x, y - 25), barcodeData, fill=(255, 0, 0), font=fontStyle)
        # # PIL图片转cv2 图片
        # PIL picture to CV2 picture
        image = cv.cvtColor(np.array(pilimg), cv.COLOR_RGB2BGR)
        # 向终端打印条形码数据和条形码类型
        # Print barcode data and barcode type to terminal
        print(u"[INFO] Found {} barcode: {}".format(barcodeType, barcodeData))
    return image

def topic(msg):
    if not isinstance(msg, ROSImage):
        return
    bridge = CvBridge()
    frame = bridge.imgmsg_to_cv2(msg, "bgr8")
    # Standardize the input image size
    frame = cv.resize(frame, (640, 480))
    start = time.time()
    font_path = "~/car_ws/src/car_astra/scripts/simple_qrcode/font/Block_Simplified.TTF"
    font_path = os.path.expanduser(font_path)
    frame = decodeDisplay(frame, font_path)
    end = time.time()
    fps = 1 / (end - start)
    text = "FPS : " + str(int(fps))
    cv.putText(frame, text, (30, 30), cv.FONT_HERSHEY_SIMPLEX, 0.6, (100, 200, 200), 1)
    cv.imshow('frame', frame)
    cv.waitKey(10)


if __name__ == '__main__':
    rospy.init_node("astra_rgb_image_py")
    sub = rospy.Subscriber("/camera/rgb/image_raw", ROSImage, topic)
    rate = rospy.Rate(2)
    rospy.spin()
