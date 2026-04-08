#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import time
import cv2 as cv
import numpy as np
import pyzbar.pyzbar as pyzbar
from PIL import Image, ImageDraw, ImageFont
import rospy
from sensor_msgs.msg import Image as ROSImage
from cv_bridge import CvBridge, CvBridgeError

class QRCodeDetector:
    def __init__(self):
        # 从参数服务器获取参数
        self.flip = rospy.get_param('~flip', False)
        self.display = rospy.get_param('~display', True)
        input_image_topic = rospy.get_param('~input_image_topic', '/usb_cam/image_raw')
        
        self.font_path = rospy.get_param('~font_path', "../font/Block_Simplified.TTF")
        self.bridge = CvBridge()
        self.cv_image = None
        
        # 订阅图像话题
        self.image_sub = rospy.Subscriber(input_image_topic, ROSImage, self.image_callback)
        
        # 发布处理后的图像（可选）
        self.output_image_pub = rospy.Publisher('/qrcode_detector/output_image', ROSImage, queue_size=1)
        
        rospy.loginfo("QRCode Detector initialized with:")
        rospy.loginfo("  Input topic: %s", input_image_topic)
        rospy.loginfo("  Flip: %s", self.flip)
        rospy.loginfo("  Display: %s", self.display)
        
    def image_callback(self, data):
        try:
            # 将ROS图像消息转换为OpenCV图像
            self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            
            # 如果需要翻转图像
            if self.flip:
                self.cv_image = cv.flip(self.cv_image, 1)  # 水平翻转
            
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))

    def decodeDisplay(self, image):
        if image is None:
            return None
            
        gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
        barcodes = pyzbar.decode(gray)
        
        found_codes = []
        
        for barcode in barcodes:
            # 提取二维码的边界框的位置
            (x, y, w, h) = barcode.rect
            cv.rectangle(image, (x, y), (x + w, y + h), (225, 0, 0), 5)
            
            encoding = 'UTF-8'
            barcodeData = barcode.data.decode(encoding)
            barcodeType = barcode.type
            
            # 使用PIL绘制中文文本
            pilimg = Image.fromarray(image)
            draw = ImageDraw.Draw(pilimg)
            
            try:
                fontStyle = ImageFont.truetype(self.font_path, size=12, encoding=encoding)
                draw.text((x, y - 25), str(barcode.data, encoding), fill=(255, 0, 0), font=fontStyle)
            except:
                # 如果字体加载失败，使用默认字体
                cv.putText(image, barcodeData, (x, y - 10), cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
            
            image = cv.cvtColor(np.array(pilimg), cv.COLOR_RGB2BGR)
            
            # 记录检测到的二维码信息
            found_codes.append({"type": barcodeType, "data": barcodeData})
            rospy.loginfo("Found {} barcode: {}".format(barcodeType, barcodeData))
        
        # 发布检测结果（可选）
        if found_codes:
            rospy.loginfo("Detected {} QR/barcode(s)".format(len(found_codes)))
            
        return image, found_codes

    def run(self):
        rate = rospy.Rate(30)  # 30Hz
        
        while not rospy.is_shutdown():
            if self.cv_image is not None:
                start = time.time()
                
                # 处理图像并检测二维码
                processed_image, detected_codes = self.decodeDisplay(self.cv_image.copy())
                
                if processed_image is not None:
                    end = time.time()
                    fps = 1 / (end - start)
                    
                    # 添加FPS信息
                    text = "FPS : " + str(int(fps))
                    cv.putText(processed_image, text, (30, 30), cv.FONT_HERSHEY_SIMPLEX, 0.6, (100, 200, 200), 1)
                    
                    # 添加检测到的二维码数量信息
                    codes_text = "Codes: " + str(len(detected_codes))
                    cv.putText(processed_image, codes_text, (30, 60), cv.FONT_HERSHEY_SIMPLEX, 0.6, (100, 200, 200), 1)
                    
                    # 发布处理后的图像
                    try:
                        output_msg = self.bridge.cv2_to_imgmsg(processed_image, "bgr8")
                        self.output_image_pub.publish(output_msg)
                    except CvBridgeError as e:
                        rospy.logerr("CvBridge Error: {0}".format(e))
                    
                    # 显示图像（如果启用显示）
                    if self.display:
                        cv.imshow('QR Code Detector', processed_image)
                
                # 检查退出键
                if self.display:
                    action = cv.waitKey(1) & 0xFF
                    if action == ord('q') or action == 113: 
                        break
                    
            rate.sleep()
            
        if self.display:
            cv.destroyAllWindows()

if __name__ == '__main__':
    try:
        rospy.init_node('qrcode_detector', anonymous=True)
        detector = QRCodeDetector()
        detector.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr("Error: {0}".format(e))