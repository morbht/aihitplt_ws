#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import time, cv2 as cv, numpy as np, pyzbar.pyzbar as pyzbar
from PIL import Image, ImageDraw, ImageFont
import rospy
from sensor_msgs.msg import Image as ROSImage
from cv_bridge import CvBridge, CvBridgeError

class QRCodeDetector:
    def __init__(self):
        # 默认话题改为 Astra Pro RGB
        self.flip       = rospy.get_param('~flip', False)
        self.display    = rospy.get_param('~display', True)
        input_topic     = rospy.get_param('~input_image_topic',
                                          '/camera/rgb/image_raw')  # <-- 关键改动
        self.font_path  = rospy.get_param('~font_path',
                                          '../font/Block_Simplified.TTF')
        self.bridge     = CvBridge()
        self.cv_image   = None

        self.image_sub  = rospy.Subscriber(input_topic, ROSImage,
                                           self.image_callback)
        self.output_pub = rospy.Publisher('~output_image', ROSImage,
                                           queue_size=1)

        rospy.loginfo("QRCode Detector (Astra Pro) started")
        rospy.loginfo("  Input topic: %s", input_topic)

    def image_callback(self, data):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            if self.flip:
                self.cv_image = cv.flip(self.cv_image, 1)  # 水平镜像
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: %s", e)

    def decodeDisplay(self, image):
        if image is None:
            return None, []
        gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
        barcodes = pyzbar.decode(gray)
        found = []

        for bc in barcodes:
            x, y, w, h = bc.rect
            cv.rectangle(image, (x, y), (x+w, y+h), (0, 0, 255), 3)

            data = bc.data.decode('utf-8')
            found.append({'type': bc.type, 'data': data})

            # 中文绘制
            pil_img = Image.fromarray(cv.cvtColor(image, cv.COLOR_BGR2RGB))
            draw = ImageDraw.Draw(pil_img)
            try:
                font = ImageFont.truetype(self.font_path, 20, encoding='utf-8')
                draw.text((x, y-25), data, fill=(255, 0, 0), font=font)
            except:
                cv.putText(image, data, (x, y-10),
                           cv.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
            image = cv.cvtColor(np.array(pil_img), cv.COLOR_RGB2BGR)

            rospy.loginfo("Found %s: %s", bc.type, data)
        return image, found

    def run(self):
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            if self.cv_image is not None:
                t0 = time.time()
                out_img, codes = self.decodeDisplay(self.cv_image.copy())
                fps = 1/(time.time()-t0)

                cv.putText(out_img, f"FPS:{int(fps)}", (10, 30),
                           cv.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                cv.putText(out_img, f"Codes:{len(codes)}", (10, 60),
                           cv.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

                try:
                    self.output_pub.publish(
                        self.bridge.cv2_to_imgmsg(out_img, "bgr8"))
                except CvBridgeError as e:
                    rospy.logerr("pub error: %s", e)

                if self.display:
                    cv.imshow("QR Astra Pro", out_img)
                    if cv.waitKey(1) & 0xFF == ord('q'):
                        break
            rate.sleep()

        if self.display:
            cv.destroyAllWindows()

if __name__ == '__main__':
    rospy.init_node('qrcode_detector_astra', anonymous=True)
    try:
        QRCodeDetector().run()
    except rospy.ROSInterruptException:
        pass