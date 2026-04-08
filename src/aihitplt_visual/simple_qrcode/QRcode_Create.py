#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
import qrcode
from PIL import Image
from pathlib import Path
import matplotlib.pyplot as plt

def add_logo(img, logo_path):
    """将 logo 居中贴到二维码中心"""
    icon = Image.open(logo_path)
    img_w, img_h = img.size
    factor = 6
    size_w, size_h = int(img_w / factor), int(img_h / factor)
    icon_w, icon_h = icon.size
    if icon_w > size_w: icon_w = size_w
    if icon_h > size_h: icon_h = size_h
    # 关键修复：PIL 10+ 用 LANCZOS
    icon = icon.resize((icon_w, icon_h), Image.LANCZOS)
    w, h = (img_w - icon_w) // 2, (img_h - icon_h) // 2
    img.paste(icon, (w, h), mask=None)
    return img

def Create_QRcode(data, file_name, logo_path):
    """生成二维码（带高容错）并可插入 logo"""
    my_file = Path(logo_path)
    qr = qrcode.QRCode(
        version=1,
        error_correction=qrcode.constants.ERROR_CORRECT_H,
        box_size=5,
        border=4,
    )
    qr.add_data(data)
    qr.make(fit=True)
    img = qr.make_image(fill_color="green", back_color="white")
    if my_file.is_file():
        img = add_logo(img, logo_path)
    img.save(file_name)
    plt.imshow(img)
    plt.axis('off')
    plt.show()
    return img

if __name__ == '__main__':
    file_path = os.getcwd()
    logo_path = file_path + "/aihit.jpg"
    out_img = file_path + '/myQRcode.jpg'
    text = input("Please enter: ")
    Create_QRcode(text, out_img, logo_path)