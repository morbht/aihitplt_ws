Header header
bool online

uint16 io              # 开关量（16bit，8探头状态）
int16 offset_left      # 左偏移 (-7~+7，分辨率5mm)
int16 offset_straight  # 直行偏移
int16 offset_right     # 右偏移
bool has_magnet        # 0=无磁条，1=有磁条
bool all_on            # 0=未全检测，1=全部有效
