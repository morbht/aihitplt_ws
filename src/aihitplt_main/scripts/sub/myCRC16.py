
def crc16_calculate(string):
    data = bytearray.fromhex(string)
    crc = 0xFFFF
    for pos in data:
        crc ^= pos
        for i in range(8):
            if ((crc & 1) != 0):
                crc >>= 1
                crc ^= 0xA001
            else:
                crc >>= 1

    return crc

    #返回高低位互换的crc
    # return hex(((crc & 0xff) << 8) + (crc >> 8))
    # return ((crc & 0xff) << 8) + (crc >> 8)



crc = crc16_calculate('010300060001')
