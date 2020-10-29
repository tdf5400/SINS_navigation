import numpy as np


def calCRC(data):# data:ADDR~Datas
    assert isinstance(data, list), "Data should be list!"
    poly = 0x8408
    crc = int(0)

    for i in range(2, len(data)):
        crc = crc ^ data[i]
        for j in range(8):
            carry = int(crc) & 1
            crc = int(crc / 2)
            if carry:
                crc = crc ^ poly

    return crc




