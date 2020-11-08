import struct


def u32ToFloat(byteData):
    """
    convert u32 to float
    :param byteData:
    :return:
    """
    byteData = byteData[::-1]   # inverted order
    output = struct.unpack("!f", byteData)[0]
    return output


def calCRC(data):   # crcData:ADDR~Datas
    # assert isinstance(data, list), "crcData should be list!"
    poly = 0x8408
    crc = int(0)

    for i in range(2, len(data)):
        crc = crc ^ data[i]
        for j in range(8):
            carry = int(crc) & 1
            crc = int(crc / 2)
            if carry:
                crc = crc ^ poly
    crc = crc.to_bytes(2, byteorder='big')
    # print('return: {}'.format('%#x' % crc))
    return crc


# print(u32ToFloat(b'\xca\x041\xbc'))
# print(u32ToFloat(b'\xbc1\x04\xca'))
#
# b'\xff\x02\x00\x90\x00\x0c\xca\x041\xbc\xa5\xe3\x93<GF\x94\xbe\xec\x80\x03'

# print(calCRC(b'\xff\x02\x00\x51\x00\x00'))

