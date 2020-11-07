import numpy as np
import serial
import struct
import time


class ERR:
    """
    Error code
    """
    UNFORMAT = 0  # 数据格式错误
    CRC = 1  # CRC校检错误


class MEMS:
    """
    device control
    """
    __comPort = None    # object of comPort
    __outFre = 0        # the frequency of output mode
    __outMask = None    # output mask

    def __init__(self, comPort, baud):
        assert isinstance(comPort, str), "comPort should be string!"
        self.__comPort = serial.Serial(comPort, baud, timeout=0.5)
        # update the state of sensor
        self.__outFre = self.getContinueMode()
        self.__outMask = self.getOutputMask()

    def __u32ToFloat(self, u32List):
        """
        convert u32 to float
        :param u32List:
        :return:
        """
        assert len(u32List) == 4, "The length of u32List should be 4 and not {}!".format(len(u32List))
        byteData = bytearray()
        # convert data to bytearray
        for i in u32List:
            byteData.append(i)
        # print(byteData)
        output = struct.unpack("!f", byteData)[0]
        return output

    def __calCRC(self, data):   # crcData:ADDR~Datas
        assert isinstance(data, list), "crcData should be list!"
        poly = 0x8408
        crc = int(0)

        for i in range(2, len(data)):
            crc = crc ^ data[i]
            for j in range(8):
                carry = int(crc) & 1
                crc = int(crc / 2)
                if carry:
                    crc = crc ^ poly
        # print('return: {}'.format('%#x' % crc))
        return crc

    def __comSent(self, data):
        self.__comPort.write(data)

    def __comRead(self):
        """
        :return: dataLen - length,
                 data    - data from sensor
        """
        # comdata = self.__comPort.read_until(0x03)
        comdata = ""
        errTimes = 0
        while errTimes<5 and comdata[-1] != 0x03:
            comdata = comdata + self.__comPort.read_all()
            errTimes += 1
        # comdata = [0xff, 0x02, 0x00, 0x90, 0x00, 0x24,
        #            0xbb, 0xff, 0x0f, 0x9e,
        #            0x3c, 0x35, 0xe1, 0x1f,
        #            0xbd, 0x2e, 0x03, 0xf5,
        #            0xb9, 0x9d, 0x48, 0x48,
        #            0x00, 0x00, 0x00, 0x00,
        #            0x00, 0x00, 0x00, 0x00,
        #            0x00, 0x00, 0x00, 0x00,
        #            0x00, 0x00, 0x00, 0x00,
        #            0x00, 0x00, 0x00, 0x00,
        #            0x00, 0x00, 0x03]
        data = []
        for i in range((len(comdata) - 6), -1, -1):
            if comdata[i] == 0xff and comdata[i + 1] == 0x02:
                data = comdata[i:-1]
        if len(data) == 0 or len(comdata) == 0:  # no 0xff or 0x03
            return ERR.UNFORMAT
        # parse length
        dataLen = data[4] << 8 | data[5]
        # print(f'len:{dataLen}')

        return dataLen, data[6:6+dataLen]

    def __setCmd(self, cmd, length, data):
        assert isinstance(data, list), "The type of data should be List!"
        comData = [0xff, 0x02, 0x00, cmd, (length&0xff00), (length&0xff)]
        comData = comData + data
        # crc calculate
        crc = self.__calCRC(comData)
        comData.append((crc & 0xff00) >> 8)
        comData.append(crc & 0xff)
        comData.append(0x03)
        # print(comData)
        # sent data to sensor
        self.__comSent(comData)
        return comData

    def __enquireData(self, cmd, length, flush=True):
        # ask for data
        if flush:
            self.__comPort.flush()
        self.__setCmd(cmd, length, [])
        # read data
        errTimes = 0
        data = self.__comRead()
        while errTimes<5 and data is ERR.UNFORMAT:
            data = self.__comRead()
            errTimes += 1

        if errTimes >= 5:
            return False
        return data

    def getContinueData(self):
        """
        parse auto sent data
        :return: state information
        """
        # get data from comPort
        data = self.__comRead()[1]

        # print(data[5:9])
        roll = self.__u32ToFloat(data[0:4])
        pitch = self.__u32ToFloat(data[4:8])
        yaw = self.__u32ToFloat(data[8:12])
        # print(roll, pitch, yaw)
        return roll, pitch, yaw

    def getContinueMode(self):
        """
        get state of continue mode
        :return: fre - 0,enquire mode; others,output frequency
        """
        data = self.__enquireData(0x54, 0x00, [])[1]
        mode = data[0]
        if mode == 0:
            fre = 0
        else:
            fre = 200 / data[1]
        return fre

    def setContinueFre(self, frequency):
        """
        set the frequency of sensor
        :param frequency:0, 1, 5, 10, 20, 25, 50, 100, 200
        :return:
        """
        availFre = (200, 100, 50, 25, 20, 10, 5, 1)
        assert frequency in availFre, "frequency should be {}, not {}!".format(availFre, frequence)

        if frequency == 0:  # require mode
            mode = 0x00
        else:               # continue mode
            mode = 0x01
            div = 200 / frequency
        # sent command
        self.__setCmd(0x53, 0x03, [0, mode, div])

    def getOutputMask(self):
        """
        get the data class which sensor send
        :return: mask - 0x01, EULER;
                        0x02, GYROSCOPES;
                        0x04, ACCELEROMETERS
        """
        u32List = self.__enquireData(0x51, 0x00, [])[1]
        byteData = bytearray()
        # convert data to bytearray
        for i in u32List:
            byteData.append(i)
        # print(byteData)
        output = int.from_bytes(byteData)
        return output

    def saveAllSetting(self):
        """
        save all setting to the EEPROM of sensor
        :return:
        """
        data = self.__enquireData(0x24, 0x00, [])[1]
        return data


if __name__ == "__main__":
    ser = MEMS("COM4", 115200)
    # ser.getContinueData()
    # print(__u32ToFloat([0xbb, 0xff, 0x0f, 0x9e]))
    # data = ser.setCmd(0x53, 3, [1,1,1])
    ser.getOutputMask()
    # print(data)

