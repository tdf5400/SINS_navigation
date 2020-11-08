import serial
import time
import dataPack
import threading
from queue import Queue     # message queue


class ERR:
    """
    Error code
    """
    UNFORMAT = 0  # 数据格式错误
    CRC = 1  # CRC校检错误
    TIMEOUT = 2  # 超时


def ERROR_Func(errCode):
    if errCode == ERR.UNFORMAT:
        raise "Unformat data!"
    if errCode == ERR.CRC:
        raise "CRC Failed"
    if errCode == ERR.TIMEOUT:
        raise "timeout!"


class MEMS:
    """
    device control
    """
    __comPort = None     # object of comPort
    __comTimeout = 0.5   # timeout of comPort
    __outFre = 10        # the frequency of output mode
    __outMask = 0x01     # output mask
    __portLock = False   # lock of serial port
    messageQueue = []  # message of serial
    state = [0, 0, 0,  0, 0, 0,  0, 0, 0]  # roll,pitch,yaw,  gx,gy,gz, ax,ay,az

    def __init__(self, comPort, baud, messageQueLen=10, printInfo=False):
        assert isinstance(comPort, str), "comPort should be string!"
        # initialize the serial port
        self.__comPort = serial.Serial(comPort, baud, timeout=self.__comTimeout)
        # create message queue
        self.messageQueue = Queue(messageQueLen)   # the length of queue
        # create message thread
        messageThread = threading.Thread(target=self.__messageThread)
        messageThread.start()

        # update the state of sensor
        print('getOutputMask!')
        self.__outMask = self.getOutputMask()
        print('getContinueMode!')
        self.__outFre = self.getContinueMode()

        if printInfo:
            print("Fre:{}".format(self.__outFre))
            print("Mask:{}".format(self.__outMask))

    def __messageThread(self):
        """
        A thread used for message receiving
        :return:
        """
        while 1:
            if 1:#self.__portLock is False:
                byte = self.__comPort.read()
                # print(byte)
                if byte == b'\xff':
                    byte = self.__comPort.read()
                    if byte == b'\x02':
                        data = b'\xff\x02'
                        comData = self.__comPort.read(4)
                        length = int.from_bytes(comData[2:4], byteorder='big', signed=False)
                        # print('len:{}'.format(length))
                        data += comData
                        data += self.__comPort.read(length + 3)
                        # print(data)
                        if data[-1] == 0x03:    # get a frame of data
                            print(data)
                            # crc calculate
                            if data[-3:-1] != dataPack.calCRC(data[:-3]):
                                print('[messageThread]\tcrc should be {} received {}'. format(dataPack.calCRC(data), data[-3:-1]))
                                continue

                            # update the state of sensor
                            if self.__outFre != 0 and data[3] == 0x90:
                                self.state = self.parseContinueData(data)

                                continue
                            # put data to the messageQueue
                            if self.messageQueue.full():
                                self.messageQueue.get()
                            self.messageQueue.put(data)

    def __selectMessage(self, cmd):
        """
        select a message from messageQueue
        :param cmd:
        :return: match - data, no match - None
        """
        for i in range(self.messageQueue.qsize() - 1, -1, -1):
            data = self.messageQueue.queue[i]
            # print(cmd, data)
            if data[3] == cmd:
                return data
        else:
            return None

    def __setCmd(self, cmd, length, data):
        comData = b'\xff\x02\x00' + cmd.to_bytes(1, byteorder='big') + length.to_bytes(2, byteorder='big') + bytes(data)
        # crc calculate
        crc = dataPack.calCRC(comData)
        comData += crc + b'\x03'
        # sent data to sensor
        self.__comPort.write(comData)
        return comData

    def __enquireData(self, cmd, length, returnCmd, sendData=b''):
        """
        send command and receive data of sensor
        :param cmd:
        :param length:
        :param returnCmd:
        :param sendData:
        :return:
        """
        # lock the port
        self.__portLock = True
        time.sleep(self.__comTimeout)

        # ask for data
        self.__setCmd(cmd, length, sendData)
        self.__portLock = False

        # read data
        errTimes = 0

        while errTimes < 5:
            self.__setCmd(cmd, length, sendData)
            data = self.__selectMessage(returnCmd)
            if data is not None:
                break
            time.sleep(1/(self.__outFre + 5))
            errTimes += 1
        else:
            ERROR_Func(ERR.TIMEOUT)
            return ERR.TIMEOUT

        # get data from a frame
        dataLen = data[4] << 8 | data[5]
        data = data[6: 6 + dataLen]

        return dataLen, data

    def parseContinueData(self, data):
        """
        parse auto sent data
        :return: state information
        """
        dataLen = data[4] << 8 | data[5]
        dataClass = int(dataLen / 12)
        beginNum = 0
        roll, pitch, yaw = 0, 0, 0
        gx, gy, gz = 0, 0, 0
        ax, ay, az = 0, 0, 0

        if (self.__outMask & 0x01) and dataClass:  # roll, pitch, yaw
            roll = dataPack.u32ToFloat(data[beginNum+6: beginNum+10])
            pitch = dataPack.u32ToFloat(data[beginNum+10: beginNum+14])
            yaw = dataPack.u32ToFloat(data[beginNum+14: beginNum+18])
            dataClass -= 1
            beginNum += 12
        if (self.__outMask & 0x02) and dataClass:   # gx, gy, gz
            gx = dataPack.u32ToFloat(data[beginNum+6: beginNum+10])
            gy = dataPack.u32ToFloat(data[beginNum+10: beginNum+14])
            gz = dataPack.u32ToFloat(data[beginNum+14: beginNum+18])
            dataClass -= 1
            beginNum += 12
        if (self.__outMask & 0x04) and dataClass:   # ax, ay, az
            ax = dataPack.u32ToFloat(data[beginNum+6: beginNum+10])
            ay = dataPack.u32ToFloat(data[beginNum+10: beginNum+14])
            az = dataPack.u32ToFloat(data[beginNum+14: beginNum+18])

        return [roll, pitch, yaw,  gx, gy, gz,  ax, ay, az]

    def getContinueMode(self):
        """
        get frequency of continue mode
        :return: fre - 0,enquire mode; others,output frequency
        """
        data = self.__enquireData(0x54, 0x00, 0x55)

        data = data[1]
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
        assert frequency in availFre, "frequency should be {}, not {}!".format(availFre, frequency)
        div = 0

        if frequency == 0:  # require mode
            mode = 0x00
        else:               # continue mode
            mode = 0x01
            div = int(200 / frequency)
        # sent command
        print(self.__setCmd(0x53, 0x03, [0, mode, div]))
        self.__enquireData(0x53, 0x03, 0x01, [0, mode, div])

    def getOutputMask(self):
        """
        get the data class which sensor send
        mask -  0x01, EULER;
                0x02, GYROSCOPES;
                0x04, ACCELEROMETERS
        :return: mask - [EULER, GYROSCOPES, ACCELEROMETERS]
        """
        output = [0, 0, 0]  # EULER, GYROSCOPES, ACCELEROMETERS
        u32List = self.__enquireData(0x51, 0x00, 0x52)[1]
        byteData = bytearray()
        # convert data to bytearray
        for i in u32List:
            byteData.append(i)
        print(byteData)
        mask = int.from_bytes(byteData, byteorder='little', signed=False)   # convert byte to int
        if mask & 0x01:
            output[0] = 1
        if mask & 0x02:
            output[1] = 1
        if mask & 0x04:
            output[2] = 1

        return int.from_bytes(output, byteorder='little')

    def saveAllSetting(self):
        """
        save all setting to the EEPROM of sensor
        :return:
        """
        data = self.__enquireData(0x24, 0x00)[1]
        return data


if __name__ == "__main__":
    ser = MEMS("COM3", 115200, printInfo=True)

    import matplotlib.pyplot as plt
    import numpy as np
    imgQueue = [Queue(50), Queue(50), Queue(50)]

    # plt.ion()
    # while 1:
    #     time.sleep(0.3)
    #     for i in imgQueue:
    #         if i.full():
    #             i.get()
    #     imgQueue[0].put(ser.state[0])
    #     imgQueue[1].put(ser.state[1])
    #     imgQueue[2].put(ser.state[2])
    #     print(ser.state)
    #
    #     plt.pause(0.1)
    #
    #     plt.subplot(311)
    #     plt.cla()
    #     plt.title('Roll')
    #     plt.xlim((0, 50))
    #     plt.ylim((-np.pi, np.pi))
    #     plt.plot(range(0, imgQueue[0].qsize()), imgQueue[0].queue, color='r')
    #
    #     plt.subplot(312)
    #     plt.cla()
    #     plt.title('Pitch')
    #     plt.xlim((0, 50))
    #     plt.ylim((-np.pi, np.pi))
    #     plt.plot(range(0, imgQueue[1].qsize()), imgQueue[1].queue, color='g')
    #
    #     plt.subplot(313)
    #     plt.cla()
    #     plt.title('Yaw')
    #     plt.xlim((0, 50))
    #     plt.ylim((-np.pi, np.pi))
    #     plt.plot(range(0, imgQueue[2].qsize()), imgQueue[2].queue, color='b')
    #
    # plt.show()



        # print(int(ser.state[0]), int(ser.state[1]), int(ser.state[2]))
    # time.sleep(1)
    ser.setContinueFre(1)
    print(ser.getContinueMode())
    # print(u32ToFloat([0xbb, 0xff, 0x0f, 0x9e]))
    # data = ser.setCmd(0x53, 3, [1,1,1])
    # print(ser.getContinueMode())
    # print(data)

