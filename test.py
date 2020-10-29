import MEMS_sensor
import serial
import time

if __name__ == "__main__":
    ser = serial.Serial("COM12", 115200, timeout=0.5)
    #ser.open()
    print('Serial Init OK!')

    myData = [0xff, 0x02, 0x00, 0x53, 0x00, 0x03, 0x00, 0x00, 0x00]
    crc = MEMS_sensor.calCRC(myData)
    myData.append(crc & 0xf0)
    myData.append(crc & 0x0f)
    myData.append(0x03)
    print(f'output: {myData}')

    # ser.write(myData)
    time.sleep(0.05)
    if ser.inWaiting():
        rec = ser.read_all()
    else:
        rec = None

    # print('return: {}'.format('%#x'%rec))
    # print('return: {}'.format(rec.hex()))
    for i in rec:
        print('%#x'%i, end=' ')

    # print(type(rec.hex()))
