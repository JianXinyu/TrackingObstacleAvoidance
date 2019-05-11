#!/usr/bin/env python
import serial
from msgdev import MsgDevice
import struct

# serial port baudrate
BAUDRATE = 115200
# connection timeout in seconds
TIMEOUT = 1
# header to recognize
Header = b'\xff\x02'
# type of each data
Fst = struct.Struct("<9fL")

def hexShow(argv):
    result = ''
    hLen = len(argv)
    for i in xrange(hLen):
        hvol = ord(argv[i])
        hhex = '%02x' % hvol
        result += hhex+' '
    # print('hexShow:', result)

def crc16(x):
    poly = 0x8408
    crc = 0x0
    for byte in x:
        crc = crc^byte
        for i in range(8):
            last = (0xFFFF&crc)&1
            crc = (0xffff&crc)>>1
            if last ==1: crc = crc^poly
    return crc&0xFFFF

def console_logger():
    import logging

    # create logger
    logger = logging.getLogger("ahrs")
    logger.setLevel(logging.INFO)

    # create console handler and set level to debug
    ch = logging.StreamHandler()
    ch.setLevel(logging.DEBUG)

    # create formatter
    formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')

    # add formatter to ch
    ch.setFormatter(formatter)

    # add ch to logger
    logger.addHandler(ch)

    return logger

class AbortProgram(Exception):
    pass

class AHRS:
    def __init__(self,url,dev_pro):
        self.url = url
        self.prefix = 'ahrs'
        self.dev = dev_pro
        self.logger = console_logger()
        self.ser = serial.Serial(self.url,BAUDRATE, timeout=TIMEOUT)
        self.header = Header
        self.buf = ""
        self.fst = Fst
        self.logger.info('AHRS open: '+self.url)
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        self.roll_speed = 0
        self.pitch_speed = 0
        self.yaw_speed = 0
        self.acce_x = 0
        self.acce_y = 0
        self.acce_z = 0
        self.devicestatus = 0

    def close(self):
        self.ser.close()
        self.logger.info('AHRS close: '+self.url)

    def update(self):
        self.buf = self.ser.read(48)
        # print(self.buf)
        idx = self.buf.find(self.header)
        if idx < 0:
            # header not found, discard buffer
            self.buf = ""
            self.logger.warning('AHRS data header not found')

        elif idx > 0:
            # header not at start, discard bytes before header
            self.buf = self.buf[idx:]
            self.logger.warning('AHRS data header is not at start')

        else:

            if len(self.buf)<48:
                # not enough bytes
                self.logger.warning('AHRS data not long enough')

            else:
                # from here, we got 48 bytes in buffer and the first 2 bytes is 0xff 0x02
                #print(self.buf[2:6])
                datas = self.fst.unpack(self.buf[5:45])
                fff = struct.Struct(">H")
                crcnum = fff.unpack(self.buf[45:47])
                if crc16(self.buf[2:45]) != crcnum[0]:
                    # ckcum error, discart first 2 bytes and read again
                    self.buf = self.buf[2:]
                    self.logger.warning('AHRS data cannot pass crc')
                
                else:
                    # print ""
                    # cksum ok, publish data, discard buffer for next frame 
                    self.buf = ""
                    # data names: ['data_count','roll', 'pitch', 'yaw','roll_speed','pitch_speed','yaw_speed','acce_x','acce_y','acce_z','devicestatus']
                    self.roll = datas[0]
                    self.pitch = datas[1]
                    self.yaw = datas[2]
                    self.roll_speed = datas[3]
                    self.pitch_speed = datas[4]
                    self.yaw_speed = datas[5]
                    self.acce_x = datas[6]
                    self.acce_y = datas[7]
                    self.acce_z = datas[8]
                    self.devicestatus = datas[9]
                    self.publish()

    def publish(self):
        print('yaw:',self.yaw)
        self.dev.pub_set1(self.prefix+'.roll',self.roll)
        self.dev.pub_set1(self.prefix+'.pitch',self.pitch)
        self.dev.pub_set1(self.prefix+'.yaw',self.yaw)
        self.dev.pub_set1(self.prefix+'.roll_speed',self.roll_speed)
        self.dev.pub_set1(self.prefix+'.pitch_speed',self.pitch_speed)
        self.dev.pub_set1(self.prefix+'.yaw_speed',self.yaw_speed)
        self.dev.pub_set1(self.prefix+'.acce_x',self.acce_x)
        self.dev.pub_set1(self.prefix+'.acce_y',self.acce_y)
        self.dev.pub_set1(self.prefix+'.acce_z',self.acce_z)
        self.dev.pub_set1(self.prefix+'.devicestatus',self.devicestatus)


#AHRS_URL = '/dev/ttyUSB0'
AHRS_URL = '/dev/serial/by-id/usb-Silicon_Labs_SBG_Systems_-_UsbToUart_001000929-if00-port0'
#AHRS_URL = '/dev/serial/by-id/usb-1a86_USB2.0-Serial-if00-port0'
#AHRS_URL = 'usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0'
#AHRS_URL = 'COM3'

if __name__ == "__main__":
    try:
        dev_pro = MsgDevice()
        dev_pro.open()
        dev_pro.pub_bind('tcp://0.0.0.0:55005')
        ahrs = AHRS(AHRS_URL,dev_pro)
        while True:
            ahrs.update()
    except (KeyboardInterrupt, Exception) as e:
        dev_pro.close()
        ahrs.close()
        raise
    finally:
        pass