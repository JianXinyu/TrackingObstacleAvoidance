#!/usr/bin/env python
import serial
from msgdev import MsgDevice,PeriodTimer
import struct

URL_VOLTAGE = '/dev/serial/by-id/usb-Arduino__www.arduino.cc__0043_85435333131351F07171-if00'
# serial port baudrate
BAUDRATE = 9600
# connection timeout in seconds
TIMEOUT = 1

def console_logger():
    import logging

    # create logger
    logger = logging.getLogger("voltage")
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

class Voltage:
    def __init__(self,url,dev_pro):
        self.url = url
        self.dev = dev_pro
        self.logger = console_logger()
        self.ser = serial.Serial(self.url,BAUDRATE, timeout=TIMEOUT)
        self.logger.info('VOLTAGE open: '+self.url)
        self.voltage = 0

    def close(self):
        self.ser.close()
        self.logger.info('VOLTAGE close: '+self.url)

    def update(self):
        self.ser.flushInput()
        vol = self.ser.readline()
        if vol != '':
            try:
                self.voltage = float(vol)
                self.publish()
            except (ValueError):
                pass
    
    def publish(self):
        self.dev.pub_set1('voltage',self.voltage)


if __name__ == "__main__":
    vol_url = URL_VOLTAGE
    try:
        dev_pro = MsgDevice()
        dev_pro.open()
        dev_pro.pub_bind('tcp://0.0.0.0:55006')
     
        voltage = Voltage(vol_url,dev_pro)
        
        while True:
            voltage.update()
            print(voltage.voltage)
    except (KeyboardInterrupt, Exception, AbortProgram) as e:
        dev_pro.close()
        voltage.close()
        raise
    finally:
        dev_pro.close()
        voltage.close()

        
