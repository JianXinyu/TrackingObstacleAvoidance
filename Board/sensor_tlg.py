#!/usr/bin/python
# -*- coding: utf-8 -*-

from math import *
from msgdev import MsgDevice,MsgDeviceError,PeriodTimer
import serial
import struct
from GNSS import *
from AHRS import *
from Voltage import *

# gnss receiver address
# if using serial port, set it something like to "COM1" or "/dev/tty1"
GPS_URL = "socket://192.168.1.100:40000"
#URL_MovingStation = "COM5"

# commands sent to GNSS receiver
GPS_INIT_COMMANDS = """UNLOGALL
FIX NONE
LOG BESTPOSA ontime 0.2
LOG BESTVELA ontime 0.2
"""

GPS_MS_COMMANDS = """UNLOGALL
NETPORTCONFIG ICOM1 192.168.1.60 TCP 40000
INTERFACEMODE ICOM1 RTCMV3 NONE ON
SAVECONFIG
"""


AHRS_URL = '/dev/serial/by-id/usb-Silicon_Labs_SBG_Systems_-_UsbToUart_001000929-if00-port0'


VOLTAGE_URL = '/dev/serial/by-id/usb-Arduino__www.arduino.cc__0043_85435333131351F07171-if00'

if __name__ == "__main__":
    try:
        dev_pro = MsgDevice()
        dev_pro.open()
        dev_pro.pub_bind('tcp://0.0.0.0:55004')
     
        gps = GNSS(GPS_URL)
        gps.write(GPS_MS_COMMANDS)
        gps.write(GPS_INIT_COMMANDS)

        ahrs = AHRS(AHRS_URL,dev_pro)

        voltage = Voltage(VOLTAGE_URL,dev_pro)

        data_count = 0

        while True:
            voltage.update()
            print 'Voltage:',voltage.voltage
            ahrs.update()
            print 'Yaw:',ahrs.yaw
            gps.update()
            gps.publish(dev_pro)
            print 'Posx:',gps.posx
            data_count += 1
            dev_pro.pub_set1('sensor.data_count',data_count)
    except (KeyboardInterrupt, Exception, GNSS_ERROR), e:
        dev_pro.close()
        voltage.close()
        ahrs.close()
        gps.close()
        raise
    finally:
    	pass