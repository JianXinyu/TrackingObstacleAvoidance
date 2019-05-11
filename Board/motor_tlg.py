#!/usr/bin/python
# -*- coding: utf-8 -*-

import serial
import sys
import struct
import modbus_tk
import modbus_tk.defines as cst
import modbus_tk.modbus_rtu as modbus_rtu
from msgdev import MsgDevice, PeriodTimer
from Motor import *

RTU_port = 'COM7'
# RTU_port = '/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_AH02J3PA-if00-port0'
#RTU_port = '/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_AH02JGNO-if00-port0'

if __name__ == "__main__":

    try:
        #Connect to the slave
        master = modbus_rtu.RtuMaster(
            serial.Serial(port=RTU_port, baudrate=57600,
                          bytesize=8, parity='E', stopbits=1, xonxoff=0))
        master.set_timeout(1.0)
        master.set_verbose(True)

        #Connect to control program
        dev_pro = MsgDevice()
        dev_pro.open()
        dev_pro.sub_connect('tcp://192.168.1.60:55002') #receive rpm from veristand
        dev_pro.pub_bind('tcp://0.0.0.0:55003') #send information to veristand

        # Connect to joystick
        dev_joy = MsgDevice()
        dev_joy.open()
        dev_joy.sub_connect('tcp://192.168.1.60:55001') #receive rpm from joystick
        dev_joy.sub_add_url('js.autoctrl')

        left_motor = Motor(dev_pro, dev_joy,'left', 0x01, master)
        right_motor = Motor(dev_pro,dev_joy,'right', 0x02, master)

        t = PeriodTimer(0.1)
        t.start()
        while True:
            with t:
                autoctrl = dev_joy.sub_get1('js.autoctrl')
                dev_pro.pub_set1('autoctrl', autoctrl)
                #print('Autoctrl:', autoctrl)
                left_motor.update(autoctrl)
                right_motor.update(autoctrl)
                #print('rpm1', left_motor.Motor_SpeedCalc, 'rpm2',right_motor.Motor_SpeedCalc)
                

    except (KeyboardInterrupt, Exception, AbortProgram, modbus_tk.modbus.ModbusError) as e:
        sys.stdout.write('bye-bye\r\n')
        
        left_motor.close()
        right_motor.close()

        dev_pro.close()
        dev_joy.close()
        master.close()
        raise
    finally:
        pass