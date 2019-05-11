import serial
import sys
import struct

#add logging capability
import modbus_tk
import modbus_tk.defines as cst
import modbus_tk.modbus_rtu as modbus_rtu


from msgdev import MsgDevice, PeriodTimer

def console_logger():
    import logging

    # create logger
    logger = logging.getLogger("motor")
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


class Motor:
    """
    Class to Store Motor Status
    """
    def __init__(self, dev_pro,dev_joy, prefix, nodeId, RTU_Master):
        self.Motor_PWM = 0
        self.Motor_Current = 0
        self.Motor_Frequency = 0
        self.Motor_SpeedCalc = 0
        self.Motor_Error = 0
        self.Motor_Speed = 0    # RealTime Speed read from Motor Controller
        self.Target_Speed = 0   # Target Speed is received from Veristand
        self.Target_Speed_old = 0
        self.Motor_Address = nodeId
        self.Motor_Master = RTU_Master
        self.poweroff = 0

        self.prefix = prefix
        self.logger = console_logger()
        self.logger.info(self.prefix+'.motor running')

        self.dev_pro = dev_pro
        self.dev_pro.sub_add_url('pro.'+self.prefix+'.speed')

        self.dev_joy = dev_joy        
        self.dev_joy.sub_add_url('js.'+self.prefix+'.speed')

        self.writecount = 0
        self.readcount = 0
        self.stopcount = 0

    def update(self,autoctrl):
        if autoctrl:
            target = self.dev_pro.sub_get1('pro.'+self.prefix+'.speed')
        else:
            target = self.dev_joy.sub_get1('js.'+self.prefix+'.speed')
        self.Set_Target_Speed(target)
        self.Update_Motor_Status()

    def close(self):
        self.Set_Motor_Stop()
        self.logger.info(self.prefix+'.motor stop')

    def Update_Motor_Status(self):
        try: 
            StatusArray = self.Motor_Master.execute(
                self.Motor_Address, cst.READ_HOLDING_REGISTERS, 0x0020, 3)
            bytes = struct.pack('!3H', *StatusArray)
            StatusArray = struct.unpack('!3h', bytes)
            # read three registers with the startaddress 0x20 ,
            # They are PWM, Current, Frequency
            self.Motor_PWM = StatusArray[0]*0.1
            self.Motor_Current = StatusArray[1]*0.01
            self.Motor_Frequency = StatusArray[2]*0.1
            self.Motor_SpeedCalc = self.Motor_Frequency*20.0/4 # 4 is the polar number of the motor
            StatusArray = self.Motor_Master.execute(
                self.Motor_Address, cst.READ_HOLDING_REGISTERS, 0x0033, 2)
            #read two registers with the startaddress 0x33,
            # They are ErrorNumber and real RPM
            bytes = struct.pack('!2H', *StatusArray)
            StatusArray = struct.unpack('!2h', bytes)
            self.Motor_Error = StatusArray[0]*1.0
            self.Motor_Speed = StatusArray[1]*1.0
            StatusArray = self.Motor_Master.execute(
                self.Motor_Address, cst.READ_HOLDING_REGISTERS, 0x0028, 1)
            bytes = struct.pack('!H', *StatusArray)
            StatusArray = struct.unpack('!h', bytes)
            self.power = StatusArray[0]
            self.readcount = 0
        except(modbus_tk.modbus.ModbusInvalidResponseError):
            self.readcount += 1
            print(self.prefix+'_readcount', self.readcount)
            if self.readcount > 3:
                raise
            else:
                pass

        self.publish()

    def publish(self):
        self.dev_pro.pub_set1(self.prefix+'.Motor_PWM', self.Motor_PWM)
        self.dev_pro.pub_set1(self.prefix+'.Motor_Current', self.Motor_Current)
        self.dev_pro.pub_set1(self.prefix+'.Motor_Frequency', self.Motor_Frequency)
        self.dev_pro.pub_set1(self.prefix+'.Motor_SpeedCalc', self.Motor_SpeedCalc)
        self.dev_pro.pub_set1(self.prefix+'.Motor_Error', self.Motor_Error)
        self.dev_pro.pub_set1(self.prefix+'.Motor_Speed', self.Motor_Speed)


    def Set_Target_Speed(self, Target_Speed):
        if Target_Speed > 3000:
            Target_Speed = 3000
            print('Target_Speed is larger than 3000!!!')
        self.Target_Speed = int(Target_Speed)
        if self.Target_Speed != self.Target_Speed_old:
            try:
                self.Motor_Master.execute(
                    self.Motor_Address, cst.WRITE_SINGLE_REGISTER, 0x0043,
                    output_value=self.Target_Speed*2)
                self.writecount = 0
            except(modbus_tk.modbus.ModbusError):
                self.writecount += 1
                print(self.prefix+'_writecount', self.writecount)
                if self.writecount > 3:
                    raise
                else:
                    pass

        self.Target_Speed_old = self.Target_Speed


    def Set_Motor_Stop(self):
        try:
            self.Motor_Master.execute(
                self.Motor_Address, cst.WRITE_SINGLE_REGISTER, 0x0040, 2)
            self.stopcount = 0
        except(modbus_tk.modbus.ModbusInvalidResponseError):
            self.stopcount += 1
            print(self.prefix+'_stopcount', self.stopcount)
            if self.stopcount > 3:
                raise
            else:
                pass


if __name__ == "__main__":

    #RTU_port = '/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_AH02J3PA-if00-port0'
    RTU_port = 'COM7'
    # RTU_port = '/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_AH02JGNO-if00-port0'
    # RTU_port = '/dev/ttyS2'
    # TRU_logger = modbus_tk.utils.create_logger("console")

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
        dev_pro.sub_connect('tcp://127.0.0.1:55002')
        dev_pro.pub_bind('tcp://0.0.0.0:55003')

        # Connect to joystick
        dev_joy = MsgDevice()
        dev_joy.open()
        dev_joy.sub_connect('tcp://127.0.0.1:55001')
        dev_joy.sub_add_url('js.autoctrl')

        left_motor = Motor(dev_pro, dev_joy,'left', 0x01, master)
        right_motor = Motor(dev_pro,dev_joy,'right', 0x02, master)

        t = PeriodTimer(0.1)
        t.start()
        while True:
            with t:
                autoctrl = dev_joy.sub_get1('js.autoctrl')
                dev_pro.pub_set1('autoctrl', autoctrl)
                print('Autoctrl:', autoctrl)
                left_motor.update(autoctrl)
                right_motor.update(autoctrl)
                print('rpm1', left_motor.Motor_SpeedCalc, 'rpm2',right_motor.Motor_SpeedCalc)

                #print LeftMotor.Motor_PWM, LeftMotor.Motor_Current, LeftMotor.Motor_Frequency, LeftMotor.Motor_SpeedCalc, LeftMotor.Motor_Error, LeftMotor.Motor_Speed
                #print RightMotor.Motor_PWM, RightMotor.Motor_Speed
                

    except (KeyboardInterrupt, Exception, AbortProgram, modbus_tk.modbus.ModbusError) as e:
        sys.stdout.write('bye-bye\r\n')

        # LeftMotor.Set_Target_Speed(0)
        # RightMotor.Set_Target_Speed(0)
        
        left_motor.close()
        right_motor.close()

        dev_pro.close()
        dev_joy.close()
        master.close()
        raise
    finally:
        pass
