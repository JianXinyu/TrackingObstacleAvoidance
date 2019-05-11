#!/usr/bin/python
# -*- coding: utf-8 -*-
import time
import pygame
import logging
from logging.handlers import RotatingFileHandler

from msgdev import MsgDevice, PeriodTimer

class Interface(object):
    def __init__(self, sub_addr, ahrs_port, gnss_port, motor_port):

        self.dev = MsgDevice()
        self.dev.open()

        self.dev.sub_connect(sub_addr+':'+ahrs_port)
        self.dev.sub_add_url('ahrs.roll')
        self.dev.sub_add_url('ahrs.pitch')
        self.dev.sub_add_url('ahrs.yaw')
        self.dev.sub_add_url('ahrs.roll_speed')
        self.dev.sub_add_url('ahrs.pitch_speed')
        self.dev.sub_add_url('ahrs.yaw_speed')
        self.dev.sub_add_url('ahrs.acce_x')
        self.dev.sub_add_url('ahrs.acce_y')
        self.dev.sub_add_url('ahrs.acce_z')

        self.dev.sub_connect(sub_addr+':'+gnss_port)
        self.dev.sub_add_url('gps.posx')
        self.dev.sub_add_url('gps.posy')
        self.dev.sub_add_url('gps.posz')
        self.dev.sub_add_url('gps.stdx')
        self.dev.sub_add_url('gps.stdy')
        self.dev.sub_add_url('gps.stdz')
        self.dev.sub_add_url('gps.hspeed')
        self.dev.sub_add_url('gps.vspeed')
        self.dev.sub_add_url('gps.track')

        self.dev.pub_bind('tcp://0.0.0.0:'+motor_port)

    def Sensor_receive(self, *args):
        package = []
        for i in args:
            package.append(self.dev.sub_get1(i))
        return package

    def Motor_send(self, left_motor, right_motor, autoctrl):
        self.dev.pub_set1('js.left.speed', left_motor)
        self.dev.pub_set1('js.right.speed', right_motor)
        self.dev.pub_set1('js.autoctrl', autoctrl)


class AbortProgram(Exception):
    pass


if __name__ == "__main__":

    sub_addr = 'tcp://127.0.0.1'
    ahrs_port = '55005'
    gnss_port = '55004'
    motor_port = '55001'
    interface = Interface(sub_addr, ahrs_port, gnss_port, motor_port)

    speed_limit = 1200
    pygame.init()
    pygame.joystick.init()

    logger = logging.getLogger(__name__)
    logger.setLevel(level = logging.INFO)

    rHandler = RotatingFileHandler("log-{:s}.txt".format(time.strftime("%m-%d-%H-%M", time.localtime())), \
                                    maxBytes=100*1024*1024, backupCount=10)
    rHandler.setLevel(logging.INFO)
    formatter = logging.Formatter('%(message)s')
    rHandler.setFormatter(formatter)

    console = logging.StreamHandler()
    console.setLevel(logging.INFO)
    console.setFormatter(formatter)

    logger.addHandler(rHandler)
    logger.addHandler(console)

    try:
        t = PeriodTimer(0.1)
        t.start()
        while True:
            with t:
                pygame.event.get()
                joystick_count = pygame.joystick.get_count()
                for i in range(joystick_count):
                    Joystick1 = pygame.joystick.Joystick(i)
                    if Joystick1.get_name() == "Mad Catz F.L.Y.5 Stick":
                        joystick = pygame.joystick.Joystick(i)
                
                joystick.init()
                diff = joystick.get_axis(0)

                if abs(diff) < 0.1:
                    diff = 0
                
                aver = joystick.get_axis(1)
                if abs(aver) < 0.1:
                    aver = 0

                state = joystick.get_axis(2)

                averspeed = -aver*speed_limit
                diffspeed = 0.5*diff*speed_limit
                left_motor = -(averspeed+diffspeed)
                right_motor = averspeed-diffspeed

                if left_motor > speed_limit:
                    left_motor = speed_limit
                if left_motor < -speed_limit:
                    left_motor = -speed_limit
                if right_motor > speed_limit:
                    right_motor = speed_limit
                if right_motor < -speed_limit:
                    right_motor = -speed_limit
                
                if state >= 0:
                    autoctrl = 1
                else:
                    autoctrl = 0

                interface.Motor_send(left_motor, right_motor, autoctrl)

                package = interface.Sensor_receive('ahrs.roll', 'ahrs.pitch', 'ahrs.yaw', \
                    'ahrs.roll_speed', 'ahrs.pitch_speed', 'ahrs.yaw_speed', \
                    'ahrs.acce_x', 'ahrs.acce_y', 'ahrs.acce_z', \
                    'gps.posx', 'gps.posy', 'gps.posz', \
                    'gps.stdx', 'gps.stdy', 'gps.stdz', \
                    'gps.hspeed', 'gps.vspeed', 'gps.track')
                package += [left_motor, right_motor, diff, aver]
                logger.info('{0:.4f} {1:.4f} {2:.4f} {3:.4f} {4:.4f} {5:.4f} {6:.4f} {7:.4f} {8:.4f} {9:.4f} {10:.4f} {11:.4f} {12:.4f} {13:.4f} {14:.4f} {15:.4f} {16:.4f} {17:.4f} {18:.4f} {19:.4f} {20:.4f} {21:.4f} {21:.4f}'.format(package[0], package[1], package[2], \
                            package[3], package[4], package[5], package[6], package[7], package[8], package[9], \
                            package[10], package[11], package[12], package[13], package[14], package[15], package[16], \
                            package[17], package[18], package[19], package[20], package[21]))
 
    except (Exception, KeyboardInterrupt, AbortProgram) as e:
        pygame.quit()
        interface.dev.close()
        print('dev closed')
