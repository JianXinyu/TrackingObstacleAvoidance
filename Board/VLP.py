#!/usr/bin/python
# -*- coding: utf-8 -*-

from math import *
from socket import *
from struct import *
from msgdev import MsgDevice,PeriodTimer
import numpy as np
import time

# User Settings
VLP16_Connected = 1 #if not connected, this program will use self-created lidar data
data_addr = 'tcp://0.0.0.0:55010' #send data to this address, including dis384,ref384,azi24
t_refresh = 1 #period to refresh

#VLP16 Settings
vlp_frequency = 5 #frequency of vlp16, matching its setting on web
vlp_returnmode = b'\x37' #37-Strongest Return,38-Last Return. Duel Return mode is not available here
max_ref = 255 #a point with reflectivity larger than this value will be regarded as noise, up to 255
min_ref = 0  #a point with reflectivity lower than this value will be regarded as noise, down to 0

# Parameter Calculation
vlp_addr = ('',2368) #ip and port of vlp16
res_azi = vlp_frequency*2  #resolution of azimuth angle
interval_packet = 1 #when set 0,every packet will be used,when set 1,one of every two packers will be used,etc.
num_packet = ceil(36000/(res_azi*24*(interval_packet+1))) #number of packets in one image,150 when vlp_frequency is 5Hz.
packet_created = (b'\xff\xee\x33\x71'+b'\x89\x59\x17'*32)*12+b'\x61\x67\xb9\x5a\x37\x22' #when vlp16 is not connected, use this packet to debug

# Structure of vlp16 raw data
PacketTail = vlp_returnmode+b'\x22'
BlockHeader = b'\xff\xee'
azi_struct = Struct("<H")
dis_struct = Struct("<HB")
time_struct = Struct("<L")
factory_struct = Struct("<2B")

# Structure of vlp16 packet
flag = np.dtype('<u2')
azimuth  = np.dtype('<u2')
distance = np.dtype('<u2')
reflectivity = np.dtype('<u1')
channel = np.dtype([('distance',distance,1),('reflectivity',reflectivity,1)])
block = np.dtype([('flag',flag,1),('azimuth',azimuth,1),('channel',channel,32)])
packet = np.dtype([('block',block,12)])


def devinitial(dev):
    dev.pub_bind(data_addr)
    dev.sub_connect('tcp://192.168.1.150:55004')
    dev.sub_connect('tcp://192.168.1.150:55005')
    dev.sub_add_url('ahrs.roll')
    dev.sub_add_url('ahrs.pitch')
    dev.sub_add_url('ahrs.yaw')
    dev.sub_add_url('gps.posx')
    dev.sub_add_url('gps.posy')

class VLP:
    def __init__(self,dev):
        #create socket for UDP client
        try:
            self.s = socket(AF_INET,SOCK_DGRAM)
        except socket.error as msg:
            print('Failed to create socket. Error code:' + str(msg[0]) + ', Error message:' + msg[1])
            raise
        else:
            print('Socket Created.')

        #connect UDP client to server
        try:
            self.s.bind(vlp_addr)
        except Exception:
            print('Failed to connect.') 
            raise
        else:
            print('VLP-16 Connected.')
        self.s.settimeout(2)
        self.dev = dev
        self.timestamp = np.zeros(1)

    def capture(self):
        if VLP16_Connected != 0:
            self.buf = self.s.recv(1206)   #length of a packet is 1206
            # confirm the packet is ended with PacketTail
            while self.buf[1204:1206] != PacketTail:
                print('Wrong Factory Information!')
                self.buf  = self.s.recv(1206)
        else:
            self.buf = packet_created

    def parse(self):
        datas = np.frombuffer(self.buf,dtype=packet,count=1)
        self.dis384 = datas['block']['channel']['distance']
        self.ref384 = datas['block']['channel']['reflectivity']
        azi12ori = datas['block']['azimuth']
        azi12ori = azi12ori.reshape(12,1)
        azi12add = azi12ori + res_azi
        azi12add = azi12add-(azi12add>=36000)*36000
        self.azi24 = np.column_stack((azi12ori,azi12add))

    def publish(self):
        self.timestamp += 1
        posx = self.dev.sub_get1('gps.posx')
        posy = self.dev.sub_get1('gps.posy')
        roll = dev.sub_get1('ahrs.roll')
        pitch = dev.sub_get1('ahrs.pitch')
        yaw = dev.sub_get1('ahrs.yaw')
        self.image = self.image+[posx,posy,roll,pitch,yaw,self.timestamp]
        #packet = np.hstack((self.dis384.flatten(),self.ref384.flatten(),self.azi24.flatten(),self.timestamp))
        self.dev.pub_set('vlp.image',self.image)

    def update(self):
        self.image = []
        for i in range(num_packet):
            self.capture()
            self.parse()
            self.packet = np.hstack((self.dis384.flatten(),self.ref384.flatten(),self.azi24.flatten()))
            self.image = self.image+self.packet.tolist()
            for j in range(interval_packet):
                self.capture()
        # self.capture()
        # self.parse()
        self.publish()


    def close(self):
        self.s.close()
        print('VLP-16 Disconnected.')

    



if __name__ == "__main__":
    try:
        dev = MsgDevice()
        dev.open()
        devinitial(dev)
        vlp = VLP(dev)
        t = PeriodTimer(t_refresh)
        t.start()
        while True:
            with t:
                print(time.strftime('%Y-%m-%d-%H-%M-%S',time.localtime()))
                vlp.update()
            
    except KeyboardInterrupt:
        vlp.close()
        dev.close()
    except Exception:
        vlp.close()
        dev.close()
        raise
    else:
        vlp.close()
        dev.close()
    finally:
        pass
