#!/usr/bin/python
# -*- coding: utf-8 -*-

from math import *
from msgdev import MsgDevice,MsgDeviceError,PeriodTimer
import serial

# 
URL_BaseStation = 'COM14'
# serial port baudrate
BAUDRATE = 115200
# connection timeout in seconds
TIMEOUT = 2

# commands sent to GNSS receiver
INIT_COMMANDS = """UNLOGALL
FIX NONE
SAVECONFIG
LOG BESTPOSA ontime 0.2
LOG BESTVELA ontime 0.2
"""

BS_COMMANDS = """UNLOGALL
INTERFACEMODE COM2 NONE RTCMV3 ON
LOG COM2 RTCM1004 ONTIME 1
LOG COM2 RTCM1104 ONTIME 1
LOG COM2 RTCM1006 ONTIME 10
SAVECONFIG 
"""

MS_COMMANDS = """UNLOGALL
NETPORTCONFIG ICOM1 192.168.1.60 TCP 40000
INTERFACEMODE ICOM1 RTCMV3 NONE ON
SAVECONFIG
"""

# lat/lon of original point, use to caculate posx/posy (north and east are positive)
ORIGIN_LAT = 31.0232
ORIGIN_LON = 121.4251


novatel_tbl = (
	0x00000000, 0x77073096, 0xee0e612c, 0x990951ba, 0x076dc419, 0x706af48f,
	0xe963a535, 0x9e6495a3, 0x0edb8832, 0x79dcb8a4, 0xe0d5e91e, 0x97d2d988,
	0x09b64c2b, 0x7eb17cbd, 0xe7b82d07, 0x90bf1d91, 0x1db71064, 0x6ab020f2,
	0xf3b97148, 0x84be41de, 0x1adad47d, 0x6ddde4eb, 0xf4d4b551, 0x83d385c7,
	0x136c9856, 0x646ba8c0, 0xfd62f97a, 0x8a65c9ec, 0x14015c4f, 0x63066cd9,
	0xfa0f3d63, 0x8d080df5, 0x3b6e20c8, 0x4c69105e, 0xd56041e4, 0xa2677172,
	0x3c03e4d1, 0x4b04d447, 0xd20d85fd, 0xa50ab56b, 0x35b5a8fa, 0x42b2986c,
	0xdbbbc9d6, 0xacbcf940, 0x32d86ce3, 0x45df5c75, 0xdcd60dcf, 0xabd13d59,
	0x26d930ac, 0x51de003a, 0xc8d75180, 0xbfd06116, 0x21b4f4b5, 0x56b3c423,
	0xcfba9599, 0xb8bda50f, 0x2802b89e, 0x5f058808, 0xc60cd9b2, 0xb10be924,
	0x2f6f7c87, 0x58684c11, 0xc1611dab, 0xb6662d3d, 0x76dc4190, 0x01db7106,
	0x98d220bc, 0xefd5102a, 0x71b18589, 0x06b6b51f, 0x9fbfe4a5, 0xe8b8d433,
	0x7807c9a2, 0x0f00f934, 0x9609a88e, 0xe10e9818, 0x7f6a0dbb, 0x086d3d2d,
	0x91646c97, 0xe6635c01, 0x6b6b51f4, 0x1c6c6162, 0x856530d8, 0xf262004e,
	0x6c0695ed, 0x1b01a57b, 0x8208f4c1, 0xf50fc457, 0x65b0d9c6, 0x12b7e950,
	0x8bbeb8ea, 0xfcb9887c, 0x62dd1ddf, 0x15da2d49, 0x8cd37cf3, 0xfbd44c65,
	0x4db26158, 0x3ab551ce, 0xa3bc0074, 0xd4bb30e2, 0x4adfa541, 0x3dd895d7,
	0xa4d1c46d, 0xd3d6f4fb, 0x4369e96a, 0x346ed9fc, 0xad678846, 0xda60b8d0,
	0x44042d73, 0x33031de5, 0xaa0a4c5f, 0xdd0d7cc9, 0x5005713c, 0x270241aa,
	0xbe0b1010, 0xc90c2086, 0x5768b525, 0x206f85b3, 0xb966d409, 0xce61e49f,
	0x5edef90e, 0x29d9c998, 0xb0d09822, 0xc7d7a8b4, 0x59b33d17, 0x2eb40d81,
	0xb7bd5c3b, 0xc0ba6cad, 0xedb88320, 0x9abfb3b6, 0x03b6e20c, 0x74b1d29a,
	0xead54739, 0x9dd277af, 0x04db2615, 0x73dc1683, 0xe3630b12, 0x94643b84,
	0x0d6d6a3e, 0x7a6a5aa8, 0xe40ecf0b, 0x9309ff9d, 0x0a00ae27, 0x7d079eb1,
	0xf00f9344, 0x8708a3d2, 0x1e01f268, 0x6906c2fe, 0xf762575d, 0x806567cb,
	0x196c3671, 0x6e6b06e7, 0xfed41b76, 0x89d32be0, 0x10da7a5a, 0x67dd4acc,
	0xf9b9df6f, 0x8ebeeff9, 0x17b7be43, 0x60b08ed5, 0xd6d6a3e8, 0xa1d1937e,
	0x38d8c2c4, 0x4fdff252, 0xd1bb67f1, 0xa6bc5767, 0x3fb506dd, 0x48b2364b,
	0xd80d2bda, 0xaf0a1b4c, 0x36034af6, 0x41047a60, 0xdf60efc3, 0xa867df55,
	0x316e8eef, 0x4669be79, 0xcb61b38c, 0xbc66831a, 0x256fd2a0, 0x5268e236,
	0xcc0c7795, 0xbb0b4703, 0x220216b9, 0x5505262f, 0xc5ba3bbe, 0xb2bd0b28,
	0x2bb45a92, 0x5cb36a04, 0xc2d7ffa7, 0xb5d0cf31, 0x2cd99e8b, 0x5bdeae1d,
	0x9b64c2b0, 0xec63f226, 0x756aa39c, 0x026d930a, 0x9c0906a9, 0xeb0e363f,
	0x72076785, 0x05005713, 0x95bf4a82, 0xe2b87a14, 0x7bb12bae, 0x0cb61b38,
	0x92d28e9b, 0xe5d5be0d, 0x7cdcefb7, 0x0bdbdf21, 0x86d3d2d4, 0xf1d4e242,
	0x68ddb3f8, 0x1fda836e, 0x81be16cd, 0xf6b9265b, 0x6fb077e1, 0x18b74777,
	0x88085ae6, 0xff0f6a70, 0x66063bca, 0x11010b5c, 0x8f659eff, 0xf862ae69,
	0x616bffd3, 0x166ccf45, 0xa00ae278, 0xd70dd2ee, 0x4e048354, 0x3903b3c2,
	0xa7672661, 0xd06016f7, 0x4969474d, 0x3e6e77db, 0xaed16a4a, 0xd9d65adc,
	0x40df0b66, 0x37d83bf0, 0xa9bcae53, 0xdebb9ec5, 0x47b2cf7f, 0x30b5ffe9,
	0xbdbdf21c, 0xcabac28a, 0x53b39330, 0x24b4a3a6, 0xbad03605, 0xcdd70693,
	0x54de5729, 0x23d967bf, 0xb3667a2e, 0xc4614ab8, 0x5d681b02, 0x2a6f2b94,
	0xb40bbe37, 0xc30c8ea1, 0x5a05df1b, 0x2d02ef8d)

def novatel_crc32(data):
	tbl = novatel_tbl
	crc = 0
	for c in data:
		crc = tbl[(crc^ord(c))&0xff]^(crc>>8)
	return crc

def d2r(d):
	return d/180.0*pi

def console_logger():
	import logging

	# create logger
	logger = logging.getLogger("gnss")
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


class GNSS_ERROR(Exception):
	pass

class GNSS:
	def __init__(self,url):
		self.url = url
		self.prefix = 'gps'
		self.logger = console_logger()
		self.ser = serial.serial_for_url(self.url,do_not_open=True, baudrate=BAUDRATE, timeout=TIMEOUT)
		self.lat = d2r(ORIGIN_LAT)
		self.lon = d2r(ORIGIN_LON)
		self.posx = 0
		self.posy = 0
		self.posz = 0
		self.stdx = 0
		self.stdy = 0
		self.stdz = 0
		self.time = 0
		self.hspeed = 0
		self.vspeed = 0
		self.track = 0
		self.satn = 0
		self.open()
		self.write('UNLOGALL\n')
		self.write('UNLOGALL\n')

	def w84_calc_ne(self, lat2, lon2):
		lat1,lon1 = self.lat,self.lon
		lat2,lon2 = d2r(lat2),d2r(lon2)
		d_lat = lat2-lat1
		d_lon = lon2-lon1

		a = 6378137.0
		e_2 = 6.69437999014e-3
		r1 = a*(1-e_2)/(1-e_2*(sin(lat1))**2)**1.5
		r2 = a/sqrt(1-e_2*(sin(lat1))**2)

		north = r1*d_lat
		east = r2*cos(lat1)*d_lon
		return north,east

	def close(self):
		self.logger.info('GNSS close: '+self.url)
		self.ser.close()

	def open(self):
		self.logger.info('GNSS open: '+self.url)
		self.line = ''
		self.ser.open()
	def write(self,cmds):
		if cmds!=None and cmds!= "":
			self.ser.write(cmds.replace("\r","").replace("\n","\r\n").encode())

	def update(self):
		l = self.ser.readline().decode()
		if l == '':
			self.logger.warning('GNSS timeout, reconnect')
			self.close()
			self.open()
		self.line = self.line + l
		if self.line.endswith('\n'):
			# a new line received
			self.parse_line()
			self.line = ''

	def parse_line(self):
		if not self.line.startswith('#'):
			return
		self.line = self.line.rstrip('\r\n')
		if not self.nv_validate():
			return
		self.parse_header()
		self.parse_content()

	def parse_header(self):
		ps = self.header.split(',')
		if len(ps)!= 10:
			raise GNSS_ERROR('invalid novatel header '+self.header)

		self.time = float(ps[6])
		self.logtype = ps[0].upper()

	def parse_content(self):
		if self.logtype == "BESTPOSA":
			self.parse_bestposa()
		elif self.logtype == "BESTVELA":
			self.parse_bestvela()

	def parse_bestposa(self):
		ps = self.content.split(',')
		if len(ps)!=21:
			raise GNSS_ERROR('invalid bestposa '+self.content)
		if ps[1]=='NONE' or ps[0]!='SOL_COMPUTED':
			self.posx, self.posy, self.posz = 0,0,0
			self.stdx, self.stdy, self.stdz = 0,0,0
			self.satn = 0
			return

		lat,lon = float(ps[2]),float(ps[3])
		self.posx,self.posy = lat,lon
		self.posz = float(ps[4])

		self.stdx,self.stdy,self.stdz = float(ps[7]),float(ps[8]),float(ps[9])

		self.satn = float(ps[14])

	def parse_bestvela(self):
		ps = self.content.split(',')
		if len(ps)!=8:
			raise GNSS_ERROR('invalid bestvela '+self.content)
		if ps[1]=='NONE' or ps[0]!='SOL_COMPUTED':
			self.hspeed = 0
			self.vspeed = 0
			self.track = 0
			return
		self.hspeed = float(ps[4])
		self.track = float(ps[5])/180*pi
		self.vspeed = float(ps[6])

	def publish(self,dev_pro):
		self.dev = dev_pro
		self.dev.pub_set1(self.prefix+'.time',self.time)
		self.dev.pub_set1(self.prefix+'.posx',self.posx)
		self.dev.pub_set1(self.prefix+'.posy',self.posy)
		self.dev.pub_set1(self.prefix+'.posz',self.posz)
		self.dev.pub_set1(self.prefix+'.stdx',self.stdx)
		self.dev.pub_set1(self.prefix+'.stdy',self.stdy)
		self.dev.pub_set1(self.prefix+'.stdz',self.stdz)
		self.dev.pub_set1(self.prefix+'.satn',self.satn)
		self.dev.pub_set1(self.prefix+'.hspeed',self.hspeed)
		self.dev.pub_set1(self.prefix+'.vspeed',self.vspeed)
		self.dev.pub_set1(self.prefix+'.track',self.track)


	def nv_validate(self):
		line = self.line.lstrip('#')
		if len(line)<20 or line[-9]!='*':
			self.logger.warning('invalid novatel log: '+line)
			return False

		ps = line.split(';')
		if len(ps)!=2 :
			self.logger.warning('invalid novatel log: '+line)
			return False
		self.header = ps[0]

		ps = ps[1].split('*')
		if len(ps)!=2 :
			self.logger.warning('invalid novatel log: '+line)
			return False
		self.content = ps[0]

		try:
			cksum = int(ps[1],16)
		except ValueError:
			self.logger.warning('invalid novatel cksum: '+ps[1])
			return False
		if cksum != novatel_crc32(self.header+';'+self.content):
			self.logger.warning('invalid novatel cksum: '+ps[1])
			return False	

		return True

if __name__ == "__main__":
    gps_url =  URL_BaseStation
    try:
        gps = GNSS(gps_url)
        gps.write(INIT_COMMANDS)
        t = PeriodTimer(0.2)
        t.start()
        aver_x = 0
        aver_y = 0
        aver_z = 0
        n = 1
        while n<300:
            with t:
                gps.update()
                if gps.posx != 0:
	                aver_x = 1.0*(n-1)/n*aver_x + 1.0/n*gps.posx
	                print(aver_x)
	                aver_y = 1.0*(n-1)/n*aver_y + 1.0/n*gps.posy
	                aver_z = 1.0*(n-1)/n*aver_z + 1.0/n*gps.posz
	                n += 1
        print('Fixed Position is:',aver_x,aver_y,aver_z)
        gps.write("UNLOGALL\r\n")
        FIX_COMMAND = "FIX POSITION" + " " + str(aver_x) + " " + str(aver_y) + " " + str(aver_z) + "\r\n"
        gps.write(FIX_COMMAND)
        gps.write(BS_COMMANDS)
    except (KeyboardInterrupt, Exception, GNSS_ERROR) as e:
        gps.close()
        raise
    finally:
    	gps.close()




