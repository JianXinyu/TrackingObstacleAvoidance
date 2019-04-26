from msgdev import MsgDevice, PeriodTimer
from math import atan2
from PID import PID
import time
import pygame
import logging
from logging.handlers import RotatingFileHandler

class Interface(object):

	def __init__(self,sub_addr,ahrs_port,gnss_port,motor_port):
		self.dev=MsgDevice()
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
		self.dev.sub_add_url('gps.time')
		self.dev.sub_add_url('gps.posx')
		self.dev.sub_add_url('gps.posy')
		self.dev.sub_add_url('gps.posz')
		self.dev.sub_add_url('gps.stdx')
		self.dev.sub_add_url('gps.stdy')
		self.dev.sub_add_url('gps.stdz')
		self.dev.sub_add_url('gps.satn')
		self.dev.sub_add_url('gps.hspeed')
		self.dev.sub_add_url('gps.vspeed')
		self.dev.sub_add_url('gps.track')

		self.dev.pub_bind('tcp://0.0.0.0:'+motor_port)


	def receive(self,*args):
		data=[]
		for i in args:
			data.append(self.dev.sub_get1(i))
		return data

	def Motor_send(self,left_motor, right_motor, autoctrl):
		self.dev.pub_set1('pro.left.speed',left_motor)
		self.dev.pub_set1('pro.right.speed',right_motor)
		self.dev.pub_set1('js.autoctrl', autoctrl)


def ship_initialize(USE_TLG001, USE_TLG002):
	if USE_TLG001:
		sub_addr1 = 'tcp://127.0.0.1'  #'tcp://192.168.1.150'
		ahrs_port1 = '55005'
		gnss_port1 = '55004'
		motor_port1 = '55002'
		interface001 = Interface(sub_addr1, ahrs_port1, gnss_port1, motor_port1)
	else:
		interface001 = 0

	if USE_TLG002:
		sub_addr2 = 'tcp://127.0.0.1'  # 'tcp://192.168.1.152'
		ahrs_port2 = '55205'
		gnss_port2 = '55204'
		motor_port2 = '55202'
		interface002 = Interface(sub_addr2, ahrs_port2, gnss_port2, motor_port2)
	else:
		interface002 = 0
	return interface001, interface002


# 下标宏定义
POS_X      = 0
POS_Y      = 1
YAW         = 2
YAW_SPEED   = 3
SPD         = 4
SPD_DIR     = 5

# PID parameters
kp = 1000
ki = 3
kd = 10

# baseline = 1000
baseline = [800, 1200, 1400]

# propeller parameter
rpm_normal_left = [800, 1200, 1400]
rpm_normal_right = [850, 1250, 1450]
rpm_max = 2000


def pure_pursuit(self, target):
	dx = target[POS_X] - self[POS_X]
	dy = target[POS_Y] - self[POS_Y]
	target_angle = atan2(dy, dx)
	dist = (dx ** 2 + dy ** 2) ** .5
	return target_angle, dist


def rpm_limit(rpm):
	# min(1, max(-1, x))
	if rpm>rpm_max:
		return rpm_max
	elif rpm<-rpm_max:
		return -rpm_max
	else:
		return rpm

# TODO 丢包处理
# TODO 斜坡函数
def main():
	# initialize
	interface001, interface002 = ship_initialize(True, True)
	pid = PID(kp=kp, ki=ki, kd=kd, minout=-2000, maxout=2000, sampleTime=0.1)

	speed_limit = 1200
	pygame.init()
	pygame.joystick.init()

	logger = logging.getLogger(__name__)
	logger.setLevel(level=logging.INFO)

	rHandler = RotatingFileHandler("log-{:s}.txt".format(time.strftime("%m-%d-%H-%M", time.localtime())), \
								   maxBytes=100 * 1024 * 1024, backupCount=10)
	rHandler.setLevel(logging.INFO)
	formatter = logging.Formatter('%(message)s')
	rHandler.setFormatter(formatter)

	console = logging.StreamHandler()
	console.setLevel(logging.INFO)
	console.setFormatter(formatter)


	import csv
	csv_file = csv.writer(open("log-{:s}.txt".format(time.strftime("%m-%d-%H-%M", time.localtime())), 'w'))



	logger.addHandler(rHandler)
	logger.addHandler(console)

	t=PeriodTimer(0.1)
	t.start()
	try:
		while True:
			with t:
				self_state=interface001.receive('gps.posx', 'gps.posy', 'ahrs.yaw', 'ahrs.yaw_speed', 'gps.hspeed',
												'gps.stdx', 'gps.stdy', 'gps.track')
				target_state=interface002.receive('gps.posx', 'gps.posy', 'ahrs.yaw', 'ahrs.yaw_speed', 'gps.hspeed',
												'gps.stdx', 'gps.stdy', 'gps.track')

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
				if state >= 0:
					autoctrl = True
				else:
					autoctrl = False

				if autoctrl:
					target_angle, dist = pure_pursuit(self_state, target_state)
					output = pid.compute(self_state[YAW], target_angle)
					output = 0 if abs(output) < 5 else output
					if dist <= 3:
						average = baseline[0]
					elif dist <= 10:
						average = baseline[1]
					else:
						average = baseline[2]
					left_motor, right_motor = average + output / 2, average - output / 2
					left_motor = rpm_limit(left_motor)
					right_motor = rpm_limit(right_motor)

				elif autoctrl == False:
					averspeed = -aver * speed_limit
					diffspeed = 0.5 * diff * speed_limit
					left_motor = -(averspeed + diffspeed)
					right_motor = averspeed - diffspeed

					if left_motor > speed_limit:
						left_motor = speed_limit
					if left_motor < -speed_limit:
						left_motor = -speed_limit
					if right_motor > speed_limit:
						right_motor = speed_limit
					if right_motor < -speed_limit:
						right_motor = -speed_limit

				print('self ship state: ', 'posx:{},posy:{},yaw:{},speed:{},left:{},right:{}'.format(self_state[POS_X],
										self_state[POS_Y], self_state[YAW], self_state[SPD], left_motor, right_motor))
				print('target ship state: ', 'posx:{}, posy: {}，distance:{}'.format(target_state[POS_X],
																				target_state[POS_Y], dist))

				interface001.Motor_send(left_motor, right_motor)

			#	isinstance(csv_file, csv.DictWriter)
				csv_file.writerow(self_state+target_state+[dist, autoctrl])

				# logger.info('{0:.4f} {1:.4f} {2:.4f} {3:.4f} {4:.4f} {5:.4f} {6:.4f} {7:.4f} {8:.4f} {9:.4f} {10:.4f} '
				# 			'{11:.4f} {12:.4f} {13:.4f} {14:.4f} {15:.4f} {16:.4f} {17:.4f} '.format(self_state[0],
				# 										self_state[1], self_state[2], self_state[3], self_state[4],
				# 									   self_state[5], target_state[0], target_state[1], target_state[2],
				# 									   target_state[3], target_state[4], target_state[5], dist, autoctrl))
	except (KeyboardInterrupt, Exception) as e:
		pass
	finally:
		interface001.Motor_send(0, 0)
		interface001.dev.close()
		interface002.dev.close()
		csv_file.closer()
		print('dev closed')


if __name__=="__main__":
	main()