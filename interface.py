from msgdev import MsgDevice,PeriodTimer
from math import atan2, sqrt, pi
from PID import PID

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

	def Motor_send(self,left_motor,right_motor):
		self.dev.pub_set1('pro.left.speed',left_motor)
		self.dev.pub_set1('pro.right.speed',right_motor)


def ship_initialize(USE_TLG001, USE_TLG002):
	if USE_TLG001:
		sub_addr1 = 'tcp://192.168.1.150'
		ahrs_port1 = '55005'
		gnss_port1 = '55004'
		motor_port1 = '55002'
		interface001 = Interface(sub_addr1, ahrs_port1, gnss_port1, motor_port1)
	else:
		interface001 = 0

	if USE_TLG002:
		sub_addr2 = 'tcp://192.168.1.152'
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
	dx=target[POS_X] - self[POS_X]
	dy=target[POS_Y] - self[POS_Y]
	target_angle = atan2(dy, dx)
	dist = (dx ** 2 + dy ** 2) ** .5

	return target_angle, dist


def rpm_limit(rpm):
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

	t=PeriodTimer(0.1)
	t.start()
	try:
		while True:
			with t:
				self_state=interface001.receive('gps.posx', 'gps.posy', 'ahrs.yaw', 'ahrs.yaw_speed', 'gps_hspeed')
				target_state=interface002.receive('gps.posx', 'gps.posy')

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
				print('posx:{},posy:{},yaw:{},left:{},right:{}'.format(self_state[POS_X], self_state[POS_Y],
																		self_state[YAW], left_motor, right_motor))

				interface001.Motor_send(left_motor, right_motor)
	except (KeyboardInterrupt, Exception) as e:
		interface001.dev.close()
		interface002.dev.close()
		print('dev closed')
		raise
	finally:
		pass


if __name__=="__main__":
	main()