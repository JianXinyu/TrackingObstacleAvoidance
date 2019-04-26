from msgdev import MsgDevice, PeriodTimer

if __name__=="__main__":
	dev=MsgDevice()
	dev.open()
	dev.pub_bind('tcp://0.0.0.0:55005')
	t=PeriodTimer(0.1)
	t.start()
	while True:
		with t:
			dev.pub_set1('ahrs.roll', 1.11111)
			dev.pub_set1('ahrs.pitch', 2.22222)
			dev.pub_set1('ahrs.yaw', 3.33333)
			dev.pub_set1('ahrs.roll_speed', 4.44444)
			dev.pub_set1('ahrs.pitch_speed', 5.55555)
			dev.pub_set1('ahrs.yaw_speed', 6.66666)
			dev.pub_set1('ahrs.acce_x', 7.77777)
			dev.pub_set1('ahrs.acce_y', 8.88888)
			dev.pub_set1('ahrs.acce_z', 9.99999)
			print('ahrs')
