from msgdev import MsgDevice, PeriodTimer
import numpy as np


if __name__=="__main__":
	dev=MsgDevice()
	dev.open()
	dev.pub_bind('tcp://0.0.0.0:55005')

	data = np.loadtxt('log-05-05-17-07.txt', skiprows=1, usecols=(3,4), delimiter=',')
	i = 0
	t=PeriodTimer(0.1)
	t.start()
	try:
		while True:
			with t:
				dev.pub_set1('ahrs.yaw', data[i,0])
				dev.pub_set1('ahrs.yaw_speed', data[i,1])
				print('ahrs.yaw', data[i,0], 'ahrs.yaw_speed', data[i,1])
				i += 1
				if i == len(data): i = 0
	except (KeyboardInterrupt, Exception) as e:
		print('closed')
	finally:
		pass