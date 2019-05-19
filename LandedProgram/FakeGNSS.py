from msgdev import MsgDevice, PeriodTimer
import numpy as np

# 0 "timestamp",
# 1'gps.posx', 2'gps.posy', 3'ahrs.yaw', 4'ahrs.yaw_speed', 5'gps.hspeed',6'gps.stdx', 7'gps.stdy', 8'gps.track',
# 9'target.posx', 10'target.posy', 11'target.yaw', 12'target.yaw_speed',
#          13'target.hspeed', 14'target.stdx', 15'target.stdy', 16'target.track',
# 17'distance', 18'left_motor', 19 'right_motor'

if __name__=="__main__":
	dev=MsgDevice()
	dev.open()
	#dev.pub_bind('tcp://0.0.0.0:55004') #TLG001
	dev.pub_bind('tcp://0.0.0.0:55204') #TLG002
	data = np.loadtxt('log-05-05-17-21.txt', skiprows=1, usecols=(9, 10, 13, 16), delimiter=',')
	i = 0
	t = PeriodTimer(0.1)
	t.start()
	try:
		while True:
			with t:
				dev.pub_set1('gps.posx', data[i, 0])
				dev.pub_set1('gps.posy', data[i, 1])
				dev.pub_set1('gps.hspeed', data[i, 2])
				dev.pub_set1('gps.track', data[i, 3])
				print('gps.posx', data[i,0], 'gps.posy', data[i,1], 'gps.hspeed', data[i,2], 'gps.track', data[i,3])
				i += 1
				if i == len(data):
					i = 0
	except (KeyboardInterrupt, Exception) as e:
		print('closed')
	finally:
		pass
