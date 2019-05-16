from msgdev import MsgDevice, PeriodTimer
import numpy as np


if __name__=="__main__":
	dev=MsgDevice()
	dev.open()
	#dev.pub_bind('tcp://0.0.0.0:55004') #TLG001
	dev.pub_bind('tcp://0.0.0.0:55204') #TLG002
	data = np.loadtxt('log-05-05-17-07.txt', skiprows=1, usecols=(1,2,5,8), delimiter=',')
	i = 0
	t=PeriodTimer(0.1)
	t.start()
	try:
		while True:
			with t:
				dev.pub_set1('gps.posx', data[i,0])
				dev.pub_set1('gps.posy', data[i,1])
				dev.pub_set1('gps.hspeed', data[i,2])
				dev.pub_set1('gps.track', data[i,3])
				print('gps.posx', data[i,0], 'gps.posy', data[i,1], 'gps.hspeed', data[i,2], 'gps.track', data[i,3])
				i += 1
				if i == len(data): i = 0
	except (KeyboardInterrupt, Exception) as e:
		print('closed')
	finally:
		pass
