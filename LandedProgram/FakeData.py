from msgdev import MsgDevice, PeriodTimer
import numpy as np

# 0 "timestamp",
# 1'gps.posx', 2'gps.posy', 3'ahrs.yaw', 4'ahrs.yaw_speed', 5'gps.hspeed',6'gps.stdx', 7'gps.stdy', 8'gps.track',
# 9'target.posx', 10'target.posy', 11'target.yaw', 12'target.yaw_speed',
#          13'target.hspeed', 14'target.stdx', 15'target.stdy', 16'target.track',
# 17'distance', 18'left_motor', 19 'right_motor'

USE_Fake_Target = True
USE_Fake_Obstacle1 = True
USE_Fake_Obstacle2 = True


if __name__=="__main__":
	if USE_Fake_Target:
		target = MsgDevice()
		target.open()
		target.pub_bind('tcp://0.0.0.0:55205')  #TLG002
		target_data = np.loadtxt('fakedata4.txt')  #, skiprows=1, usecols=(9, 10, 13, 16), delimiter=',')

	if USE_Fake_Obstacle1:
		ob1 = MsgDevice()
		ob1.open()
		ob1.pub_bind('tcp://0.0.0.0:55305')
		ob1_data = np.loadtxt('fakedata5.txt')

	if USE_Fake_Obstacle2:
		ob2 = MsgDevice()
		ob2.open()
		ob2.pub_bind('tcp://0.0.0.0:55405')
		ob2_data = np.loadtxt('fakedata6.txt')

	i = 0
	ii = 0
	iii = 0
	cnt = 0
	t = PeriodTimer(0.1)
	t.start()
	try:
		while True:
			with t:
				cnt += 1
				print('Step:', cnt)
				if USE_Fake_Target:
					target.pub_set1('gps.posx', target_data[i, 0])
					target.pub_set1('gps.posy', target_data[i, 1])
					# target.pub_set1('gps.hspeed', target_data[i, 2])
					# target.pub_set1('gps.track', target_data[i, 3])
					print('Target: gps.posx %.3f, gps.posy %.3f' %(target_data[i, 0], target_data[i, 1]))  #, 'gps.hspeed', target_data[i, 2], 'gps.track', target_data[i, 3])
					i += 1
					if i == len(target_data):
						i = 0

				if USE_Fake_Obstacle1:
					ob1.pub_set1('gps.posx', ob1_data[ii, 0])
					ob1.pub_set1('gps.posy', ob1_data[ii, 1])
					ob1.pub_set1('gps.hspeed', ob1_data[ii, 2])
					ob1.pub_set1('gps.track', ob1_data[ii, 3])
					print('Obstacle 1: gps.posx %.3f, gps.posy %.3f, gps.speed %.3f, gps.track %.3f' %(
						ob1_data[ii, 0],  ob1_data[ii,1],
						ob1_data[ii, 2],  ob1_data[ii, 3]))
					ii += 1
					if ii == len(ob1_data):
						ii = 0

				if USE_Fake_Obstacle2:
					ob2.pub_set1('gps.posx', ob2_data[iii, 0])
					ob2.pub_set1('gps.posy', ob2_data[iii, 1])
					ob2.pub_set1('gps.hspeed', ob2_data[iii, 2])
					ob2.pub_set1('gps.track', ob2_data[iii, 3])
					print('Obstacle 2: gps.posx %.3f, gps.posy %.3f, gps.speed %.3f, gps.track %.3f'
						  % (ob2_data[iii, 0], ob2_data[iii, 1],
						ob2_data[iii, 2], ob2_data[iii, 3]))
					iii += 1
					if iii == len(ob2_data):
						iii = 0


	except (KeyboardInterrupt, Exception) as e:
		print('closed')
	finally:
		pass
