from msgdev import MsgDevice, PeriodTimer

if __name__=="__main__":
	dev=MsgDevice()
	dev.open()
	dev.pub_bind('tcp://0.0.0.0:55004')
	t=PeriodTimer(0.1)
	t.start()
	while True:
		with t:
			dev.pub_set1('gps.posx', 10.11111)
			dev.pub_set1('gps.posy', 20.22222)
			dev.pub_set1('gps.posz', 30.33333)
			dev.pub_set1('gps.stdx', 40.44444)
			dev.pub_set1('gps.stdy', 50.55555)
			dev.pub_set1('gps.stdz', 60.66666)
			dev.pub_set1('gps.hspeed', 70.77777)
			dev.pub_set1('gps.vspeed', 80.88888)
			dev.pub_set1('gps.track', 90.99999)
			print('GNSS')

