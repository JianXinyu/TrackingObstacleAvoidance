# default_values=(0,)
# print (default_values)
# print (type(default_values))
# import array
# def a2b_little(arr):
# 	return array.array('d', arr).tostring()

# def b2a_little(bytes):
# 	return array.array('d', bytes)


# value=a2b_little(default_values)

# print(b2a_little(value)[0])
# print(len(17.0))

from msgdev import MsgDevice,PeriodTimer
from math import sin,cos

if __name__=="__main__":
	dwa = MsgDevice()
	dwa.open()
	dwa.pub_bing('tcp://0.0.0.0:55010')
	interval = 0.3
	t = PeriodTimer(interval)
	t.start()
	try:
		while True:
			with t:


	except KeyboardInterrupt:
		print('close')
		dwa.dev.close()


