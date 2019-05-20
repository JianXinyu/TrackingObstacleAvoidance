from msgdev import MsgDevice,PeriodTimer
import pyqtgraph as pg
from collections import deque
from PID import PID

class Interface_rec(object):
    def __init__(self, sub_addr, gnss_port):
        self.dev = MsgDevice()
        self.dev.open()
        self.dev.sub_connect(sub_addr + ':' + gnss_port)
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

    def receive(self, *args):
        data = []
        for i in args:
            data.append(self.dev.sub_get1(i))
        return data


class Interface_msg_pub(object):
    def __init__(self, sub_addr, ahrs_port, gnss_port, motor_port):
        self.dev = MsgDevice()
        self.dev.open()
        self.dev.sub_connect(sub_addr + ':' + ahrs_port)
        self.dev.sub_add_url('ahrs.roll')
        self.dev.sub_add_url('ahrs.pitch')
        self.dev.sub_add_url('ahrs.yaw')
        self.dev.sub_add_url('ahrs.roll_speed')
        self.dev.sub_add_url('ahrs.pitch_speed')
        self.dev.sub_add_url('ahrs.yaw_speed')
        self.dev.sub_add_url('ahrs.acce_x')
        self.dev.sub_add_url('ahrs.acce_y')
        self.dev.sub_add_url('ahrs.acce_z')

        self.dev.sub_connect(sub_addr + ':' + gnss_port)
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

        self.dev.pub_bind('tcp://0.0.0.0:' + motor_port)

    def receive(self, *args):
        data = []
        for i in args:
            data.append(self.dev.sub_get1(i))
        return data

    def Motor_send(self, left_motor, right_motor):
        self.dev.pub_set1('pro.left.speed', left_motor)
        self.dev.pub_set1('pro.right.speed', right_motor)


PID_Tuner = False
pid_spd = PID(kp=3000.0, ki=100.0, kd=0, minout=0, maxout=2000, sampleTime=0.1)  # TODO
pid_yawspd = PID(kp=5000, ki=10.0, kd=0, minout=-1200, maxout=1200, sampleTime=0.1)
pid_yaw = PID(kp=300, ki=3, kd=10, minout=-1200, maxout=1200, sampleTime=0.1)

if __name__=="__main__":

	try:

		# 本船
		sub_addr = 'tcp://192.168.1.150'
		ahrs_port = '55005'
		gnss_port = '55004'
		if PID_Tuner:
			motor_port ='55002'
			interface001 = Interface_msg_pub(sub_addr, ahrs_port, gnss_port, motor_port)
		else:
			interface001 = Interface_rec(sub_addr, gnss_port)

		# 被跟踪船
		sub_addr = 'tcp://127.0.0.1'  # 'tcp://192.168.1.152'
		gnss_port = '55205'
		target = Interface_rec(sub_addr, gnss_port)

		# 障碍船1
		sub_addr = 'tcp://127.0.0.1'
		gnss_port = '55305'
		obstacle1 = Interface_rec(sub_addr, gnss_port)

		# 障碍船2
		sub_addr = 'tcp://127.0.0.1'
		gnss_port = '55405'
		obstacle2 = Interface_rec(sub_addr, gnss_port)

		app = pg.mkQApp()

		win = pg.GraphicsWindow()
		win.setWindowTitle(u'Sensor Monitor')
		win.resize(1600, 800)

		# CoorAxis = win.ViewBox()
		# CoorAxis.disableAutoRange()
		p1 = win.addPlot(title='Pos', row=0, col=0, labels={'left':'posx','bottom':'posy'})
		p1.showGrid(x=True, y=True)
		curve1 = p1.plot(pen=(255, 0, 0), name="Red curve")
		point0 = p1.plot(symbol='s')
		point1 = p1.plot(symbol='o')
		point2 = p1.plot(symbol='o')

		# point1 = p1.plot(pen=(0, 255, 0), symbol='o')

		p2 = win.addPlot(title='yaw', row=0, col=1, labels={'left': 'yaw(rad)', 'bottom': 'time(0.1s)'})
		p2.showGrid(x=True, y=True)
		curve2 = p2.plot(pen=(0, 255, 0), name="Green curve")

		p3 = win.addPlot(title='yaw speed', row=1, col=0, labels={'left': 'yaw speed(rad/s)', 'bottom': 'time(0.1s)'})
		p3.showGrid(x=True, y=True)
		curve3 = p3.plot(pen=(0, 255, 0), name="Green curve")

		p4 = win.addPlot(title='speed', row=1, col=1, labels={'left': 'speed(m/s)', 'bottom': 'time(0.1s)'})
		p4.showGrid(x=True, y=True)
		curve4 = p4.plot(pen=(0, 0, 255), name="Blue curve")

		cnt = 0
		data_rec = deque(maxlen=100)
		target_rec = deque(maxlen=1)
		ob1_rec = deque(maxlen=1)
		ob2_rec = deque(maxlen=1)
		count = deque(maxlen=100)

		def update():
			global cnt
			# 本船
			# tmp = interface001.receive('gps.posx', 'gps.posy', 'ahrs.yaw', 'gps.hspeed', 'ahrs.yaw_speed')
			# data_rec.append(tmp)
			# 被跟踪船
			tmp = target.receive('gps.posx', 'gps.posy')
			target_rec.append(tmp)
			# obstacle 1
			tmp = obstacle1.receive('gps.posx', 'gps.posy')
			ob1_rec.append(tmp)
			# obstacle 2
			tmp = obstacle2.receive('gps.posx', 'gps.posy')
			ob2_rec.append(tmp)

			count.append(cnt)
			# curve1.setData([-d[1] for d in data_rec], [-d[0] for d in data_rec])
			point0.setData([-target_rec[-1][1]], [-target_rec[-1][0]])
			point1.setData([-ob1_rec[-1][1]], [-ob1_rec[-1][0]])
			point2.setData([-ob2_rec[-1][1]], [-ob2_rec[-1][0]])
			# point1.setData([data_rec[-1][1]], [data_rec[-1][0]])
			# curve2.setData(count, [d[2] for d in data_rec])
			# curve3.setData(count, [d[4] for d in data_rec])
			# curve4.setData(count, [d[3] for d in data_rec])
			cnt += 1

		timer = pg.QtCore.QTimer()
		timer.timeout.connect(update)
		timer.start(100)

		app.exec_()

	except (KeyboardInterrupt, Exception) as e:
		print('closed')
	finally:
		pass