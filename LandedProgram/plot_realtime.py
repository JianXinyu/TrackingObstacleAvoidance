from msgdev import MsgDevice,PeriodTimer
from interface import Interface
import pyqtgraph as pg

if __name__=="__main__":
	# sub_addr='tcp://192.168.1.150'
	sub_addr = 'tcp://127.0.0.1'
	ahrs_port = '55005'
	gnss_port = '55004'
	motor_port = '55002'
	interface=Interface(sub_addr, ahrs_port, gnss_port, motor_port)


	app = pg.mkQApp()

	win = pg.GraphicsWindow()
	win.setWindowTitle(u'Sensor Monitor')
	win.resize(1600, 800)


	p1 = win.addPlot(title='Pos',labels={'left':'posx','bottom':'posy'})
	p1.showGrid(x=True, y=True)
	curve1 = p1.plot(pen=(255, 0, 0), name="Red curve")

	p2 = win.addPlot(title='yaw',labels={'left':'yaw','bottom':'i'})
	p2.showGrid(x=True, y=True)
	curve2 = p2.plot(pen=(0, 255, 0), name="Green curve")
	# curve3 = p1.plot(pen=(0, 0, 255), name="Blue curve")
	i=0
	data=[]
	x=[]
	def update():
		global i
		tmp=interface.receive('gps.posx','gps.posy','ahrs.yaw')
		if len(data)<200:
			data.append(tmp)
			x.append(i)
		else:
			data[:-1] = data[1:]
			data[-1] = tmp
			x[:-1]=x[1:]
			x[-1]=i

		curve1.setData([d[1] for d in data],[d[0] for d in data])
		curve2.setData(x,[d[2] for d in data])
		i+=1
	timer = pg.QtCore.QTimer()
	timer.timeout.connect(update)
	timer.start(500)

	# if __name__ == "__main__":
	app.exec_()