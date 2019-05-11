# -*- coding: utf-8 -*
import serial
import serial.tools.list_ports

port_list = list(serial.tools.list_ports.comports())
i=0
if len(port_list) <= 0:
	print('a')
else:
	while i<len(port_list):
		port_list_0 =list(port_list[i])
		port_serial = port_list_0[0]
	#port_serial=port_list[0]
		ser = serial.Serial(port_serial,9600,timeout = 60)
		print ("check which port was really used >",ser.name)
		#print(port_list)
		#print(port_list_0)
		print(port_serial)
		i=i+1