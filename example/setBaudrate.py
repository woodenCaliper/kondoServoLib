#!/usr/bin/env python
#coding: utf-8

import b3mCtrl
import icsCtrl
import time



if __name__ == '__main__':
	aaa = b3mCtrl.B3mClass()

	# aaa.begin("/dev/ttyUSB0",3000000)

	aaa.begin("/dev/ttyUSB0",1500000)
	for id in [11,12,13,14,15,16]:
		print aaa.loadCmd(id)
		time.sleep(0.5)
		print aaa.setRam(id, 115200, "Baudrate")
		time.sleep(0.5)
		print aaa.saveCmd(id)
		time.sleep(1)

		# print aaa.setMode(id,"FREE")
		# print aaa.setTrajectoryType(id,"EVEN")
		# print aaa.setMode(id,"POSITION")
		# print aaa.positionCmd(id,0,1000)
		# time.sleep(1)

