#!/usr/bin/env python
#coding: utf-8

import b3mCtrl
import icsCtrl
import time



if __name__ == '__main__':
	aaa = b3mCtrl.B3mClass()

	# aaa.begin("/dev/ttyUSB0",3000000)

	aaa.begin("/dev/ttyUSB1",1500000)
	newId=1

	for id in [0]:
		# print aaa.setRam(id, newId, "ID")
		# time.sleep(0.5)
		# print aaa.saveCmd(id)
		# time.sleep(1)

		print aaa.setMode(id,"FREE")
		print aaa.setTrajectoryType(id,"EVEN")
		print aaa.setMode(id,"POSITION")
		print aaa.positionCmd(id,32000,0)
		time.sleep(3)
		print aaa.positionCmd(id,-32000,0)
		time.sleep(2.5)
		print "end"

