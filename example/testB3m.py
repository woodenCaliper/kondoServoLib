#!/usr/bin/env python
#coding: utf-8

import b3mCtrl
import time



if __name__ == '__main__':
	aaa = b3mCtrl.B3mClass()
	aaa.begin("/dev/ttyUSB0",1500000)

	idx= [3]
	for id in idx:
		print aaa.setMode(id,"FREE")
		print aaa.setTrajectoryType(id,"EVEN")
		print aaa.setMode(id,"POSITION")

		hoge = aaa.positionCmd(id, 0)
		if(hoge is not False):
			print id

	time.sleep(1)
	for id in idx:
		hoge = aaa.positionCmd(id, 18000)
		if(hoge is not False):
			print id