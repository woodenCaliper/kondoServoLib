#!/usr/bin/env python
#coding: utf-8

import b3mCtrl
import icsCtrl
import time



if __name__ == '__main__':
	aaa = icsCtrl.IcsClass()


	aaa.begin("/dev/ttyUSB0")
	newId=21
	for id in range(32):
		print id
		# print aaa.idCmd(newId,"W")
		time.sleep(0.5)
		print aaa.setPos(id, 1000)
		time.sleep(0.5)
		print aaa.setPos(id, 000)
