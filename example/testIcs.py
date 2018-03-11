#!/usr/bin/env python
#coding: utf-8

import icsCtrl
import time



if __name__ == '__main__':
	aaa = icsCtrl.IcsClass()
	aaa.begin("/dev/ttyUSB1",115200)

	for id in range(31):
		hoge = aaa.setPos(id, 5000)
		if(hoge is not False):
			print id
	for id in range(31):
		hoge = aaa.setPos(id, 10000)
		if(hoge is not False):
			print id