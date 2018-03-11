#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
import time
class IcsClass(object):
	_MAX_ID=31
	_MIN_ID=0
	_MAX_POS = 11500
	_MIN_POS = 3500

	def __init__(self, _port=None, _baudrate=115200, _timeout=0.02):
		self.port = _port
		self.baudrate = _baudrate
		self.timeout = _timeout

	def __del__(self):
		self.ics.close()

	def begin(self, _port=None, _baudrate=None, _timeout=None):
		if (self.port == None) and (_port == None):
			return False
		elif _port != None:
			self.port = _port
		if _baudrate != None:
			self.baudrate=_baudrate
		if _timeout != None:
			self.timeout=_timeout

		import serial
		self.ics = serial.Serial(self.port, self.baudrate,  bytesize=serial.EIGHTBITS, parity=serial.PARITY_EVEN, stopbits=serial.STOPBITS_ONE, timeout=self.timeout)
		return True

	def __synchronize(self, txBuf, rxLen, tryNum=2):
		if self.port==None:
			return False
		for i in range(tryNum):
			# self.ics.reset_input_buffer()	#残存rxバッファを消す
			self.ics.read(self.ics.inWaiting())	#残存rxバッファを消す
			self.ics.write(bytearray(txBuf))
			self.ics.flush()
			# start = time.time()
			#0.015s程度
			rxBuf = self.ics.read(rxLen)	#一定数受信まで待機（timeoutはする）
			# print time.time() - start
			if len(rxBuf) == rxLen:
				break
			if len(rxBuf) != rxLen:
				print "ics rx signal error"
		if len(rxBuf) != rxLen:
			print "ics rx signal false"
			return False
		rxBuf = map(lambda x:ord(x),rxBuf)	#ASKIIを数値に変換
		return rxBuf
		return 0

	@staticmethod
	def __checkIdRange(id):
		return (IcsClass._MIN_ID <= id <= IcsClass._MAX_ID)

	@staticmethod
	def __checkPosRange(pos):
		return (IcsClass._MIN_POS <= pos <= IcsClass._MAX_POS)

	@staticmethod
	def degToPos(deg):
		pos = int(deg*(11500-3500)/270.0+7500)
		if (IcsClass.__checkPosRange(pos)==False):
			return False
		else:
			return pos

	@staticmethod
	def posToDeg(pos):
		if (IcsClass.__checkPosRange(pos)==False):
			return False
		return (pos-7500)*270.0/(11500-3500)

	@staticmethod
	def radToPos(rad):
		pos = (11500-3500)/(270.0/180.0)*(rad/math.pi)+7500
		if (IcsClass.__checkPosRange(pos)==False):
			return False
		return int(pos)

	@staticmethod
	def posToRad(pos):
		if (IcsClass.__checkPosRange(pos)==False):
			return False
		return (pos-7500)*(270.0/180.0)*math.pi/(11500-3500)

	def readCmd(self, id, subCmd):
		if IcsClass.__checkIdRange(id)==False:
			return False
		if not (subCmd in {"STRC", "SPD", "CUR", "TMP", "EEPROM", "TCH"}):
			return False
		rxLen=3
		if subCmd=="EEPROM":
			sc_ = 0x00
			rxLen=66
		elif subCmd=="STRC":
			sc_ = 0x01
		elif subCmd=="SPD":
			sc_ = 0x02
		elif subCmd=="CUR":
			sc_ = 0x03
		elif subCmd=="TMP":
			sc_ = 0x04
		elif subCmd=="TCH":	#only ICS3.6
			sc_ = 0x05
			rxLen=4
		txData = [0xA0+id, sc_]
		rxData = self.__synchronize(txData, rxLen)
		if rxData==False:
			return False
		return reData[2:]

	def writeCmd(self, id, subCmd, data):
		if IcsClass.__checkIdRange(id)==False:
			return False
		if not (subCmd in {"STRC", "SPD", "CURLIM", "TMPLIM", "EEPROM"}):
			return False
		if type(data)!=type(list()):
			data=[data]
		rxLen=3
		if subCmd=="EEPROM":
			if len(data)!=64:
				return False
			sc_ = 0x00
			rxLen=2
		elif subCmd=="STRC":
			if not (1 <= data[0] <= 127):
				return False
			sc_ = 0x01
		elif subCmd=="SPD":
			if not (1 <= data[0] <= 127):
				return False
			sc_ = 0x02
		elif subCmd=="CURLIM":
			if not (1 <= data[0] <= 63):
				return False
			sc_ = 0x03
		elif subCmd=="TMPLIM":
			if not (1 <= data[0] <= 127):
				return False
			sc_ = 0x04
		txData = [0xC0+id, sc_] + data
		rxData = self.__synchronize(txData, rxLen)
		if rxData==False:
			return False
		return rxData[2:]

	def idCmd(self, id, RorW):
		if IcsClass.__checkIdRange(id)==False:
			return False
		if RorW=="R":
			txData = [0xE0+id, 0x00, 0x00, 0x00]
		elif RorW=="W":
			txData = [0xE0+id, 0x01, 0x01, 0x01]
		else:
			return False
		rxData = self.__synchronize(txData, 1)
		return rxData[0]&0x1F

	def setPos(self, id, pos):
		if IcsClass.__checkIdRange(id)==False:
			print "id range error"
			return False
		if (IcsClass.__checkPosRange(pos)==False) and (pos!=0):
			print "pos range error"
			return False
		txData = [ 0x80+id, (pos>>7)&0x7F, pos&0x7F ]
		rxData = self.__synchronize(txData, 3)
		if (rxData == False):
			return False
		rePos =  ( (rxData[1]<<7) & 0x3f80 ) + ( rxData[2]&0x007f )
		return rePos

	def setFree(self, id):
		if IcsClass.__checkIdRange(id)==False:
			return False
		txData = [0x80+id, 0x00, 0x00]
		rxData = self.__synchronize(txData, 3)
		if rxData==False:
			return False
		rePos = ( (rxData[1]<<7) & 0x3f80 ) + ( rxData[2]&0x007f )
		return rePos

	def setStrc(self, id, strc):
		return self.writeCmd(id, "STRC", strc)

	def setSpd(self, id, spd):
		return self.writeCmd(id, "SPD", spd)

	def setCur(self, id, curLim):
		return self.writeCmd(id, "CURLIM", curLim)

	def setTmp(self, id, tmpLim):
		return self.writeCmd(id, "TMPLIM", tmpLim)

	def getStrc(self, id):
		return self.readCmd(id, "STRC")

	def getSpd(self, id):
		return self.readCmd(id, "SPD")

	def getCur(self, id):
		return self.readCmd(id, "CUR")

	def getTmp(self, id):
		return self.readCmd(id, "TMP")

	def getPos(self, id):	#only ICS3.6
		reBuf = self.readCmd(id, "TCH")
		if reBuf==False:
			return False
		return ((reBuf[0]<<7)&0x3F80) + (reBuf[1]&0x007F)

# aaa=IcsClass("/dev/ttyUSB0")
# aaa.begin()
# print aaa.setPos(1,7500)
