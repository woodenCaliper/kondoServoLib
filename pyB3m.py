#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math, time
import serial
import enum
import csv
import os
import warnings


class PyB3m(object):
	MAX_ID=255
	MIN_ID=0
	MAX_POS = 32000
	MIN_POS = -32000


	def __importMemoryMap():
		scriptPath = os.path.dirname(os.path.abspath(__file__))
		with open(scriptPath+'/b3mMemoryMap.csv','r') as memFile:
			memoryReader = csv.DictReader(memFile)
			stringForAddressName = ""
			MEMORY_MAP = [None]	#enumのvalueと列番号を対応付けるため、1列目にNoneを入れている
			for row in memoryReader:
				if row["remarks"] =="":
					row["address"] = int(row["address"],16)
					if not row["min"]=="":
						row["min"] = int(row["min"], 10)
					if not row["max"]=="":
						row["max"] = int(row["max"], 10)
					stringForAddressName += row["addressName"]+" "
					MEMORY_MAP.append(row)

			ADDRESS_NAME = enum.Enum("ADDRESS_NAME", stringForAddressName)
			return tuple(MEMORY_MAP), ADDRESS_NAME

	MEMORY_MAP, ADDRESS_NAME = __importMemoryMap()

	def __init__(self, port=None, baudrate=1500000, timeout=0.005):
		self.port = port
		self.baudrate = baudrate
		self.timeout = timeout

	def __del__(self):
		self.b3mSerial.close()

	@staticmethod
	def __toRowVector(L):	#多次元を１次元に直す
		if isinstance(L, list):
			if L == []:
				return []
			else:
				return PyB3m.__toRowVector(L[0]) + PyB3m.__toRowVector(L[1:])
		else:
			return [L]

	@staticmethod
	def degToPos(deg):
		pos = int(deg*(PyB3m.MAX_POS - PyB3m.MIN_POS)/640.0)
		if not PyB3m.__isInRange(pos, PyB3m.MAX_POS, PyB3m.MIN_POS):
			warnings.warn("degee range error")
			return False
		return pos
	@staticmethod
	def posToDeg(pos):
		if not PyB3m.__isInRange(pos, PyB3m.MAX_POS, PyB3m.MIN_POS):
			print "pos range error"
			return False
		return (pos-7500)*640.0/(PyB3m.MAX_POS-PyB3m.MIN_POS)
	@staticmethod
	def radToPos(rad):
		pos = (PyB3m.MAX_POS - PyB3m.MIN_POS)/(640.0/180.0)*(rad/math.pi)
		if not PyB3m.__isInRange(pos, PyB3m.MAX_POS, PyB3m.MIN_POS):
			print "rad range error"
			return False
		return int(pos)
	@staticmethod
	def posToRad(pos):
		if not PyB3m.__isInRange(pos, PyB3m.MAX_POS, PyB3m.MIN_POS):
			print "pos range error"
			return False
		return pos*(640.0/180.0)*math.pi/(PyB3m.MAX_POS-PyB3m.MIN_POS)
	@staticmethod
	def __assemblyByte(data):
		reData=0
		if type(data) is not list:
			return False
		for i in range(len(data))[::-1]:	#rangeの反転
			reData += data[i]<<(8*i)
		return reData
	@staticmethod
	def __disassemblyByte(data, byteLength):	#リトルエンディアン
		reData = []
		for i in range(byteLength):
			reData.append( data&0xFF )
			data = data>>8
		return reData
	@staticmethod
	def __unsignedToSigned(bit, byteLength):
		if bit >= 1<<(8*byteLength-1):
			return -( (0b1<<(8*byteLength)) - bit )
		return bit
	@staticmethod
	def __signedToUnsigned(num, byteLength):
		if (num<0):
			return (1<<(8*byteLength)) + num	#MSBを1に
		return num
	@staticmethod
	def __isInRange(val, max, min):	#valが以上以下に収まっているか
		val = PyB3m.__toRowVector(val)
		for i in range(len(val)):
			if (val[i]<min) or (max<val[i]):
				return False
		return True
	@staticmethod
	def __isCheckOption(option):
		return option in {"ERROR", "SYSTEM", "MOTOR", "UART", "COMMAND", "CLEAR"}
	@staticmethod
	def __optionToCmd(option):
		if option == "ERROR":
			return 0b000
		elif option=="SYSTEM":
			return 0b001
		elif option == "MOTOR":
			return 0b010
		elif option == "UART":
			return 0b011
		elif option == "COMMAND":
			return 0b100
		elif option == "CLEAR":
			return 0b10000000
		elif option == "NORMAL":
			return 0b00

	## 通信開始設定
	# @param _port(str) = デバイス名
	# @param _baudrate(int) = 通信速度[bps]
	# @param _timeout(float) = 受信待ちタイムアウト時間[sec]
	def begin(self, port=None, baudrate=None, timeout=None):
		self.port = port or self.port
		self.baudrate = baudrate or self.baudrate
		if timeout is not None:
			self.timeout = timeout

		self.gardTime = 2.0/self.baudrate + 220*(10**-6)	#2バイト分のデータ＋220us以上
		if self.timeout<self.gardTime:
			raise ValueError("timeout is shorter than gardTime")
		self.b3mSerial = serial.Serial(self.port, self.baudrate,  bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, timeout=self.timeout)

	def __synchronize(self, txBuf, rxLen):
		txBuf = [len(txBuf)+2] + txBuf
		txBuf += [sum(txBuf)&0xFF]

		self.b3mSerial.reset_input_buffer()	#残存rxバッファを消す
		self.b3mSerial.write(bytearray(txBuf))
		self.b3mSerial.flush()	#全送信するまで待機
		rxBuf=self.b3mSerial.read(rxLen)
		rxBuf = map(lambda x:ord(x), rxBuf)

		if len(rxBuf) == rxLen:
			return rxBuf
		else:
			warnings.warn("b3m rx signal error")
			return False

	## ROMをRAMに書き込み(シングル/マルチ)
	# @param id(int_or_list) = サーボID
	# @param option(str) = 返値オプション
	# @return 成功(tuple) = (True, ステータス)
	# @return 失敗(tuple) = (False, False)
	def loadCmd(self, id, option="ERROR"):
		if type(id) is not list:
			id = [id]
		if not PyB3m.__isInRange(id, PyB3m.MAX_ID, PyB3m.MIN_ID):
			raise ValueError("id is out of range. must change 0-255")
			# return (False,False)
		if not PyB3m.__isCheckOption(option):
			raise ValueError("option error")
			# return (False,False)
		txBuf = [0x01, PyB3m.__optionToCmd(option)] + id
		if len(id)==1:	#シングルモード
			rxBuf = self.__synchronize(txBuf, 5)
			if (rxBuf is False):
				return (False,False)
			reStatus = rxBuf[2]
			return (True, reStatus)
		else:	#マルチモード
			rxBuf = self.__synchronize(txBuf, 0)
			if (rxBuf is False):
				return (False,False)
			return (True, True)

	## RAMをROMに書き込み(シングル/マルチ)
	# @param id(int_or_list) = サーボID
	# @param option(str) = 返値オプション
	# @return 成功(tuple) = (True, True)
	# @return 失敗(tuple) = (False, False)
	def saveCmd(self, id, option="ERROR"):	#RAMをROMに書き込み
		if type(id) is not list:
			id = [id]
		if not PyB3m.__isInRange(id, PyB3m.MAX_ID, PyB3m.MIN_ID):
			raise ValueError("id is out of range. must change 0-255")
			# print "id range error"
			# return (False,False)
		if not PyB3m.__isCheckOption(option):
			raise ValueError("option error")
			# return (False,False)
		txBuf = [0x02, PyB3m.__optionToCmd(option)] + id
		if len(id)==1:	#シングルモード
			rxBuf = self.__synchronize(txBuf, 5)
			if (rxBuf is False):
				return (False,False)
			reStatus = rxBuf[2]
			return (True, reStatus)
		else:	#マルチモード
			rxBuf = self.__synchronize(txBuf, 0)
			if (rxBuf is False):
				return (False,False)
			return (True, True)

	## RAMの読み出し(シングル)
	# @param id(int) = サーボID
	# @param address(int) = 読み込みアドレス
	# @param length(int) = 読み込みデータ長さ
	# @param option(str) = 返値オプション
	# @return 成功(tuple) = (RAMデータ, ステータス)
	# @return 失敗(tuple) = (False, False)
	def readCmd(self, id, address, length, option="ERROR"):	#RAMの読み出し
		if type(id) is not list:
			id = [id]
		if not PyB3m.__isInRange(id, PyB3m.MAX_ID, PyB3m.MIN_ID):
			raise ValueError("id is out of range. must change 0-255")
			# print "id range error"
			# return (False,False)
		if not PyB3m.__isInRange(address, 0xFF, 0x00):
			raise ValueError("address is out of range.")
			# print "address range error"
			# return (False,False)
		if not PyB3m.__isInRange(length, 0xFA, 0x01):
			raise ValueError("length is out of range.")
			# print "length range error"
			# return (False,False)
		if not PyB3m.__isCheckOption(option):
			raise ValueError("option error")
			# return (False,False)
		txBuf = [0x03, PyB3m.__optionToCmd(option)] + id + [address, length]
		rxBuf = self.__synchronize(txBuf, length+5)
		if (rxBuf is False):
			print "rx error "
			return (False,False)
		reRam = rxBuf[4:-1]
		reStatus = rxBuf[2]
		return (reRam, reStatus)

	## RAMへの書き込み(シングル/マルチ)
	# @param id(int_or_list) = サーボID
	# @param address(int) = 書き込みアドレス
	# @param data(list) = 書き込みデータ(シングル)
	# @param data(list_of_list) = 書き込みデータ(マルチ) <br>
	# 例)data次元数 = idの次元数+1		id=[1,2,3]	data=[[10,20], [100,200], [1000,2000]]
	# @param option(str) = 返値オプション
	# @return シングル成功(tuple) = (RAMデータ, ステータス)
	# @return マルチ成功(tuple) = (True, True)
	# @return 失敗(tuple) = (False, False)
	def writeCmd(self, id, address, data, option="ERROR"):	#RAMへの書き込み
		if type(id) is not list:
			id = [id]
		if type(data) is not list:
			data=[data]
		if not PyB3m.__isInRange(id, PyB3m.MAX_ID, PyB3m.MIN_ID):
			raise ValueError("id is out of range. must change 0-255")
			# print "id range error"
			# return (False,False)
		if not PyB3m.__isInRange(address, 0xFF, 0x00):
			raise ValueError("address is out of range.")
			# print "address range error"
			# return (False,False)
		if not PyB3m.__isInRange(data, 0xFF, 0x00):
			raise ValueError("data is out of range")
		# 	print "data range error"
		# 	return (False,False)
		# if not PyB3m.__isCheckOption(option):
			raise ValueError("option error")
			# return (False,False)
		if len(id)==1:	#シングル
			txBuf = [0x04, PyB3m.__optionToCmd(option), id[0]] + PyB3m.__toRowVector(data) + [address, 0x01]
			rxBuf = self.__synchronize(txBuf, 5)
			if (rxBuf is False):
				return (False,False)
			reStatus = rxBuf[2]
			return (True, reStatus)
		else:	#マルチ
			if (len(id)!=len(data)):
				return (False,False)
			id_data = []
			for i in range(len(id)):
				id_data += [id[i]] + data[i]
			txBuf = [0x04] + [ PyB3m.__optionToCmd(option)] + id_data + [address] + [len(id)]
			rxBuf = self.__synchronize(txBuf, 0)
			if (rxBuf is False):
				return (False,False)
			return (True,True)

	## 再起動要求(シングル/マルチ)
	# @param id(int_or_list) = サーボID
	# @param time(float) = 再起動遅延時間[sec]
	# @param option(str) = 返値オプション
	# @return 成否(bool)
	def resetCmd(self, id, time=0, option="ERROR"):
		time = int(time*100)
		if type(id) is not list:
			id = [id]
		if not PyB3m.__isInRange(id, PyB3m.MAX_ID, PyB3m.MIN_ID):
			raise ValueError("id is out of range. must change 0-255")
			# print "id range error"
			# return False
		if not PyB3m.__isInRange(time, 25500, 0):
			raise ValueError("time is out of range.")
			# print "time range error"
			# return False
		if not PyB3m.__isCheckOption(option):
			raise ValueError("option error")
			# return (False,False)
		txBuf = [0x05, PyB3m.__optionToCmd(option)] + id + [time&0xFF, time>>8]
		rxBuf = self.__synchronize(txBuf, 0)
		if (rxBuf is False):
			return False
		return True

	## 位置設定(シングル/マルチ)
	# @param id(int_or_list) = サーボID
	# @param pos(float_or_list) = 目標位置[rad]
	# @param time(float) = 移動完了目標時間[sec]
	# @param option(str) = 返値オプション
	# @return 成否(bool)
	def positionCmd(self, id, pos, time=0, option="ERROR"):
		time = int(time*1000)	#s → ms
		if type(id) is not list:
			id = [id]
		if type(pos) is not list:
			pos = [pos]
		if not PyB3m.__isInRange(id, PyB3m.MAX_ID, PyB3m.MIN_ID):
			raise ValueError("id is out of range. must change 0-255")
			# print "id range error"
			# return (False,False)
		if not PyB3m.__isInRange(pos, PyB3m.MAX_POS, PyB3m.MIN_POS):
			raise ValueError("pos is out of range.")
			# print "position range error"
			# return (False,False)
		if not PyB3m.__isInRange(time, 655335, 0):
			raise ValueError("time is out of range.")
			# print "time range error"
			# return (False,False)
		if (len(id) != len(pos)):
			raise ValueError("length of id and pos is not same.")
			# print "id and pos length error"
			# return (False,False)
		if not PyB3m.__isCheckOption(option):
			raise ValueError("option error")
			# return (False,False)

		id_pos=[]
		for i in range(len(id)):
			id_pos += [id[i]]
			pos[i] = PyB3m.__signedToUnsigned(pos[i], 2)
			id_pos +=  PyB3m.__disassemblyByte(pos[i], 2)
		txBuf = [0x06, PyB3m.__optionToCmd(option)] + id_pos + PyB3m.__disassemblyByte(int(time),2)
		if len(id)==1:	#シングルモード
			rxBuf = self.__synchronize(txBuf, 7)
			# rxBuf = self.__unsynchronize(txBuf, 7)
			if (rxBuf is False):
				return (False,False)
			reStatus = rxBuf[2]
			rePos = PyB3m.__assemblyByte(rxBuf[4:6])
			rePos = PyB3m.__unsignedToSigned(rePos, 2)
			return rePos, reStatus
		else:	#マルチモード
			rxBuf = self.__synchronize(txBuf, 0)
			if (rxBuf is False):
				return (False,False)
			return (True,True)

#非コマンド>>>
	## モードを設定
	# @param id(int) = サーボID
	# @param mode(str) = モード
	def setMode(self, id, mode, option="ERROR"):
		address = 0x28
		if not mode in {"POSITION", "SPEED", "VELOCITY", "CURRENT", "TORQUE", "FEEDFORWARD", "NORMAL", "FREE", "HOLD"}:
			return False
		if mode == "NORMAL":
			data = 0b0000
		elif mode == "FREE":
			data = 0b0010
		elif mode == "HOLD":
			data = 0b0011
		else:
			re = self.setMode(id, "HOLD")
			if re is False:
				return False
			if mode == "POSITION":	#以下は"NORMAL"も同時に書き込んでいる
				data = 0b0000
			elif mode == "SPEED" or  mode == "VELOCITY":
				data = 0b0100
			elif mode == "CURRENT" or mode == "TORQUE":
				data = 0b1000
			elif mode == "FEEDFORWARD":
				data = 0b1100

		if type(id) is not list:	#シングルモード
			rxBuf = self.writeCmd(id, address, data, option)
		elif len(id)==1:					#シングルモード
			rxBuf = self.writeCmd(id[0], address, data[0], option)
		else:								#マルチモード
			rxBuf = self.writeCmd([id]*len(id), address, [[data]*len(id)], option)
		return rxBuf

	def setTrajectoryType(self, id, _type, option="ERROR"):
		address = 0x29
		if _type == "NORMAL":
			data = 0
		elif _type == "EVEN":
			data = 1
		elif _type == "THIRDPOLY":
			data = 3
		elif _type == "FORTHPOLY":
			data = 4
		elif _type == "FIFTHPOLY":
			data = 5
		else:
			return False
		if type(id) is not list:	#シングルモード
			rxBuf = self.writeCmd(id, address, data, option)
		elif len(id)==1:					#シングルモード
			rxBuf = self.writeCmd(id[0], address, data[0], option)
		else:								#マルチモード
			rxBuf = self.writeCmd([id]*len(id), address, [[data]*len(id)], option)
		return rxBuf	#Status

	def setRam(self, id, data, addressName, option="ERROR"):
		if not type(addressName) is type(PyB3m.ADDRESS_NAME):
			print "addressName type error"
			return False,False
		memoryPath = addressName.value

		if not PyB3m.__isInRange(data, PyB3m.MEMORY_MAP[memoryPath]["min"] , PyB3m.MEMORY_MAP[memoryPath]["max"]):
			print "data range error"
			return (False, False)
		address = PyB3m.MEMORY_MAP[memoryPath]["address"]
		if PyB3m.MEMORY_MAP[memoryPath]["type"] == "char":
			data = PyB3m.__signedToUnsigned(data, 1)	#符号付き→符号なしに変換
			txData = PyB3m.__disassemblyByte(data, 1)	#数値を1byteづつのリストに変換
		elif PyB3m.MEMORY_MAP[memoryPath]["type"] == "byte":
			txData = PyB3m.__disassemblyByte(data, 1)
		elif PyB3m.MEMORY_MAP[memoryPath]["type"] == "short":
			data = PyB3m.__signedToUnsigned(data, 2)
			txData = PyB3m.__disassemblyByte(data, 2)
		elif PyB3m.MEMORY_MAP[memoryPath]["type"] == "ushort":
			txData = PyB3m.__disassemblyByte(data, 2)
		elif PyB3m.MEMORY_MAP[memoryPath]["type"] == "long":
			data = PyB3m.__signedToUnsigned(data, 4)
			txData = PyB3m.__disassemblyByte(data, 4)
		elif PyB3m.MEMORY_MAP[memoryPath]["type"] == "ulong":
			txData = PyB3m.__disassemblyByte(data, 4)
		(Success, reStatus) = self.writeCmd(id, address, txData, option)
		return (Success, reStatus)

	def getRam(self, id, addressName, option="ERROR"):
		if not type(addressName) is type(PyB3m.ADDRESS_NAME):
			print "addressName type error"
			return (False,False)
		memoryPath = addressName.value

		address = PyB3m.MEMORY_MAP[memoryPath]["address"]
		if self.MEMORY_MAP[memoryPath]["type"] == "char":
			(rxRam, reStatus) = self.readCmd(id, address, 1, option)
			reData = PyB3m.__assemblyByte( rxRam )	#char（長さ１）では、要素取り出すだけ
			reData = PyB3m.__unsignedToSigned(reData, 1)
		elif self.MEMORY_MAP[memoryPath]["type"] == "byte":
			(rxRam, reStatus) = self.readCmd(id, address, 1, option)
			reData = rxRam[0]
		elif self.MEMORY_MAP[memoryPath]["type"] == "short":
			(rxRam, reStatus) = self.readCmd(id, address, 2, option)
			reData = PyB3m.__assemblyByte( rxRam )
			reData = PyB3m.__unsignedToSigned(reData, 2)
		elif self.MEMORY_MAP[memoryPath]["type"] == "ushort":
			(rxRam, reStatus) = self.readCmd(id, address, 2, option)
			reData = PyB3m.__assemblyByte( rxRam )
		elif self.MEMORY_MAP[memoryPath]["type"] == "long":
			(rxRam, reStatus) = self.readCmd(id, address, 4, option)	#書き込み
			reData = PyB3m.__assemblyByte( rxRam )		#返り値(1byteづつのリスト)を結合して1つの値に変換
			reData = PyB3m.__unsignedToSigned(reData, 4)	#符号なし→符号付きに変換
		elif self.MEMORY_MAP[memoryPath]["type"] == "ulong":
			(rxRam, reStatus) = self.readCmd(id, address, 4, option)
			reData = PyB3m.__assemblyByte( rxRam )
		return reData, reStatus

	def setNewId(self, currentId, newId):
		re1 = self.loadCmd(currentId)
		re2 = self.setRam(currentId, newId, "ID")
		re3 = self.saveCmd(newId)
		return ( re1[0]==True and re2[0]==True and re3[0]==True )

	def getError(self, id):
		(systemError, motorError) = self.getRam(id, "StatusSystem", "MOTOR")
		(uartError, commandError) = self.getRam(id, "StatusUart", "COMMAND")
		(statusError, clearError) = self.getRam(id, "StatusError", "CLEAR")
		if systemError & 0b00000001:
			print "Watchdog Timerが起動したときに1になります"
		if systemError & 0b00000010:
			print "MCUのROMに保存されているデータに何らかの不都合があった場合に1になります。"
		if systemError & 0b00000100:
			print "メモリーに何らかの不具合があり、RAM割り当てに失敗したときに1になります。起動時に1度だけチェックします。"
		if systemError & 0b00001000:
			print "入力電圧が上限値を超えたか、下限値を下回った場合に1になります"
		if systemError & 0b00010000:
			print "MCU温度が上限値を超えた場合に1になります"
		if systemError & 0b00100000:
			print "AD変換に失敗したときに1になります"
		if systemError & 0b01000000:
			print "I2C通信に失敗したときに1になります（未使用）"
		if systemError & 0b10000000:
			print "SPI通信に失敗したときに1になります"

		if motorError & 0b00000001:
			print "モーター温度が上限値を超えた場合に1になります"
		if motorError & 0b00000010:
			print "モーターロックが検知された場合に1になります"
		if motorError & 0b0000100:
			print "モーターに流れる電流が上限値を超えた場合に1になります"
		if motorError & 0b00001000:
			print "ブラシレスモーターのホールICに不具合があった場合に1になります"

		if uartError & 0b00000001:
			print "フレミングエラー発生時に1になります"
		if uartError & 0b00000001:
			print "パリティエラー発生時に1になります"
		if uartError & 0b00000001:
			print "ブレークエラー発生時に1になります"
		if uartError & 0b00000001:
			print "オーバーランエラー発生時に1になります"

		if commandError & 0b00000001:
			print "コマンドのチェックサムが間違っている場合に1になります"
		if commandError & 0b00000010:
			print "コマンドのデバイス数が多すぎるあるいは少なすぎる場合に1になります"
		if commandError & 0b00000100:
			print "取得するデータ長さがアドレスを越えるほど長い場合に1になります"
		if commandError & 0b00001000:
			print "アドレスが指定範囲外だった場合に1になります"
		if commandError & 0b00001000:
			print "コマンド自身が間違っている場合に1になります"




# aaa=PyB3m("/dev/ttyUSB0", 1500000)
# aaa.begin()

# print "free"
# print aaa.setMode(0,"FREE")

# print "nomal mode"
# print aaa.setMode(0,"NORMAL")

# print "pos"
# print aaa.positionCmd(0,0)
# import time
# time.sleep(2)
# print aaa.positionCmd(0,20000)
# time.sleep(2)
"""
#終了時間制御をする例
print aaa.setTrajectoryType(0,"EVEN")
print aaa.setMode(0,"POSITION")
print aaa.positionCmd(0,32000, 10000)
"""
"""
#トルクを設定する例
print aaa.setMode(0,"TORQUE")
print aaa.setRam(0,100,"DesiredTorque")
"""