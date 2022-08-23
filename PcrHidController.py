# pcrEmulator.py
# -*- coding: utf-8 -*-

import json
import time
import os
from math import fabs

# import zmq
import hid
import struct

import logging
import threading 
from datetime import datetime, timedelta
from enum import IntEnum

logging.basicConfig(format='%(asctime)s - %(message)s', level=logging.INFO)
logger = logging.getLogger(__name__)

PCR_PORT = os.environ.get('PCR_PORT', '7001')

# Main loop
# context = zmq.Context()
# listener = context.socket(zmq.REP)
# listener.bind('tcp://*:%s' % PCR_PORT)

DEFAULT_PIDS = [
	{'start_temp' : 25.0, 'target_temp' : 95.0, 'Kp' : 460.0, 'Kd' : 3000.0, 'Ki' : 0.2},
	{'start_temp' : 95.0, 'target_temp' : 60.0, 'Kp' : 250.0, 'Kd' : 1000.0, 'Ki' : 0.3},
	{'start_temp' : 60.0, 'target_temp' : 72.0, 'Kp' : 350.0, 'Kd' : 3000.0, 'Ki' : 0.11},
	{'start_temp' : 72.0, 'target_temp' : 95.0, 'Kp' : 460.0, 'Kd' : 3000.0, 'Ki' : 0.18},
	{'start_temp' : 96.0, 'target_temp' : 50.0, 'Kp' : 500.0, 'Kd' : 1000.0, 'Ki' : 0.3}
	
]

class State(IntEnum):
    READY = 0x00,
    RUNNING = 0x01,

class Command(IntEnum):
    READY = 0x00,
    PCR_RUN = 0x01,
    PCR_STOP = 0x02,
    FAN_ON = 0x03,
    FAN_OFF = 0x04,

class Action:
	def __init__(self, label, temp, time):
		self.label = label
		self.temp  = temp
		self.time  = time
	
	def toDict(self):
		return {"label" : self.label, "temp" : self.temp, "time" : self.time} 
	
	def __str__(self):
		return f'label : {self.label}, temp : {self.temp}, time : {self.time}'

class Controller(threading.Thread):
	def __init__(self):
		threading.Thread.__init__(self)
		self.daemon = True

		# HID Device params
		vid = 0x04d8
		pid = 0x0041
		self.device = hid.Device(vid, pid)
		self.serial_number = self.device.serial
		self.currentError = 0

		self.pids = DEFAULT_PIDS
		self.pid = self.pids[0]

		self.running = False
		self.currentCommand = Command.READY
		self.complatePCR = False
		self.targetArrival = False
		self.targetArrivalFlag = False
		self.targetArrivalDelta = 0.5
		self.freeRunning = False
		self.freeRunningCounter = 0

		# Timed Loop params
		self.leftSec = 0				# current action remain time
		self.leftTotalSec = 0			# total remain time
		self.currentActionNumber = -1	# current action number
		self.totalActionNumber = 0		# total action number
		self.leftGotoCount = -1			# remain goto count
		self.startTime = None			# pcr start time
		self.elapsedTime = ''			# 
		

		self.state = State.READY
		self.stateString = 'idle'

		self.currentTemp = 20.0			# current pcr chip temperature
		self.prevTargetTemp = 25.0		# previous target temperature
		self.currentTargetTemp = 25.0	# current target temperature

		# need to setup
		self.compensation = 30			# 
		self.currentError = 0			# error code
		self.integralMax = 2600			# integral max value
		self.FLRelativeMax = 3600		# 

		# Detection params 
		self.leds = [True, True, True, True] 	# turn off : True(1), turn on : False(0)
		self.led_control = False				# led control param
		self.leds_pwm = [0] * 4					# leds pwm param
		# self.leds_pwm = [255] * 4
		self.currentPhotodiode = 0				# current photodiode value	
		self.photodiodes = [[], [], [], []]		# photodiodes values to List

		# for history
		self.result = ['', '', '', '']
		self.resultCts = ['', '', '', '']
		self.tempLogger = []
		

		# for filter 
		self.shotCounter = 0
		self.currentCycle = 0
		self.filterIndex = 0
		self.filterRunning = False
		
		self.filters = ["", "", "", ""]
		self.filterNames = ["", "", "", ""]
		self.filterCts = ["", "", "", ""]

		# filter string to list 
		filterStrings = ['FAM', 'HEX', 'ROX', 'CY5']


		
		
		# load recent protocol first
		# TEST Code
		default_protocol = [{"label":"1", "temp":95.0, "time":5},{"label":"2", "temp":95.0, "time":5},{"label":"3", "temp":55.0, "time":5},{"label":"4", "temp":72.0, "time":5},{"label":"GOTO", "temp":2.0, "time":4},{"label":"5", "temp":72.0, "time":5}]
		self.protocol = [Action(**action) for action in defaultProtocol]
		self.protocolName = 'Default Protocol'
		
		# TEST Code 
		self.totalActionNumber = len(self.protocol)
		# Calculate the protocol left time
		for idx, action in enumerate(self.protocol):
			_label = action.label
			_temp = action.temp
			_time = action.time

			if _label == 'GOTO':
				targetLabel = f'{_temp}'
				gotoIdx = -1
				for idx2, tempAction in enumerate(self.protocol):
					checkLabel = tempAction.label

					if checkLabel == targetLabel:
						gotoIdx = idx2
						break
				if gotoIdx != -1:
					tempTime = 0
					for i in range(gotoIdx, idx):
						tempTime += self.protocol[i].time
					tempTime *= time
					self.leftTotalSec += tempTime
			elif _label != 'SHOT':
				self.leftTotalSec += _time
	def startPCR(self):
		self.initValues()
		self.currentTargetTemp = self.protocol[0].temp

		self.photodiodes = [[], [], [], []]
		self.running = True
		self.currentCommand = Command.PCR_RUN
		pass
	
	def stopPCR(self):
		self.processCleanpPCR()

	def find_pid(self):
		if fabs(self.prevTargetTemp - self.currentTargetTemp) > .5:
			return
		dist = 10000
		index = 0

		for idx, pid in enumerate(self.pids):
			tmp = fabs(self.prevTargetTemp - pid['start_temp']) + fabs(self.currentTargetTemp - pid['target_temp'])
			if tmp < dist:
				dist = tmp
				index = idx
		self.pid = self.pids[index]

	
	def run(self):
		roundTimer = time.time()
		usbTimer = time.time()

		while True:
			currentTime = time.time()
			
			# USB Task (500 millesecond timer)
			if currentTime - usbTimer >= 0.5:
				# reset the usb task timer 
				usbTimer = time.time()

				# xfer hid device
				buffer = [self.currentCommand, int(self.currentTargetTemp)] # 2 byte
				buffer += [int(self.pid['start_temp']), int(self.pid['target_temp'])]  # 2 byte
				buffer += struct.pack('fff', self.pid['Kp'], self.pid['Ki'], self.pid['Kd'])	# 12 byte
				buffer += struct.pack('f', self.integralMax) # 4 byte
				buffer += [self.led_control] + self.leds # 5 byte
				buffer += [self.compensation] # 1 byte
				buffer += self.leds_pwm # 4 byte
				buffer += [0] * 34 # 30 ~ 63 reserved
				
				logMsg = f'Send Buffer - cmd : {self.currentCommand}, target_temp : {self.currentTargetTemp:.2f}, pid : {self.pid}'
				# fileLogger.info(logMsg)
				# buffer = bytes(self.make_buffer())
				self.device.write(bytes(buffer))
				buffer = self.device.read(64)
				
				# update params
				state = int(buffer[0])
				temperature = struct.unpack('<f', buffer[3:7])[0]
				photodiode = (int(buffer[7]) << 8) | int(buffer[8])
				current_error = int(buffer[9])
				request_data = int(buffer[10])
				# target_arrival = bool(buffer[11]) # this param is not using 
				# # 12 ~ 63 : reserved 
				
				self.state = state
				self.currentTemp = temperature
				self.currentPhotodiode = photodiode
				self.currentError = current_error

				# logging params  
				logMsg = f'state : {state}, temp : {temperature:.1f}, photo : {photodiode}, leftSec : {self.leftSec}, leftTotalSec :{self.leftTotalSec}'
				logger.info(logMsg)

			# Real-time PCR Task (1000 millesecond timer)
			if currentTime - roundTimer >= 1:
				# reset the timer 
				roundTimer = time.time()

				if self.running:
					elapsedTimeSec = (datetime.now() - self.startTime).total_seconds()
					mins, secs = divmod(elapsedTimeSec, 60)
					hours, mins = divmod(mins, 60)
					self.elapsedTime = '%02d:%02d:%02d' % (hours, mins, secs)	
					# logger.info(f'Elapsed time : {self.elapsedTime}')

					# ended current action
					if self.leftSec == 0:
						self.currentActionNumber += 1
						logger.info(f'Action {self.currentActionNumber}/{self.totalActionNumber}')

						if self.currentActionNumber >= self.totalActionNumber:
							logger.info('End of protocol.')
							self.complatePCR = True
							
							self.processCleanpPCR()
							continue

						currentAction = self.protocol[self.currentActionNumber]
						if currentAction.label == 'GOTO': # GOTO Label
							if self.leftGotoCount < 0:
								self.leftGotoCount = int(currentAction.time)
							
							if self.leftGotoCount == 0:
								logger.info('GOTO ended!')
								self.leftGotoCount = -1
							else:
								self.leftGotoCount -= 1 
								targetActionLabel = f'{int(currentAction.temp)}'
								logger.info(f'Check goto target label, left gotocount {self.leftGotoCount}, target : {targetActionLabel}')
								for i in range(self.currentActionNumber):
									if targetActionLabel == self.protocol[i].label:
										# The action number will increase, so decrease -1 idx first.
										self.currentActionNumber = i -1 
										logger.info(f'Target GOTO label found, {self.currentActionNumber}')
										break

						elif currentAction.label == 'SHOT': # SHOT Label
							filters = ['FAM', "HEX", "ROX", "CY5"]

							for idx, filterName in enumerate(filters):
								if self.filterIndex == idx:
									if not filterName in self.filters:
										self.filterIndex = idx+1

							# 4 is last filter
							if self.filterIndex == 4:
								self.filterIndex = 0
								self.currentCycle += 1

							else:
								if self.filterRunning:
									# TODO : checkthe motor moving is done.
									
									# turn on the led, save the result and turn off the led.
									self.leds[self.filterIndex] = False
									self.shotCounter += 1
									if self.shotCounter >= 2:
										# save the filter data
										self.photodiodes[self.filterIndex].append(self.currentPhotodiode)
										logger.info(f'Save the filter data{self.filterIndex} : {self.currentPhotodiode}')

										self.shotCounter = 0

										# next filter
										self.filterIndex += 1
										self.filterRunning = False
										# leds turn off
										self.leds = [True, True, True, True] 
								else:
									self.filterRunning = True
									# TODO : Run the filter motor


						else: 
							logger.info(f'current action is {currentAction}')
							self.prevTargetTemp = self.currentTargetTemp
							self.currentTargetTemp = currentAction.temp

							self.find_pid()

							self.targetArrivalFlag = self.prevTargetTemp > self.currentTargetTemp

							self.targetArrival = False
							self.leftSec = int(currentAction.time)

					else: # the action is running now.
						if not self.targetArrival:
							logger.info('Not target arrived.')
							pass
						else:
							# Just decrease the left seconds
							self.leftSec -= 1
							self.leftTotalSec -= 1
							logger.info(f'left time {self.leftSec}/{self.leftTotalSec}') 

					if self.targetArrivalFlag and not self.freeRunning:
						if self.currentTemp <= self.currentTargetTemp:
							logger.info('FreeRuning True!')
							self.freeRunning = True
							self.freeRunningCounter = 0
					
					if self.freeRunning:
						self.freeRunningCounter	+= 1
						logger.info(f'Free running counter : {self.freeRunningCounter}')

						# Check 3 second 
						if self.freeRunningCounter >= 3:
							logger.info("FreeRunning ended & target arrived")
							self.targetArrivalFlag = False
							self.freeRunning = False
							self.freeRunningCounter = 0
							self.targetArrival = True

					# Target arrived check
					if abs(self.currentTemp - self.currentTargetTemp) < self.targetArrivalDelta and not self.targetArrivalFlag:
						logger.info('target arrived checked')
						self.targetArrival = True
				if self.currentCommand == Command.PCR_STOP and self.state == State.READY:
					self.currentCommand = Command.READY
					self.stateString = 'idle'
				# optic task 

				# PCR task 
	def initValues(self):
		pass
	def processCleanpPCR(self):
		if self.complatePCR:
			for idx in range(4):
				if self.filters[idx] != '':
					# self.calcCT(idx)
					pass

			currentDate = (datetime.now() - timedelta(hours=9)).strftime('%Y-%m-%d %H:%M:%S')
			target = json.dumps(self.filterNames)
			filterData = json.dumps(self.filters)
			ct = json.dumps(self.resultCts)
			result = json.dumps(self.result)
			graphData = json.dumps(self.photodiodes)
			tempData = json.dumps(self.tempLogger)
			logger.info("history saved!")


		self.initValues()
		self.running = False

		# This value for notification on this function, not used yet.
		self.completePCR = False
		self.currentCommand = Command.PCR_STOP

				
defaultProtocol = [{"label":"1", "temp":95.0, "time":10},{"label":"2", "temp":95.0, "time":5},{"label":"3", "temp":55.0, "time":5},{"label":"4", "temp":72.0, "time":5},{"label":"GOTO", "temp":2.0, "time":4},{"label":"5", "temp":72.0, "time":5}]
controller = Controller()
# controller.protocol = defaultProtocol
controller.running =  True
controller.currentCommand = Command.PCR_RUN
controller.currentTargetTemp = controller.protocol[0].temp
controller.startTime = datetime.now()

controller.run()