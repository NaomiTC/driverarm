#!/usr/bin/env python

import RPi.GPIO as GPIO
from numpy import *
import time
from sys import exit

from roboclaw import Roboclaw

class tank:
    def __init__(self,port,baud,pwmLimit,motorsID):
        self.pord = port
        self.brte = baud
        self.pwml = pwmLimit
        self.msID = motorsID
        self.rccm = self.createRC(port,baud)
        #self.Arm_Actuator()
        self.open()


	


	'''
		array_pos1 = 0
		array_pos2 = 0
		device = 0
		speed = 0

		if array_pos1 == 1:
			self.rccm[0].BackwardM1(device,speed)
			print "BACK"
		elif array_pos2 == -1:
			self.rccm[0].ForwardM1(device,speed)
			print "FORWARD"
		elif array_pos2 == 1 or array_pos1 == 0:
			self.rccm[0].ForwardM1(device,0)
	'''
    def tankDrive(self,joy):
        if joy.y > 0.2:
            self.goForward('rigth',self.fixPwm(joy.y))
        elif joy.y < -0.2:
            self.goBackward('rigth',self.fixPwm(-joy.y))
        else:
            self.setTo0('rigth')

        if joy.x > 0.2:
            self.goForward('left',self.fixPwm(joy.x))
        elif joy.x < -0.2:
            self.goBackward('left',self.fixPwm(-joy.x))
        else:
            self.setTo0('left')
    def armDrive(self,arm):
        self.pich(arm.y)
        self.yaw(arm.z)
        self.Forward(arm.x)


    def open(self):
        for i in range(0,len(self.pord)):
            #print (self.rccm)
            if self.rccm[i].Open():
               print(self.rccm[i]._port)
            else: 
                exit("Error: cannot open port: " + self.pord[i])

    def createRC(self,port,baud):
        print(port)
        listrc = [None,None,None]
        for i in range(0,len(port)):
            listrc[i] = Roboclaw('/dev/' + port[i], baud)
        return listrc

    def goForward(self,side,pwm):
        if side == 'rigth':
            for i in range(1,2):
                self.rccm[i].ForwardM1(self.msID[0],pwm)
        elif side == 'left':
            for i in range(1,2):
                self.rccm[i].BackwardM1(self.msID[1],pwm)

    def Forward(self,x):
        if x == 1:
            self.goForward('right',50)
            self.goForward('left',50)    
        else:
            self.setTo0('right')
            self.setTo0('left')    

	

    def goBackward(self,side,pwm):
        if side == 'rigth':
            for i in range(1,2):
                self.rccm[i].BackwardM1(self.msID[0],pwm)
        elif side == 'left':
            for i in range(1,2):
                self.rccm[i].ForwardM1(self.msID[1],pwm)
        else:
            print("Warnig: command not found.")

    def pitch(self,id):
        if id == 1:
            self.rccm[0].ForwardM1(self.msID[0],80)
        elif id == -1:
            self.rccm[0].BackwardM1(self.msID[0],80)
        else:
            self.rccm[0].ForwardM1(self.msID[0],0)

	def otrachida(self):
		return

	def brazoactuador(self,meh):
		meh = 0
		return
	
	def otravergas(self):
		return

    def yaw(self,id):
        if id == 1:
            self.rccm[0].ForwardM1(self.msID[1],20)
        elif id == -1:
            self.rccm[0].BackwardM1(self.msID[1],20)
        else:
            self.rccm[0].ForwardM1(self.msID[1],0)

    def setTo0(self,side):
        if side == 'rigth':
            for i in range(1,2):
                self.rccm[i].ForwardM1(self.msID[0],0)
        if side == 'left':
            for i in range(1,2):
                self.rccm[i].ForwardM1(self.msID[1],0)

    def fixPwm(self,percentage):
        return long(round(percentage*self.pwml,2))

	#ARM
	
	
	def Arm_DC(self, array_pos1, array_pos2, device, speed):
		if self.joy[array_pos1] == 1:
			self.robclw.ForwardM2(device,speed)
			print "Right"
		elif self.joy[array_pos2] == 1:
			self.robclw.BackwardM2(device,speed)
			print "Left"
		elif self.joy[array_pos1] == 0 or self.joy[array_pos2] == 0:
			self.robclw.ForwardM2(device,0)

class servo:
	def __init__(self, ports):
		self.serv = ports
		GPIO.setmode(GPIO.BCM)

		for i in range (0,1):
			GPIO.setup(self.serv[i],GPIO.OUT)	
			self.ServoPWM = GPIO.PWM(self.serv[i],50)
			self.ServoPWM.start(7.5)

	def Arm_servo(self, servo, dut_cyc1, dut_cyc2):
		if servo == 1:
			self.ServoPWM.ChangeDutyCycle(dut_cyc1)
			time.sleep(0.010)
			print "derecha"
		elif servo == -1:
			self.ServoPWM.ChangeDutyCycle(dut_cyc2)
			print "izquierda"
		else:
			self.ServoPWM.ChangeDutyCycle(0)


