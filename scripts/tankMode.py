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
        self.open()
        self.speed = 50

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
	
    def goBackward(self,side,pwm):
        if side == 'rigth':
            for i in range(1,2):
                self.rccm[i].BackwardM1(self.msID[0],pwm)
        elif side == 'left':
            for i in range(1,2):
                self.rccm[i].ForwardM1(self.msID[1],pwm)
        else:
            print("Warnig: command not found.")

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
    #Driver 128, actuators

	#LEFT - Link base
    def Arm_Act(self,array_pos1,array_pos2,device,speed):
       
        if array_pos1 == 1:
            self.rccm[0].BackwardM2(device,speed)
            print "Back lbase"
        elif array_pos2 == -1:
            self.rccm[0].ForwardM2(device,speed)
            print "Ford lbase"
        elif array_pos2 == 1 and array_pos1 == 0:
            self.rccm[0].ForwardM2(device,0)
   
    #RIGHT - Link wrist
    def Arm_Act1(self,array_pos1,array_pos2,device,speed):
       
        if array_pos1 == 1:
            self.rccm[0].BackwardM1(device,speed)
            print "Back lwrist"
        elif array_pos2 == -1:
            self.rccm[0].ForwardM1(device,speed)
            print "Ford lwrist"
        elif array_pos2 == 1 and array_pos1 == 0:
            self.rccm[0].ForwardM1(device,0)
	
    #Driver 129, DC

    #Base - A (forward) / Y (backward)
    def Arm_DC(self, array_pos1, array_pos2, device, speed):
        if array_pos1 == 1:
			self.rccm[1].ForwardM2(device,speed)
			print "Ford base"
        elif array_pos2 == 1:
			self.rccm[1].BackwardM2(device,speed)
			print "Back base"
        elif array_pos1 == 0 and array_pos2 == 0:
			self.rccm[1].ForwardM2(device,0)

    #Wrist - B (forward) / X (backward)
    def Arm_DC1(self, array_pos1, array_pos2, device, speed):
        if array_pos1 == 1:
			self.rccm[1].ForwardM1(device,speed)
			print "Ford wrist"
        elif array_pos2 == 1:
			self.rccm[1].BackwardM1(device,speed)
			print "Back wrist"
        elif array_pos1 == 0 and array_pos2 == 0:
			self.rccm[1].ForwardM1(device,0)

    def set_speedDC(self, joy):
		#Arm base
        self.Arm_DC(joy.x,joy.y, 129, self.speed)	
		#Yaw Wrist 
        self.Arm_DC1(joy.z, joy.w, 129, self.speed)

    def set_speedActuator(self, joy2):
		#Link base 
        self.Arm_Act(joy2.x, joy2.y, 128, self.speed)

		#Link wrist
        self.Arm_Act1(joy2.z,joy2.w, 128, self.speed)

     

class servo:
    def __init__(self):
        GPIO.setmode(GPIO.BCM)
                
    def Arm_servoWrist(self, servo):
        GPIO.setup(13,GPIO.OUT)       
        self.SP = GPIO.PWM(13,50)
        self.SP.start(7.5)

        if servo == 1:
                self.SP.ChangeDutyCycle(10.5)
                time.sleep(0.010)
                print "izqW"

        elif servo == -1:
               self.SP.ChangeDutyCycle(4.5)
               time.sleep(0.010)
               print "derW"
        else:
               self.SP.ChangeDutyCycle(0)

        self.SP.stop()

    def Arm_servoGrip(self, servo):
        GPIO.setup(12,GPIO.OUT)       
        self.SP = GPIO.PWM(12,50)
        self.SP.start(7.5)

        if servo == 1:
                self.SP.ChangeDutyCycle(10.5)
                time.sleep(0.010)
                print "izqG"

        elif servo == -1:
               self.SP.ChangeDutyCycle(4.5)
               time.sleep(0.010)
               print "derG"
        else:
               self.SP.ChangeDutyCycle(0)

        self.SP.stop()
