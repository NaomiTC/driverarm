#! usr/bin/python
import RPi.GPIO as GPIO
from numpy import *
import time
import os

import rospy
from sensor_msgs.msg import Joy
from tankMode import *
from geometry_msgs.msg import Quaternion

class control:
	def __init__(self):
#-------------Variables for drivers Pololu-------------
		self.joy = Quaternion()
		self.joy2 = Quaternion()		
		self.dev = [128,129]          #Array to daisy chain assigment
		self.pitchServo     = 0
		self.grapServo      = 0
		self.arm            = tank(port=['ttyACM3', 'ttyACM2'],baud='38400',pwmLimit=50,motorsID=self.dev)
		self.gripper        = servo()


		#Define suscriber
		rospy.Subscriber("joy", Joy, self.movements)
		
		r = rospy.Rate(10)
		while not rospy.is_shutdown():
			self.arm.set_speedDC(self.joy)
			self.arm.set_speedActuator(self.joy2)
			print self.joy2
			self.gripper.Arm_servo(self.grapServo)
			#self.gripper.Arm_servoGrip(self.pitchServo)
			r.sleep()
		
	def movements(self,data):

		#ARM BASE
		#Arm base left movement (A)	
		self.joy.x = data.buttons[0]
		#Arm base right movement (Y)
		self.joy.y = data.buttons[3] 
		 	 
		#YAW WRIST
		#Yaw Wrist right movement (B)
		self.joy.z = data.buttons[1] 
		#Yaw Wrist left movement (X) 
		self.joy.w = data.buttons[2]
		
		#LINK BASE
		#Contraction Link-Base movement (LB) 
		self.joy2.x = data.buttons[4]
		#Extension   Link-Base movement (LT) 
		self.joy2.y = data.axes[2]

		#LINK WRIST
		#Contraction Link-Wrist movement (RB) 
		self.joy2.z = data.buttons[5]
		#Extension   Link-Wrist movement (RT) 
		self.joy2.w = data.axes[5]

		#SERVOS		
		#self.pitchServo  = data.axes[7]   #Wrist movement (horizontal cross)
		self.grapServo   = data.axes[6]	  #Grap movement (vertical cross)
		

	
			
 	
if __name__ == '__main__':
	os.system("sudo chmod 777 /dev/ttyACM0")
	os.system("sudo chmod 777 /dev/ttyACM1")
	#os.system("sudo chmod 777 /dev/ttyAMA0")	
	rospy.init_node('ArmControl')
	control()
	
		
	

	

	

	
