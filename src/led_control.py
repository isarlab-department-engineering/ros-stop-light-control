#!/usr/bin/env python

# this is a test script for drive motor 
# in function of stop and front lights detection
# this script will be implemented in another node 

# import libraries
import rospy,sys,time,atexit
import RPi.GPIO as gpio 
from std_msgs.msg import String
from Adafruit_MotorHAT import Adafruit_MotorHAT, Adafruit_DCMotor

# define variables
flag = 0 # tonera utile lo so 
mh = Adafruit_MotorHAT(addr=0x60)
m1 = mh.getMotor(1)
m2 = mh.getMotor(2)

def turnOffMotors():
	mh.getMotor(1).run(Adafruit_MotorHAT.RELEASE)
	mh.getMotor(2).run(Adafruit_MotorHAT.RELEASE)

def setSpeed(motor1,motor2):
	if motor1 == 0 and motor2 == 0:
		turnOffMotors()
	else:
		m1.setSpeed(motor1)
		m2.stetSpeed(motor2)

def avoidVehicle(): 
	turnOffMotors
	flag = 0 

def callback(data):
	rospy.loginfo(rospy.get_caller_id() +" Led control String received: %s",data.data)
	if data.data == "stop" :
		turnOffMotors()
	elif data.data == "front" and flag == 0:
		avoidVehicle()
		# until avoiding flag is high
		flag = 1
	
def led_control():
	rospy.init_node('led_control',anonymous=True)
	rospy.Subscriber('led_control_topic',String,callback)
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")

if __name__ == '__main__':
	led_control()
