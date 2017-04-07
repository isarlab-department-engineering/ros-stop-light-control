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
mh = Adafruit_MotorHAT(addr=0x60)
m1 = mh.getMotor(1)
m2 = mh.getMotor(2)
speed = 150
motor1Balance = 0
motor2Balance = 0
m1.setSpeed(speed+motor1Balance)
m2.setSpeed(speed+motor2Balance)

def turnOffMotors():
	mh.getMotor(1).run(Adafruit_MotorHAT.RELEASE)
	mh.getMotor(2).run(Adafruit_MotorHAT.RELEASE)

def callback(data):
	rospy.loginfo(rospy.get_caller_id())
	
def led_control():
	rospy.init_node('led_control',anonymous=True)
	rospy.Subscriber('led_control_public',String,callback)
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")

if __name__ == '__main__':
	led_control()