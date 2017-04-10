#!/usr/bin/env python

# import necessary libraries
import sys, time, rospy
import RPi.GPIO as gpio
from std_msgs.msg import String

# LED configuration
FRONT_RED	= 19
FRONT_GREEN	= 20
FRONT_BLUE	= 21
REAR_RED	= 12
REAR_GREEN	= 13
REAR_BLUE	= 16

def turnOffGreen(int):

def turnOnPurple(int):

def turnOffPurple(int):

def emitter():
	# set up and initialize led 
	gpio.setmode(gpio.BCM)
	gpio.setwarnings(False)
	gpio.setup(FRONT_RED,gpio.OUT)
	gpio.setup(FRONT_GREEN,gpio.OUT)
	gpio.setup(FRONT_BLUE,gpio.OUT)
	gpio.setup(REAR_RED,gpio.OUT)
	gpio.setup(REAR_GREEN,gpio.OUT)
	gpio.setup(REAR_BLUE,gpio.OUT)
	# set up ros configurations
	rospy.init_node('led_emitter',anonymous=True)
	rospy.Subscriber("rospibot_network",String,callback)
	# loop until ros shutdown
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")

def callback(data):
	if(data.data == "w"): # turn on green and turn off purple when the car goes forward
		rospy.loginfo("turn on green lights")
		gpio.output(FRONT_GREEN,gpio.HIGH)
		gpio.output(REAR_RED,gpio.LOW)
		gpio.output(REAR_BLUE,gpio.LOW)
	if(data.data == "x"): # turn off green and turn on purple when car stops
		rospy.loginfo("turn on purple lights")
		gpio.output(FRONT_GREEN,gpio.LOW)
		gpio.output(REAR_RED,gpio.HIGH)
		gpio.output(REAR_BLUE,gpio.HIGH)

if __name__ == '__main__':
	try:
		emitter()
	except rospy.ROSInterruptException:
		pass
