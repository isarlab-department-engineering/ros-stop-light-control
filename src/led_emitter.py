#!/usr/bin/env python

# import necessary libraries
import sys, time, rospy
import RPi.GPIO as gpio
from std_msgs.msg import String

# LED configuration
RED_PIN		= 38
GREEN_PIN	= 40
BLUE_PIN	= 35

def emitter():
	# set up and initialize led 
	gpio.setmode(gpio.BOARD)
	gpio.setwarnings(False)
	gpio.setup(RED_PIN,gpio.OUT)
	gpio.setup(GREEN_PIN,gpio.OUT)
	gpio.setup(BLUE_PIN,gpio.OUT)
	# set up ros configurations
	rospy.init_node('led_emitter',anonymous=True)
	rospy.Subscriber("rospibot_network",String,callback)
	# loop until ros shutdown
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")

def callback(data):
	if(data.data == "w"):
		rospy.loginfo("turn on led")
		gpio.output(RED_PIN,gpio.HIGH) # turn on red
	if(data.data == "x"):
		rospy.loginfo("turn off led")
		gpio.output(RED_PIN,gpio.LOW) # turn off red
		gpio.output(GREEN_PIN,gpio.LOW)
     	if(data.data == "a"):
		gpio.output(GREEN_PIN,gpio.HIGH)            

if __name__ == '__main__':
	try:
		emitter()
	except rospy.ROSInterruptException:
		pass
