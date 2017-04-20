#!/usr/bin/env python

# add at exit stop light
# add configuration for commone cathode / commone anode diode

# import necessary libraries
import sys, time, rospy
import RPi.GPIO as gpio
from std_msgs.msg import String

# LED configuration
FRONT_GREEN1	= 21
FRONT_GREEN2	= 20
REAR_BLUE1	= 19
REAR_BLUE2	= 16

def emitter():
	# set up and initialize led 
	gpio.setmode(gpio.BCM)
	gpio.setwarnings(False)
	gpio.setup(FRONT_GREEN1,gpio.OUT)
	gpio.setup(REAR_BLUE1,gpio.OUT)
	gpio.setup(FRONT_GREEN2,gpio.OUT)
	gpio.setup(REAR_BLUE2,gpio.OUT)
	# set up ros configurations
	rospy.init_node('led_emitter',anonymous=True)
	rospy.Subscriber("cmdinfo",String,callback)
	# loop until ros shutdown
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")

def callback(data):
	if(data.data == "forward"): # turn on green and turn off purple when the car goes forward
		rospy.loginfo("turn on green lights")
		gpio.output(FRONT_GREEN1,gpio.HIGH)
		gpio.output(FRONT_GREEN2,gpio.HIGH)
		gpio.output(REAR_BLUE1,gpio.HIGH)
		gpio.output(REAR_BLUE2,gpio.HIGH)
	if(data.data == "stop"): # turn off green and turn on purple when car stops
		rospy.loginfo("turn on blue lights")
		gpio.output(REAR_BLUE1,gpio.LOW)
		gpio.output(REAR_BLUE2,gpio.LOW)
		gpio.output(FRONT_GREEN1,gpio.LOW)
		gpio.output(FRONT_GREEN2,gpio.LOW)

if __name__ == '__main__':
	try:
		emitter()
	except rospy.ROSInterruptException:
		pass
