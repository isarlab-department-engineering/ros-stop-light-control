#!/usr/bin/env python

import rospy,sys,cv2,numpy,roslib
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from stop_detection.srv import * 
from stop_detection.msg import Twist

# init cvbridge
bridge = CvBridge()
# define message
stopmotor = Twist()

''' 
HERE GOES ALL PARAMETERS 
'''
# stop light = purple
# front light = green
# purple hsv treshold boundaries
purpleLower = numpy.array([125,180,50],dtype = "uint8")
purpleUpper = numpy.array([145,255,155],dtype = "uint8")
# green hsv treshold boundaries
greenLower = numpy.array([60,200,60],dtype = "uint8")
greenUpper = numpy.array([70,255,155],dtype = "uint8")
# treshold detection of mean value
tresholdPurpleDetection = 100
tresholdGreenDetection = 100
purpleSectors = 20
greenSectors = 20
count = 0
# set stop motor message
stopmotor.linear.x=0
stopmotor.linear.y=0
stopmotor.linear.z=0
stopmotor.angular.x=0
stopmotor.angular.y=0
stopmotor.angular.z=0
# suppose you will always find lights in the low part of image
# image have 160*128 resolution
# treshold for crop image
ymin = 88
ymax = 118
xmin = 30
xmax = 130
# variables for sectors
# NOTE. xmax-xmin must be divisibile for xsector, same for ymax-ymin with ysector
xsectors = 20
ysectors = 10
# dimension of sectors
xsectordim = (xmax-xmin)/xsectors
ysectordim = (ymax-ymin)/ysectors
# ros variables 
# !!!!!! change in services system
# deprecated ledControlPublish = rospy.Publisher("led_control_topic",String,queue_size=10)
stop_service = rospy.ServiceProxy('stop',StopService)
# debbuging topic
croppedPublish = rospy.Publisher("cropped_image",Image,queue_size=10)
notTassellatedPurplePublish = rospy.Publisher("led_detector_purple_not_tassellated",Image,queue_size=10)
notTassellatedGreenPublish = rospy.Publisher("led_detector_green_not_tassellated",Image,queue_size=10)
tassellatedPurplePublish = rospy.Publisher("led_detector_purple_tassellated",Image,queue_size=10)
tassellatedGreenPublish = rospy.Publisher("led_detector_green_tassellated",Image,queue_size=10)

''' 
END PARAMETERS
'''

def detector():
	rospy.init_node('led_detector',anonymous=True)
	rospy.Subscriber("slow_image_topic",Image,callback)
	rospy.wait_for_service('stop')
	try:
		rospy.loginfo("Entering ros spin")
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")


# insert service in callback function

def callback(data):
   try:
	global count, stopmotor
	if(count % 30 == 0):
		if(stop_service(stopmotor) == True):
			rospy.loginfo("Information received")
	count = count +1
   except CvBridgeError as e:
	print(e)

if __name__ == "__main__":
	try:
		detector()
	except rospy.ROSInterruptException:
		pass
