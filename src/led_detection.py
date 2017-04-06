#!/usr/bin/env python
from __future__ import print_function
import sys,rospy,cv2, numpy, roslib
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

# init cvbridge
bridge = CvBridge()

'''
HERE GOES ALL PARAMETERS 
'''

# stop light = purple
# front light = green
# purple hsv treshold boundaries
rLower1 = numpy.array([0,204,60],dtype = "uint8")
rUpper1 = numpy.array([10,255,127],dtype = "uint8")
# green hsv treshold boundaries
rLower2 = numpy.array([170,204,60],dtype = "uint8")
rUpper2 = numpy.array([180,255,127],dtype = "uint8")
# treshold detection of mean value
tresholdDetection = 10
# suppose you will always find lights in the low part of image
# image have 160*120 resolution
# treshold for crop image
ymin = 70
ymax = 120
xmin = 30
xmax = 130
# variables for sectors
# NOTE. xmax-xmin must be divisibile for xsector, same for ymax-ymin with ysector
xsectors = 20
ysectors = 10
# dimension of sectors
ysectordim = (xmax-xmin)/xsectors
xsectordim = (ymax-ymin)/ysectors
# ros variables
croppedPublish = rospy.Publisher("cropped_image",Image,queue_size=10)
notTassellatedPublish = rospy.Publisher("led_detector_not_tassellated",Image,queue_size=10)
tassellatedPublish = rospy.Publisher("led_detector_tassellated",Image,queue_size=10)
controlPublish = rospy.Publisher("redmask_detection_topic",String,queue_size=10)

'''
END PARAMETERS
'''

def detector():
	rospy.init_node('led_detector',anonymous=True)
	rospy.Subscriber("slow_image_topic",Image,callback)
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")

def callback(data):
   try:
	rospy.loginfo(rospy.get_caller_id()+"Received an image")
	# import cv image
	cvImage = bridge.imgmsg_to_cv2(data)
	# crop image 
	croppedImage = cvImage[ymin:ymax,xmin:xmax]
	croppedPublish.publish(bridge.cv2_to_imgmsg(croppedImage))
	# convert in hsv format
	hsvImage = cv2.cvtColor(croppedImage,cv2.COLOR_BGR2HSV)
	# generate red mask to test detection
	redMask1 = cv2.bitwise_and(hsvImage,hsvImage,mask=cv2.inRange(hsvImage,rLower1,rUpper1))
	redMask2 = cv2.bitwise_and(hsvImage,hsvImage,mask=cv2.inRange(hsvImage,rLower2,rUpper2))
	redMask = cv2.bitwise_or(redMask1,redMask2)
	notTassellatedPublish.publish(bridge.cv2_to_imgmsg(redMask))
	# tassellation of calculated redmask 	
	for y in range(ysectors):
		for x in range(xsectors):
			mean = numpy.mean(redMask[y*ysectordim:(y+1)*ysectordim,x*xsectordim:(x+1)*xsectordim])
			for i in range(y*ysectordim,(y+1)*ysectordim):
				for j in range(x*xsectordim,(x+1)*xsectordim):
					redMask[i][j] = mean
	# search red led in calulcated sectors
	for y in range(ysectordim/2,ymax-ymin,ysectordim):
		for x in range(xsectordim,xmax-xmin,xsectordim):
			 if(redMask[y][x][0] > tresholdDetection):
				rospy.loginfo("FOUND RED")
				break
		else:
			continue
		break
	tassellatedPublish.publish(bridge.cv2_to_imgmsg(redMask))
   except CvBridgeError as e:
	print(e)
	
if __name__ == '__main__':
	try:
		detector()
	except rospy.ROSInterruptException:
		pass
