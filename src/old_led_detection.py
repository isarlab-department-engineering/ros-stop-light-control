#!/usr/bin/env python
from __future__ import print_function
import sys,rospy,cv2, numpy, roslib
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

bridge = CvBridge()
# treshold boundaries
rLower1 = numpy.array([0,204,90],dtype = "uint8")
rLower2 = numpy.array([170,204,90],dtype = "uint8")
rUpper1 = numpy.array([10,255,127],dtype = "uint8")
rUpper2 = numpy.array([180,255,127],dtype = "uint8")
# suppose you will always find lights in the low part of image
# image will be 160x112
ymin = 0
ymax = 67
xmin = 0
xmax = 230
# ros variables
imagePublish = rospy.Publisher("image_led_detector",Image,queue_size=10)
controlPublish = rospy.Publisher("redmask_detection_topic",String,queue_size=10)
def detector():
	rospy.init_node('led_detector',anonymous=True)
	rospy.Subscriber("led_image",Image,callback)
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")

def callback(data):
   try:
	rospy.loginfo(rospy.get_caller_id()+"Received an image")
	# convert and resize image
	cvImage = cv2.resize(bridge.imgmsg_to_cv2(data),(xmax,ymax))
	cvImage = cv2.cvtColor(cvImage,cv2.COLOR_BGR2HSV)
	# generate red mask to test detection
	# redMask = cv2.inRange(cvImage,rLower,rUpper)
	redMask1 = cv2.bitwise_and(cvImage,cvImage,mask=cv2.inRange(cvImage,rLower1,rUpper1))
	redMask2 = cv2.bitwise_and(cvImage,cvImage,mask=cv2.inRange(cvImage,rLower2,rUpper2))
	redMask = cv2.bitwise_or(redMask1,redMask2)
	# search red led
	# insert while or method to escape for cycle 
	''' 
	for i in range(ymin,ymax,1):
		for j in range(xmin,xmax,1):
			if redMask[i][j][0] > 0:
				rospy.loginfo("FOUND RED")
				controlPublish.publish("RRR")
				break
		else:
			continue
		break
	'''
	for i in range(ymin,ymax,1):
		for j in range(xmin,xmax,1):
			if redMask[i][j][0] > 0:
				rospy.loginfo("FOUND RED")
				controlPublish.publish("RRR")
				break
		else:
			continue
		break
	cv2.rectangle(redMask,(xmin,ymin),(xmax,ymin),(30,30,255),1)
	imagePublish.publish(bridge.cv2_to_imgmsg(redMask))
   except CvBridgeError as e:
	print(e)
	
if __name__ == '__main__':
	try:
		detector()
	except rospy.ROSInterruptException:
		pass
