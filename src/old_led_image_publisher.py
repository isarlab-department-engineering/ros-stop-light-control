#!/usr/bin/env python

''' 
This script read full picture from image_topic 
take only some of all frames per second avaible
and crops the pic for help led detection
''' 

import rospy,time,cv2
from cv_bridge import CvBridge,CvBridgeError
from sensor_msgs.msg import Image
# init cvBridge
bridge = CvBridge()
# suppose you will always find lights in the
# inner and lower part of the image 
# image will be 1280+720 pixel
ymin = 225
ymax = 360
xmin = 90
xmax = 550
# init ros variables
imagePub = rospy.Publisher('led_image',Image,queue_size=1)

def led_publish():
	# initalize ros functions
	rospy.init_node('led_image_publisher',anonymous=True)
	rospy.Subscriber('slow_image_topic',Image,callback,queue_size=1)
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")

def callback(data):
	try:
		# take the slow rate published frame and crop to use in led detecion
		cvImage =cv2.resize(bridge.imgmsg_to_cv2(data),(640,360))
		croppedImage = cvImage[ymin:ymax,xmin:xmax]
		imagePub.publish(bridge.cv2_to_imgmsg(croppedImage))
		rospy.loginfo("Published pic")
	except CvBridgeError as e:
		print(e)

if __name__ == '__main__':
	led_publish()
