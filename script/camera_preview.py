#!/usr/bin/env python3

################################################################################
## Author: Khairul Izwan Bin Kamsani
## Version: {1}.{0}.{0}
## Email: {wansnap@gmail.com}
################################################################################

# import the necessary Python packages
from __future__ import print_function
import cv2

# import the necessary ROS packages
import rospy
from clover import srv
from std_srvs.srv import Trigger
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
from cv_bridge import CvBridgeError

class CloverCameraPreview:
	def __init__(self):
	
		self.bridge = CvBridge()
		self.image_received = False
		
		rospy.logwarn("Camera Preview Node [ONLINE]...")
		
		# rospy shutdown
		rospy.on_shutdown(self.cbShutdown)
		
		self.get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
		self.navigate = rospy.ServiceProxy('navigate', srv.Navigate)
		self.navigate_global = rospy.ServiceProxy('navigate_global', srv.NavigateGlobal)
		self.set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
		self.set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
		self.set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
		self.set_rates = rospy.ServiceProxy('set_rates', srv.SetRates)
		self.land = rospy.ServiceProxy('land', Trigger)
		
		# Subscribe to CompressedImage msg
		self.cloverImage_topic = "/main_camera/image_raw/compressed"
		self.cloverImage_sub = rospy.Subscriber(
						self.cloverImage_topic, 
						CompressedImage, 
						self.cbImage
						)
		
		# Allow up to one second to connection
		rospy.sleep(1)
		
	# 
	# Convert image to OpenCV format
	def cbImage(self, msg):
		try:
			# direct conversion to cv2
			self.cv_image = self.bridge.compressed_imgmsg_to_cv2(
								msg, 
								"bgr8"
								)
		except CvBridgeError as e:
			print(e)

		if self.cv_image is not None:
			self.image_received = True
		else:
			self.image_received = False
		
	#
	def cbShowImage(self):
		cv2.imshow("Camera Preview", self.cv_image)
		cv2.waitKey(1)

	# Preview image + info
	def cbPreview(self):

		if self.image_received:
			self.cbShowImage()
		else:
			rospy.logerr("No images recieved")
			
	# rospy shutdown callback
	def cbShutdown(self):
		rospy.logwarn("Camera Preview Node [OFFLINE]...")
		
if __name__ == '__main__':

	# Initialize
	rospy.init_node('clover_camera_preview', anonymous=False)
	c = CloverCameraPreview()
	r = rospy.Rate(10)
	
	# main
	while not rospy.is_shutdown():
		c.cbPreview()
		r.sleep()
