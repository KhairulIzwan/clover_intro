#!/usr/bin/env python3

################################################################################
## Author: Khairul Izwan Bin Kamsani
## Version: {1}.{0}.{0}
## Email: {wansnap@gmail.com}
## References:
## 1. https://pyimagesearch.com/2020/12/28/determining-aruco-marker-type-with-opencv-and-python/
## 2. https://pyimagesearch.com/2020/12/21/detecting-aruco-markers-with-opencv-and-python/
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
from aruco_pose.msg import MarkerArray

class CloverArucoDetection:
	def __init__(self):
	
		self.bridge = CvBridge()
		self.image_received = False
		
		# define names of each possible ArUco tag OpenCV supports
		self.ARUCO_DICT = {
			"DICT_4X4_50": cv2.aruco.DICT_4X4_50,
			"DICT_4X4_100": cv2.aruco.DICT_4X4_100,
			"DICT_4X4_250": cv2.aruco.DICT_4X4_250,
			"DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
			"DICT_5X5_50": cv2.aruco.DICT_5X5_50,
			"DICT_5X5_100": cv2.aruco.DICT_5X5_100,
			"DICT_5X5_250": cv2.aruco.DICT_5X5_250,
			"DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
			"DICT_6X6_50": cv2.aruco.DICT_6X6_50,
			"DICT_6X6_100": cv2.aruco.DICT_6X6_100,
			"DICT_6X6_250": cv2.aruco.DICT_6X6_250,
			"DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
			"DICT_7X7_50": cv2.aruco.DICT_7X7_50,
			"DICT_7X7_100": cv2.aruco.DICT_7X7_100,
			"DICT_7X7_250": cv2.aruco.DICT_7X7_250,
			"DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
			"DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
			"DICT_APRILTAG_16h5": cv2.aruco.DICT_APRILTAG_16h5,
			"DICT_APRILTAG_25h9": cv2.aruco.DICT_APRILTAG_25h9,
			"DICT_APRILTAG_36h10": cv2.aruco.DICT_APRILTAG_36h10,
			"DICT_APRILTAG_36h11": cv2.aruco.DICT_APRILTAG_36h11
		}
		
		# load the ArUCo dictionary, grab the ArUCo parameters, and detect
		# the markers
#		print("[INFO] detecting '{}' tags...".format(args["type"]))
		self.arucoDict = cv2.aruco.Dictionary_get(self.ARUCO_DICT["DICT_4X4_250"])
		self.arucoParams = cv2.aruco.DetectorParameters_create()

		rospy.logwarn("ArUco Detection Node [ONLINE]...")
		
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
#		# Publish
#		self.cloverAruco_topic = "/main_camera/markers"
#		self.cloverAruco_sub = rospy.Subscriber(
#						self.cloverAruco_topic, 
#						MarkerArray, 
#						self.cbMarkers)
		
		# Allow up to one second to connection
		rospy.sleep(1)
		
	# Convert image to OpenCV format
	def cbImage(self, msg):
	
		try:
			# direct conversion to cv2
			self.cv_image = self.bridge.compressed_imgmsg_to_cv2(
								msg, 
								"bgr8"
								)
#			self.cv_image = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2GRAY)
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
			self.cbArUcoDetection()
			self.cbShowImage()
		else:
			rospy.logerr("No images recieved")
		
	#
	def cbArUcoDetection(self):
	
		# loop over the types of ArUco dictionaries
		for (arucoName, arucoDict) in self.ARUCO_DICT.items():
			# load the ArUCo dictionary, grab the ArUCo parameters, and
			# attempt to detect the markers for the current dictionary
			self.arucoDict = cv2.aruco.Dictionary_get(arucoDict)
			self.arucoParams = cv2.aruco.DetectorParameters_create()
			(corners, ids, rejected) = cv2.aruco.detectMarkers(
										self.cv_image, 
										self.arucoDict, 
										parameters=self.arucoParams)
										
			# verify *at least* one ArUco marker was detected
			if len(corners) > 0:
				# flatten the ArUco IDs list
				ids = ids.flatten()
				# loop over the detected ArUCo corners
				for (markerCorner, markerID) in zip(corners, ids):
					# extract the marker corners (which are always returned in
					# top-left, top-right, bottom-right, and bottom-left order)
					corners = markerCorner.reshape((4, 2))
					(topLeft, topRight, bottomRight, bottomLeft) = corners
					
					# convert each of the (x, y)-coordinate pairs to integers
					topRight = (int(topRight[0]), int(topRight[1]))
					bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
					bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
					topLeft = (int(topLeft[0]), int(topLeft[1]))
					
					# draw the bounding box of the ArUCo detection
					cv2.line(self.cv_image, topLeft, topRight, (0, 255, 0), 1)
					cv2.line(self.cv_image, topRight, bottomRight, (0, 255, 0), 1)
					cv2.line(self.cv_image, bottomRight, bottomLeft, (0, 255, 0), 1)
					cv2.line(self.cv_image, bottomLeft, topLeft, (0, 255, 0), 1)
					
					# compute and draw the center (x, y)-coordinates of the ArUco
					# marker
					cX = int((topLeft[0] + bottomRight[0]) / 2.0)
					cY = int((topLeft[1] + bottomRight[1]) / 2.0)
					cv2.circle(self.cv_image, (cX, cY), 4, (0, 0, 255), -1)
					
					# draw the ArUco marker ID on the image
					cv2.putText(self.cv_image, str(markerID),
						(topLeft[0], topLeft[1] - 15), cv2.FONT_HERSHEY_SIMPLEX,
						0.5, (0, 255, 0), 1)
#					print("[INFO] ArUco marker ID: {}".format(markerID))
#					print("[INFO] detected {} markers for '{}'".format(
#												len(corners), 
#												arucoName)
#												)
				
	# rospy shutdown callback
	def cbShutdown(self):
		rospy.logwarn("ArUco Detection Node [OFFLINE]...")
		
if __name__ == '__main__':

	# Initialize
	rospy.init_node('clover_camera_preview', anonymous=False)
	c = CloverArucoDetection()
	r = rospy.Rate(10)
	
	# main
	while not rospy.is_shutdown():
		c.cbPreview()
		r.sleep()
