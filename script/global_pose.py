#!/usr/bin/env python3

################################################################################
## Author: Khairul Izwan Bin Kamsani
## Version: {1}.{0}.{0}
## Email: {wansnap@gmail.com}
################################################################################

# import the necessary ROS packages
import rospy
from clover import srv
from std_srvs.srv import Trigger

# import the necessary Python packages
import math

class GlobalPose:
	def __init__(self):
	
		rospy.logwarn("Global Pose Node [ONLINE]...")
		
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
		
		# Allow up to one second to connection
		rospy.sleep(1)
		
	#
	def cbTelemetry(self):
		if not math.isnan(self.get_telemetry().lat):
			rospy.loginfo("Global position is available")
		else:
			rospy.logerr("No global position")
		
	# rospy shutdown callback
	def cbShutdown(self):
		rospy.logwarn("Global Pose Node [OFFLINE]...")
		
if __name__ == '__main__':

	# Initialize
	rospy.init_node('global_telemetry', anonymous=False)
	c = GlobalPose()
	r = rospy.Rate(10)
	
	# main
	while not rospy.is_shutdown():
		c.cbTelemetry()
		r.sleep()
