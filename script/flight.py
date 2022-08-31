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


class CloverBasicFlight:
	def __init__(self):
	
		rospy.logwarn("Auto Takeoff and Land Node [ONLINE]...")
		
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
	def cbTakeoff(self):
		rospy.loginfo("Take off and hover 1 m above the ground")
		self.navigate(x=0, y=0, z=1, frame_id='body', auto_arm=True)
		
	#
	def cbLand(self):
		rospy.loginfo("Perform landing")
		self.land()
		
	#
	def cbAutoFlight(self):
		self.cbTakeoff()
		
		# Wait for 5 seconds
		rospy.logwarn("Wait for 5 seconds...")
		rospy.sleep(10)
		
		self.cbLand()
		
	# rospy shutdown callback
	def cbShutdown(self):
		self.cbLand()
		rospy.logwarn("Auto Takeoff and Land Node [OFFLINE]...")
		
if __name__ == '__main__':

	# Initialize
	rospy.init_node('clover_basic_takeoff_and_land', anonymous=False)
	c = CloverBasicFlight()
	r = rospy.Rate(10)
	
	# main
	while not rospy.is_shutdown():
		c.cbAutoFlight()
		r.sleep()
		break
		
	c.cbShutdown()
