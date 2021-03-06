#!/usr/bin/env python

from breaker_board_node.srv import *
import rospy
import Adafruit_GPIO as GPIO
import time
import diagnostic_updater
import diagnostic_msgs
import mraa

class BreakerBoardNode:

	def __init__(self):
		
		self._breaker = [[],[],[],[],[],[],[],[]]
		self._breaker[0]=[rospy.get_param('~gpio0', None), None]
		self._breaker[1]=[rospy.get_param('~gpio1', None), None]
		self._breaker[2]=[rospy.get_param('~gpio2', None), None]
		self._breaker[3]=[rospy.get_param('~gpio3', None), None]
		self._breaker[4]=[rospy.get_param('~gpio4', None), None]
		self._breaker[5]=[rospy.get_param('~gpio5', None), None]
		self._breaker[6]=[rospy.get_param('~gpio6', None), None]
		self._breaker[7]=[rospy.get_param('~gpio7', None), None]
		
		print(self._breaker[0])
	
    		for i in range (8):
			if self._breaker[i][0] is not None: 
				try:
					self._breaker[i][0] = mraa.Gpio(self._breaker[i][0])
					self._breaker[i][0].dir(mraa.DIR_OUT)
					self._breaker[i][0].write(1)
					self._breaker[i][1]=1
				except:
					self._breaker[i][0]=None           
				
		self.breaker_srv = rospy.Service('set_breaker_output', SetBreakerOutputs, self.set_breaker_output)
		
		self._updater = diagnostic_updater.Updater()
		self._updater.setHardwareID("none")
		self._updater.add("Breaker Board Status", self.breaker_diagnostics)

	def set_breaker_output (self,req):
		if req.pin > 7 or req.pin < 0:
			rospy.logerr("Breaker Board Pin Invalid [0-7]") 
			return SetBreakerOutputsResponse (False)
		
        	elif req.value > 1 or req.pin <0:
			rospy.logerr("Pin Value Invalid [0-1] [LOW-HIGH]")
			return SetBreakerOutputsResponse (False)
        
		elif self._breaker[req.pin][0] is None:
  			rospy.logerr("The pin was not configured on startup")
			return SetBreakerOutputsResponse (False)     
		
		else:
			if req.value:			
				self._breaker[req.pin][0].write(1)
				rospy.loginfo("Breaker Board pin "+str(req.pin)+" set")
                		self._breaker[req.pin][1]=1
			else:
				self._breaker[req.pin][0].write(0)
                		self._breaker[req.pin][1]=0
				rospy.loginfo("Breaker Board pin "+ str(req.pin)+" cleared")

		return SetBreakerOutputsResponse(True)	


	def breaker_diagnostics (self,stat):
		stat.summary(diagnostic_msgs.msg.DiagnosticStatus.OK, "Breaker Board OK")
		stat.add("Breaker_0", self._breaker[0][1])
		stat.add("Breaker_1", self._breaker[1][1])
		stat.add("Breaker_2", self._breaker[2][1])
		stat.add("Breaker_3", self._breaker[3][1])
		stat.add("Breaker_4", self._breaker[4][1])
        	stat.add("Breaker_5", self._breaker[5][1])
        	stat.add("Breaker_6", self._breaker[6][1])
       		stat.add("Breaker_7", self._breaker[7][1])
        	return stat		
 

if __name__ == "__main__":

	rospy.init_node('breaker_board')
	breaker = BreakerBoardNode()
	while not rospy.is_shutdown():
	        breaker._updater.update()
        	rospy.sleep(0.5)
		
