import roslib;roslib.load_manifest('create_2_dashboard')
import rospy

import diagnostic_msgs
import create_node.srv
import create_node.msg

from rqt_robot_dashboard.dashboard import Dashboard
from rqt_robot_dashboard.widgets import MonitorDashWidget, ConsoleDashWidget, MenuDashWidget, BatteryDashWidget, IconToolButton, NavViewDashWidget
from QtGui import QMessageBox, QAction
from python_qt_binding.QtCore import QSize

from .battery import CreateBattery
from .dock import DockingWidget
from .motor import MotorWidget
from .buzzer import BuzzerWidget
from .breaker import BreakerButton

import rospkg
import os.path

rp = rospkg.RosPack()

image_path = image_path = os.path.join(rp.get_path('create_2_dashboard'), 'images')

class Create2Dashboard(Dashboard):
	def setup(self, context):
		self.message = None

		self._dashboard_message = None
		self._last_dashboard_message_time = 0.0

        # These were moved out of get_widgets because they are sometimes not defined
        # before being used by dashboard_callback. Could be done more cleanly than this
        # though.
		self.lap_bat = BatteryDashWidget("Laptop")
		self.create_bat = CreateBattery("Create")
		self.motors = [MotorWidget('/main_motor')]
		self.breaker_board = [  BreakerButton('Breaker_0',0),
								BreakerButton('Breaker_1',1),
								BreakerButton('Breaker_2',2),
								BreakerButton('Breaker_3',3),
								BreakerButton('Breaker_4',4),
								BreakerButton('Breaker_5',5),
								BreakerButton('Breaker_6',6),
								BreakerButton('Breaker_7',7)]

		self.buzzer = [BuzzerWidget('/song')]
		self.dock = [DockingWidget ('/undock', '/dock')]
		    
        # This is what gets dashboard_callback going eagerly
		self._dashboard_agg_sub = rospy.Subscriber('diagnostics_agg', diagnostic_msgs.msg.DiagnosticArray, self.dashboard_callback)

	def get_widgets(self):

		return [[MonitorDashWidget(self.context), ConsoleDashWidget(self.context)],self.breaker_board, self.motors,
                [self.lap_bat, self.create_bat]
		,self.buzzer
		,self.dock,
                [NavViewDashWidget(self.context)]]

	def dashboard_callback(self, msg):
		self._dashboard_message = msg
		self._last_dashboard_message_time = rospy.get_time()
  
		battery_status = {}  # Used to store TurtlebotBattery status info
		safety_status = {}
		serial_status = {}
	
		laptop_battery_status = {}
        
		for status in msg.status:
			print("Status callback %s"%status.name)
			if status.name == "/Power System/Battery Status":
				for value in status.values:


					if value.key == 'Percent':
						self.create_bat.update_perc(float(value.value)*100)
						#This should be self._last_dashboard_message_time?
                       				 # Is it even used graphically by the widget
						self.create_bat.update_time(float(value.value)*100)
					elif value.key == 'Charging state':
						if value.value == "Trickle charging" or value.value == "Full charging" or value.value == "Reconditioning":
							self.create_bat.set_charging(True)
						else:
							self.create_bat.set_charging(False)

			elif status.name == "/Power System/Laptop Battery":
				for value in status.values:
					laptop_battery_status[value.key]=value.value
			elif status.name == "/Breaker Board/Breaker Board Status":
				i=0
				for value in status.values:
					if value.value == "None":
						self.breaker_board[i].update_state(2)

					else:
						self.breaker_board[i].update_state(int(value.value))
					i+=1
		

        # If battery diagnostics were found, calculate percentages and stuff  
		if (laptop_battery_status):
			percentage = float(laptop_battery_status['Charge (Ah)'])/float(laptop_battery_status['Capacity (Ah)'])
			self.lap_bat.update_perc(percentage*100)
			self.lap_bat.update_time(percentage*100)
			charging_state = True if float(laptop_battery_status['Current (A)']) > 0.0 else False
			self.lap_bat.set_charging(charging_state)

	def shutdown_dashboard(self):
		self._dashboard_agg_sub.unregister()



