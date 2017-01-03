import rospy
from breaker_board_node.srv import *

from functools import partial
from rqt_robot_dashboard.widgets import IconToolButton
from python_qt_binding.QtCore import QSize


class BreakerButton(IconToolButton):
	def __init__(self, name, pin):
		self._on_icon = ['bg-green.svg', 'ic-breaker.svg']
		self._off_icon = ['bg-red.svg', 'ic-breaker.svg']
		self._stale_icon = ['bg-grey.svg', 'ic-breaker.svg']
    
		icons = [self._on_icon, self._off_icon,  self._stale_icon]
		super(BreakerButton, self).__init__(name, icons=icons)
		self.setFixedSize(self._icons[0].actualSize(QSize(50,30)))

		super(BreakerButton, self).update_state(2)
		self.setToolTip("Breaker: Not used")
		self.clicked.connect(self.toggle_breaker)

		self.pin=pin
		self.service_proxy = rospy.ServiceProxy('set_breaker_output', SetBreakerOutputs)


	def toggle_breaker(self):
		if super(BreakerButton, self).state is not 2:
			try:
				response=self.service_proxy(self.pin, not super(BreakerButton, self).state)
				return response.done

			except rospy.ServiceException, e:
				self.message = QMessageBox()
				self.message.setText("Service call failed with error: %s"%(e))
				self.message.exec_()
				return False
          
	def update_state(self, state):
		if state is not super(BreakerButton, self).state:
			super(BreakerButton, self).update_state(state)
		if state is 1 :
			self.setToolTip("Breaker: Off")
		elif state is 0:
			self.setToolTip("Breaker: On")
		else:
			self.setToolTip("Breaker: Not used")


