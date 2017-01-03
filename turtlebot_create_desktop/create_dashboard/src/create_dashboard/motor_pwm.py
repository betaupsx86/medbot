#from rqt_robot_dashboard.widgets import ButtonDashWidget
#
#class BreakerControl(ButtonDashWidget):
#   def __init__(self, context, name, index, cb = None, icon = None):
#        super(BreakerControl, self).__init__(context, name, cb, icon)
#
#       self.clicked.connect(self.toggle)
#        self._power_control = rospy.ServiceProxy('turtlebot_node/set_digital_outputs', create_node.srv.SetDigitalOutputs)

import rospy
import std_msgs
from functools import partial


from rqt_robot_dashboard.widgets import MenuDashWidget
from python_qt_binding.QtCore import QSize

class MotorWidget(MenuDashWidget):
    def __init__(self, topic):
        self._pub = rospy.Publisher(topic, Float32, queue_size=5)

        self._off_icon = ['bg-grey.svg', 'ic-breaker.svg']
        self._green_icon = ['bg-green.svg', 'ic-breaker.svg']
		self._green_icon = ['bg-yellow.svg', 'ic-breaker.svg']
        self._orange_icon = ['bg-orange.svg', 'ic-breaker.svg']
        self._red_icon = ['bg-red.svg', 'ic-breaker.svg']

        icons = [self._off_icon, self._red_icon, self._orange_icon, self._yellow_icon, self._green_icon]
        super(MotorWidget, self).__init__(topic, icons=icons)
        self.setFixedSize(QSize(40,40))

        self.add_action('Off', partial(self.update_state, 0))
        self.add_action('Red', partial(self.update_state, 1))
        self.add_action('Orange', partial(self.update_state, 2))
        self.add_action('Yellow', partial(self.update_state, 3))
        self.add_action('Green', partial(self.update_state, 4))
        
        self.setToolTip("LED: Off")

    def update_state(self, state):
        super(LedWidget, self).update_state(state)
        self._pub.publish(0.25*state)
        if state is 1:
            self.setToolTip("LED: Red")
        elif state is 2:
            self.setToolTip("LED: Orange")
        elif state is 3:
            self.setToolTip("LED: Yellow")
        elif state is 4:
            self.setToolTip("LED: Green")
        else:
            self.setToolTip("LED: Off")

    def close(self):
        self._pub.unregister()

