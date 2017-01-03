
import rospy
from functools import partial

from std_msgs.msg import UInt8

from rqt_robot_dashboard.widgets import MenuDashWidget
from python_qt_binding.QtCore import QSize

class BuzzerWidget(MenuDashWidget):
    def __init__(self, topic):
	self._pub = rospy.Publisher(topic, UInt8, queue_size=5)

	self._buzz_1 = ['bg-green.svg', 'ic-led.svg']
	self._buzz_2 = ['bg-yellow.svg', 'ic-led.svg']
	self._buzz_3 = ['bg-orange.svg', 'ic-led.svg']
	self._buzz_4 = ['bg-red.svg', 'ic-led.svg']
	self._buzz_5 = ['bg-red.svg', 'ic-led.svg']

	icons = [self._buzz_1, self._buzz_2, self._buzz_3, self._buzz_4, self._buzz_5]        
	super(BuzzerWidget, self).__init__(topic,icons=icons)
        self.setFixedSize(QSize(40,40))



        self.add_action('Sound 1', partial(self.update_state, 0))
        self.add_action('Sound 2', partial(self.update_state, 1))
        self.add_action('Sound 3', partial(self.update_state, 2))
        self.add_action('Tune 1', partial(self.update_state, 3))
        self.add_action('Tune 2', partial(self.update_state, 4))
        
    def update_state(self, state):
        super(BuzzerWidget, self).update_state(state)
        self._pub.publish(super(BuzzerWidget, self).state)
        
        if state is 0:
            self.setToolTip("Buzzer: Sound 1")
        elif state is 1:
            self.setToolTip("Buzzer: Sound 2")	
        elif state is 2:
            self.setToolTip("Buzzer: Sound 3")
        elif state is 3:
            self.setToolTip("Buzzer: Tune 1")
        else:
            self.setToolTip("Buzzer: Tune 2")

    def close(self):
        self._pub.unregister()

