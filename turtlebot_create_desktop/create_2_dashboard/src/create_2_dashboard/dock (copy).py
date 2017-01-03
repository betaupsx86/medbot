import rospy

from functools import partial

from std_msgs.msg import Empty

from rqt_robot_dashboard.widgets import IconToolButton
from python_qt_binding.QtCore import QSize

class DockingWidget(IconToolButton):
    def __init__(self, topic):
        self._pub = rospy.Publisher(topic, Float32, queue_size=5)

        self._off_icon = ['bg-red.svg', 'ic-breaker.svg']
        self._on_icon = ['bg-green.svg', 'ic-breaker.svg']
        
        icons = [self._off_icon, self._on_icon]
        super(DockingWidget, self).__init__(topic, icons=icons)
        self.setFixedSize(QSize(40,40))

        super(DockingWidget, self).update_state(0)
        self.setToolTip("Docking: Off")

        self.clicked.connect(self.toggle)


    def update_state(self, state):
        if state is not super(DockingWidget, self).state:
            super(DockingWidget, self).update_state(state)
            if state is 0:
                self.setToolTip("Docking: Off")
            else:
                self.setToolTip("Docking: On")

    def toggle(self):	
        if super(DockingWidget, self).state is 1:
            self._pub.publish(0.0)
            super(DockingWidget, self).update_state(0)
            self.setToolTip("Docking: Off")
        else:
            self._pub.publish(1.0)
            super(DockingWidget, self).update_state(1)
            self.setToolTip("Docking: On")

    def close(self):
        self._pub.unregister()

