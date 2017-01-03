#!/usr/bin/env python

import rospy
from breaker_board_node.srv import *

service = 'set_breaker_output'

def turn_on_kinect():
    rospy.init_node('turn_on_kinect')
    rospy.wait_for_service(service)

    while True:
        service_proxy = rospy.ServiceProxy(service, SetBreakerOutputs)
        response = service_proxy(0,0)
        if response.done:
            break
        else:
            rospy.sleep(3)
            continue

if __name__ == '__main__':
    try:
        turn_on_kinect()
    except rospy.ROSInterruptException: pass
    except IOError: pass
