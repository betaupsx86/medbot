#!/usr/bin/env python

import rospy
from breaker_board_node.srv import *

service = 'set_breaker_output'

def turn_on_fan():
    rospy.init_node('turn_on_fan')
    rospy.wait_for_service(service)

    while True:
        service_proxy = rospy.ServiceProxy(service, SetBreakerOutputs)
        response = service_proxy(4,0)
        if response.done:
            break
        else:
            rospy.sleep(3)
            continue

if __name__ == '__main__':
    try:
        turn_on_fan()
    except rospy.ROSInterruptException: pass
    except IOError: pass
