#!/usr/bin/env python

#####################################################################
# Software License Agreement (BSD License)
#
# Copyright (c) 2016, Michal Drwiega
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#####################################################################

import rospy
import logging
import sys
import time

#import Adafruit_ADS1x15 as ADS1x15
import ADS1x15 as Adafruit_ADS1x15
from time import time
from sensor_msgs.msg import Range, FluidPressure

from tf.transformations import quaternion_from_euler
from dynamic_reconfigure.server import Server
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

if __name__ == '__main__':
    rospy.init_node("adc_mux_node")

    busnum = rospy.get_param('~busnum', 0)
    frequency = rospy.get_param('~frequency', 15)
    gain = rospy.get_param('~gain', 1)
    frame_id_0 = rospy.get_param('~frame_id', 'ultrasonic_0_link')
    frame_id_2 = rospy.get_param('~frame_id', 'ultrasonic_2_link')
  #  adc = Adafruit_ADS1x15.ADS1115(busnum=busnum)
    adc = Adafruit_ADS1x15.ADS1115(busnum=busnum)

    ultra_data=Range()
    tank_data=FluidPressure()
    pub_ultra_0 = rospy.Publisher('ultrasonic_0', Range, queue_size=15)
    pub_ultra_2 = rospy.Publisher('ultrasonic_2', Range, queue_size=15)
    pub_tank = rospy.Publisher('tank', FluidPressure, queue_size=15)
    values = [0]*4

    rate = rospy.Rate(frequency)
    seq=0
    while not rospy.is_shutdown():
	
    	for i in range(4):       
            values[i] = adc.read_adc(i, gain=gain)
	
	time=rospy.Time.now()
        ultra_data.header.stamp=time
        ultra_data.header.frame_id=frame_id_0
        ultra_data.header.seq = seq
        ultra_data.radiation_type=0
        ultra_data.field_of_view=0.347
        ultra_data.min_range=0.152
        ultra_data.max_range=6.45
        ultra_data.range=values[0]/2401.0
        pub_ultra_0.publish(ultra_data)

        ultra_data.header.stamp=time
        ultra_data.header.frame_id=frame_id_2
        ultra_data.header.seq = seq
        ultra_data.radiation_type=0
        ultra_data.field_of_view=0.347
        ultra_data.min_range=0.152
        ultra_data.max_range=6.45
        ultra_data.range=values[2]/2401.0
        pub_ultra_2.publish(ultra_data)

        tank_data.header.stamp=time
        tank_data.header.frame_id='base_link'
        tank_data.header.seq=seq
        tank_data.fluid_pressure=values[3]
        tank_data.variance=0
        pub_tank.publish(tank_data)

	seq = seq + 1
        rate.sleep()


