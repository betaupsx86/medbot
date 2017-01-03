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


from Adafruit_BNO055 import BNO055
from time import time
from sensor_msgs.msg import Imu, Temperature, MagneticField

from tf.transformations import quaternion_from_euler
from dynamic_reconfigure.server import Server
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue


# Main function
if __name__ == '__main__':
    rospy.init_node("bosch_imu_node")

    # Sensor measurements publishers
    pub_data = rospy.Publisher('imu/data', Imu, queue_size=30)
    pub_raw = rospy.Publisher('imu/raw', Imu, queue_size=1)
    pub_mag = rospy.Publisher('imu/mag', MagneticField, queue_size=1)
    pub_temp = rospy.Publisher('imu/temp', Temperature, queue_size=1)

    # srv = Server(imuConfig, reconfig_callback)  # define dynamic_reconfigure callback

    # Get parameters values
    busnum = rospy.get_param('~busnum', 1)
    rst = rospy.get_param('~rst', 7)
    frame_id = rospy.get_param('~frame_id', 'imu_link')
    frequency = rospy.get_param('~frequency', 100)
    operation_mode = rospy.get_param('~operation_mode', 12)
    calibration = rospy.get_param('~calibration', [251, 255, 223, 255, 2, 0, 253, 254, 5, 0, 188, 0, 0, 0, 255, 255, 0, 0, 232, 3, 202, 2])

    bno = BNO055.BNO055(rst=rst,busnum=busnum)

    if len(sys.argv) == 2 and sys.argv[1].lower() == '-v':
        logging.basicConfig(level=logging.DEBUG)

    if not bno.begin(mode=operation_mode):
    	raise RuntimeError('Failed to initialize BNO055! Is the sensor connected?')

    # Print system status and self test result.
    status, self_test, error = bno.get_system_status()
    print('System status: {0}'.format(status))
    print('Self test result (0x0F is normal): 0x{0:02X}'.format(self_test))
    # Print out an error if system status is in error mode.
    if status == 0x01:
        print('System error: {0}'.format(error))
        print('See datasheet section 4.3.59 for the meaning.')

    # Print BNO055 software revision and other diagnostic data.
    sw, bl, accel, mag, gyro = bno.get_revision()
    print('Software version:   {0}'.format(sw))
    print('Bootloader version: {0}'.format(bl))
    print('Accelerometer ID:   0x{0:02X}'.format(accel))
    print('Magnetometer ID:    0x{0:02X}'.format(mag))
    print('Gyroscope ID:       0x{0:02X}\n'.format(gyro))

    imu_data = Imu()            # Filtered data
    imu_raw = Imu()             # Raw IMU data
    temperature_msg = Temperature() # Temperature
    mag_msg = MagneticField()       # Magnetometer data
    seq=0    
    
    rate = rospy.Rate(frequency)
    bno.set_calibration(calibration)
    calib=bno.get_calibration()
	
#    init_sys, init_gyro, init_accel, init_mag = bno.get_calibration_status()
#    if not init_gyro == init_accel == init_mag == 3:
#        bno.set_calibration(calibration)
    while not rospy.is_shutdown():

	# Publish raw data
        heading, roll, pitch = bno.read_euler()
	sys, gyro, accel, mag = bno.get_calibration_status()
        if not gyro == accel == 3:
             rospy.logwarn("[BOSCH_BNO055] Not fully calibrated. Please terminate and run calibration routine")
        print('Heading={0:0.2F} Roll={1:0.2F} Pitch={2:0.2F}\tSys_cal={3} Gyro_cal={4} Accel_cal={5} Mag_cal={6}'.format(
        heading, roll, pitch, sys, gyro, accel, mag))    
		
#	print('Heading={0:0.2F} Roll={1:0.2F} Pitch={2:0.2F}\tSys_cal={3} Gyro_cal={4} Accel_cal={5} Mag_cal={6}'.format(
#           heading, roll, pitch, sys, gyro, accel, mag))
	#print(bno.get_calibration())
	imu_raw.header.stamp = rospy.Time.now()
	imu_raw.header.frame_id = frame_id
	imu_raw.header.seq = seq
	imu_raw.orientation_covariance[0] = -1
	imu_raw.linear_acceleration.x = float(bno.read_accelerometer()[0]) 
	imu_raw.linear_acceleration.y = float(bno.read_accelerometer()[1]) 
	imu_raw.linear_acceleration.z = float(bno.read_accelerometer()[2]) 
	imu_raw.linear_acceleration_covariance[0] = -1
	imu_raw.angular_velocity.x = float(bno.read_gyroscope()[0])
	imu_raw.angular_velocity.y = float(bno.read_gyroscope()[1])
	imu_raw.angular_velocity.z = float(bno.read_gyroscope()[2])
	imu_raw.angular_velocity_covariance[0] = -1
	pub_raw.publish(imu_raw)

            #            print("read: ", binascii.hexlify(buf), "acc = (",imu_data.linear_acceleration.x,
            #                  imu_data.linear_acceleration.y, imu_data.linear_acceleration.z, ")")


	imu_data.header.stamp = rospy.Time.now()
	imu_data.header.frame_id = frame_id
	imu_data.header.seq = seq

         # Publish filtered data
	imu_data.orientation.x = 0.0
	imu_data.orientation.y = 0.0
#	imu_data.orientation.x = float(bno.read_quaternion()[0])
#	imu_data.orientation.y = float(bno.read_quaternion()[1])
	imu_data.orientation.z = float(bno.read_quaternion()[2])
	imu_data.orientation.w = float(bno.read_quaternion()[3])
	imu_data.linear_acceleration.x = float(bno.read_linear_acceleration()[0])
	imu_data.linear_acceleration.y = float(bno.read_linear_acceleration()[1])
	imu_data.linear_acceleration.z = float(bno.read_linear_acceleration()[2])
#	imu_data.linear_acceleration.x = float(bno.read_accelerometer()[0]) 
#	imu_data.linear_acceleration.y = float(bno.read_accelerometer()[1]) 
#	imu_data.linear_acceleration.z = float(bno.read_accelerometer()[2]) 
	#imu_data.linear_acceleration_covariance[0] = -1
	imu_data.angular_velocity.x = float(bno.read_gyroscope()[0]*0.0174533)
	imu_data.angular_velocity.y = float(bno.read_gyroscope()[1]*0.0174533)
	imu_data.angular_velocity.z = float(bno.read_gyroscope()[2]*0.0174533)
	#imu_data.angular_velocity_covariance[0] = -1
	pub_data.publish(imu_data)

	#Publish magnetometer data
	mag_msg.header.stamp = rospy.Time.now()
	mag_msg.header.frame_id = frame_id
	mag_msg.header.seq = seq
	mag_msg.magnetic_field.x = float(bno.read_magnetometer()[0])
	mag_msg.magnetic_field.y = float(bno.read_magnetometer()[1])
	mag_msg.magnetic_field.z = float(bno.read_magnetometer()[2])
	pub_mag.publish(mag_msg)

	#Publish temperature
	temperature_msg.header.stamp = rospy.Time.now()
	temperature_msg.header.frame_id = frame_id
	temperature_msg.header.seq = seq
	temperature_msg.temperature = bno.read_temp()
	pub_temp.publish(temperature_msg)
	#print(bno.get_calibration())
	seq = seq + 1
        rate.sleep()

