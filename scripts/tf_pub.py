#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
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
#
# Revision $Id$

## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic


import rospy
from std_msgs.msg import String

import serial
from sensor_msgs.msg import Imu
from time import time, sleep

import threespace as ts_api
import tf.transformations as tr


default_port='/dev/ttyACM0'
port = rospy.get_param('device', default_port)
ser = serial.Serial(port=port,baudrate=115200, timeout=1)

ser.write(">240,202\n")

line = ser.readline()

device_list = [ts_api.ComInfo(com_port='/dev/ttyACM0', friendly_name='PrioVR Base Station (ttyACM0)', dev_type='BS')]

com_port, name, dev_type = device_list[0]

pvr_base = ts_api.PVRSystem(com_port)







def talker():
    
    #part to logical id and nickname
    Chest = [0, "chest"]
    Head = [1, "head"]
    RightUpperArm = [12, "ru"]
    RightForearm = [13, "rf"]
    RightHand = [14, "rh"]

    partlist = [Chest, Head, RightUpperArm, RightForearm, RightHand]

    #make publisher for each
    pub = {}
    imuMsg = {}
    for id in partlist:  
        pub[id[0]] = rospy.Publisher(id[1], Imu)
        imuMsg[id[0]] = Imu()
        pvr_base.tareWithCurrentOrientation(id[0], [0,0,0,1])
    pub[10] = rospy.Publisher('diff', Imu)
    imuMsg[10] = Imu()

    #pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    r = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():

	#get values from PVR/publish
        for id in partlist:
            q = {}
            q[id[0]] = pvr_base.getUntaredOrientationAsQuaternion(id[0])
	    
	        #test for no value
            if not q[id[0]] == None:
                imuMsg[id[0]].orientation.x = q[id[0]][0] #magnetometer
                imuMsg[id[0]].orientation.y = q[id[0]][1]
                imuMsg[id[0]].orientation.z = q[id[0]][2]
                imuMsg[id[0]].orientation.w = q[id[0]][3]
                
                imuMsg[id[0]].header.stamp= rospy.Time.now()
                imuMsg[id[0]].header.frame_id = 'base_link'
                pub[id[0]].publish(imuMsg[id[0]])
        #print list(pvr_base.getUntaredOrientationAsQuaternion(0))
        chest = list(pvr_base.getUntaredOrientationAsQuaternion(0))
        rightup = list(pvr_base.getUntaredOrientationAsQuaternion(12))
        diffq = tr.quaternion_multiply(rightup, tr.quaternion_inverse(chest))
        imuMsg[10].orientation.x = diffq[0]
        imuMsg[10].orientation.y = diffq[1]
        imuMsg[10].orientation.z = diffq[2]
        imuMsg[10].orientation.w = diffq[3]
        imuMsg[10].header.stamp= rospy.Time.now()
        imuMsg[10].header.frame_id = 'base_link'
        pub[10].publish(imuMsg[10])
        
        r.sleep()

"""
	#test for no value
	if not (q == None or q1 == None):
		print q[0]

		
		imuMsg.orientation.x = q[0][0] #magnetometer
		imuMsg.orientation.y = q[0][1]
		imuMsg.orientation.z = q[0][2]
		imuMsg.orientation.w = q[0][3]

		#imuMsg1.orientation.x = q1[0] #magnetometer
		#imuMsg1.orientation.y = q1[1]
		#imuMsg1.orientation.z = q1[2]
		#imuMsg1.orientation.w = q1[3]

		imuMsg.header.stamp= rospy.Time.now()
		imuMsg.header.frame_id = 'base_link'
		#print imuMsg
		pub.publish(imuMsg)

		#imuMsg1.header.stamp= rospy.Time.now()
		#imuMsg1.header.frame_id = 'base_link'
		#print imuMsg
		#pub1.publish(imuMsg1)
"""
	#r.sleep()
"""
        str = line
        rospy.loginfo(str)
        pub.publish(str)
        r.sleep()
        
"""

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException: pass

