#!/usr/bin/env python
import rospy, sys, math
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion, Vector3Stamped, Vector3

# Quaternions tools
import numpy as np
import tf.transformations as tr

# Threespace Python API
import threespace as ts_api

SENSORS = { 'chest':        0,
            'head':         1,
            'l_upper_arm':  8,
            'l_lower_arm':  9,
            'l_hand':       10,
            'r_upper_arm':  12,
            'r_lower_arm':  13,
            'r_hand':       14
          }

class ImuRawData(object):
  def __init__(self):
    # Read from parameter server
    self.com_port = self.read_parameter('~com_port', '/dev/ttyACM0')
    self.baudrate = self.read_parameter('~baudrate', 921600)    
    # Connect to the PVRSystem
    self.pvr_system = ts_api.PVRSystem(com_port=self.com_port, baudrate=self.baudrate)
    # Tare the sensors to [0,0,0,1]
    quat = [0,0,0,1]
    for sensor, id_number in SENSORS.items():
      self.pvr_system.tareWithCurrentOrientation(id_number, quat)
      rospy.loginfo('[%s] tared with: \t%s' % (sensor, quat))
    # Set-up publishers/subscribers
    self.imu_pub = dict()
    self.mag_pub = dict()
    for sensor in SENSORS:
      self.imu_pub[sensor] = rospy.Publisher('%s/imu/data_raw' % sensor, Imu)
      self.mag_pub[sensor] = rospy.Publisher('%s/imu/mag' % sensor, Vector3Stamped)
    # Initial values
  
  def run(self):
    while not rospy.is_shutdown():
      # Get and publish all the sensors data
      for sensor, id_number in SENSORS.items():
        imu_msg = Imu()
        mag_msg = Vector3Stamped()
        # Order: GyroRate[0:2], AccelerometerVector[3:5], CompassVector[6:8]
        #~ raw_data = self.pvr_system.getAllCorrectedComponentSensorData(id_number)
        tared_quat = self.pvr_system.getTaredOrientationAsQuaternion(id_number)
        #~ if raw_data and tared_quat:
        if tared_quat:
          imu_msg.orientation = Quaternion(*tared_quat)
          #~ imu_msg.angular_velocity = Vector3(*raw_data[0:3])
          #~ imu_msg.linear_acceleration = Vector3(*raw_data[3:6])
          #~ mag_msg.vector = Vector3(*raw_data[6:9])
          # Publish the data
          imu_msg.header.stamp = rospy.Time.now()
          imu_msg.header.frame_id = 'world'
          mag_msg.header = imu_msg.header
          self.mag_pub[sensor].publish(mag_msg)
          self.imu_pub[sensor].publish(imu_msg)
        

  def read_parameter(self, name, default):
    if not rospy.has_param(name):
      rospy.logwarn('Parameter [%s] not found, using default: %s' % (name, default))
    return rospy.get_param(name, default)
    

if __name__ == '__main__':
  rospy.init_node('imu_raw_data')
  rospy.loginfo('Starting the imu_raw_data node')
  raw = ImuRawData()
  raw.run()


