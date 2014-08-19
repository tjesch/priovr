#!/usr/bin/env python

import rospy, sys, math
# Messages
from geometry_msgs.msg import Vector3
from priovr_msgs.msg import ImuArray
# Threespace Python API
import threespace as ts_api

from priovr_interface.utils import *


# The Class
class SensorOrientations(object):
  def __init__(self):
    # Read from parameter server
    if not rospy.has_param('~sensors'):
      rospy.logerr('Parameter [~sensors] not found')
      sys.exit(1)
    self.frame_id = read_parameter('~frame_id', 'world')
    self.enable_joysticks = read_parameter('~enable_joysticks', False)
    self.com_port = read_parameter('~com_port', '/dev/ttyACM0')
    self.baudrate = read_parameter('~baudrate', 921600)
    self.sensors = rospy.get_param('~sensors', dict())
    # Connect to the PVRSystem
    rospy.loginfo('Connecting to the PVRSystem')
    try:
      self.pvr_system = ts_api.PVRSystem(com_port=self.com_port, baudrate=self.baudrate)
    except:
      rospy.logerr('Failed to connect to the PVRSystem. (Did you add user to the dialout group?)')
      sys.exit(1)
    # Validate sensors names and that they are connected
    for name, id_number in self.sensors.items():
      if name not in SENSOR_NAMES:
        rospy.logwarn('Invalid sensor name [/sensors/%s]' % name)
        del self.sensors[name]
      if not self.pvr_system.getAllRawComponentSensorData(id_number):
        rospy.logwarn('Sensor [/sensors/%s] not connected' % name)
        del self.sensors[name]
    rospy.loginfo('Connection to the PVRSystem succedded')
    # Set-up publishers/subscribers
    self.publisher = rospy.Publisher('/priovr/sensor_imus', ImuArray)
    if self.enable_joysticks:
      # Start streaming to receive the joysticks data
      self.pvr_system.startStreaming()
      # Shutdown hookup for stoping PVR streaming
      rospy.on_shutdown(self.shutdown)

  def shutdown(self):
    if self.pvr_system:
      self.pvr_system.stopStreaming()

  def run(self):
    while not rospy.is_shutdown():
      fail_sensor_read = False
      ros_msg = ImuArray()
      for name, id_number in self.sensors.items():
        q_tared = None
        # Get all the sensor imu's data
        try:
          # Order: GyroRate[0:2], AccelerometerVector[3:5], CompassVector[6:8]
          data = self.pvr_system.getAllCorrectedComponentSensorData(id_number)
        except:
          fail_sensor_read = True
          break
        if data:
          ros_msg.name.append(name)
          ros_msg.angular_velocity.append(Vector3(*data[0:3]))
          ros_msg.linear_acceleration.append(Vector3(*data[3:6]))
          ros_msg.compass.append(Vector3(*data[6:9]))
        else:
          rospy.logdebug('Failed getAllCorrectedComponentSensorData from [%s]' % (name))
          fail_sensor_read = True
          break
      # Skip reading if we missed any sensor reading
      if fail_sensor_read:
        continue
      # Publish the Orientations msg
      ros_msg.header.frame_id = self.frame_id
      ros_msg.header.stamp = rospy.Time.now()
      self.publisher.publish(ros_msg)


if __name__ == '__main__':
  node_name = 'sensor_imus'
  rospy.init_node(node_name)
  rospy.loginfo('Starting [%s] node' % node_name)
  so = SensorOrientations()
  so.run()
  rospy.loginfo('Shuting down [%s] node' % node_name)
