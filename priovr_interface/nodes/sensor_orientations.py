#!/usr/bin/env python

import rospy, sys, math
# Messages
from geometry_msgs.msg import Quaternion
from priovr_msgs.msg import QuaternionArray
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
    self.tare_quaternions = read_parameter('~tare_quaternions', dict())
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
    # Check consistency of the tare_quaternions dictionary
    for key in self.tare_quaternions.keys():
      if key not in self.sensors.keys():
        rospy.logdebug('Deleting tare quaternion for [%s]' % key)
        del self.tare_quaternions[key]
    # Tare the sensors defined in tare_quaternions
    for name, quat in self.tare_quaternions.items():
      id_number = self.sensors[name]
      self.pvr_system.tareWithCurrentOrientation(id_number, quat)
      rospy.logdebug('[%s] tared with: %s' % (name, quat))
    # Set-up publishers/subscribers
    self.orientations_pub = rospy.Publisher('/priovr/sensor_orientations', QuaternionArray)
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
      quat_array_msg = QuaternionArray()
      for name, id_number in self.sensors.items():
        q_tared = None
        # Get the tared orientation
        try:
          q_tared = self.pvr_system.getTaredOrientationAsQuaternion(id_number)
        except:
          fail_sensor_read = True
          break
        if q_tared:
          quat_array_msg.name.append(name)
          quat_array_msg.quaternion.append(array_to_quaternion(q_tared))
        else:
          rospy.logdebug('Failed getTaredOrientationAsQuaternion from [%s]' % (name))
          fail_sensor_read = True
          break
      # Skip reading if we missed any sensor reading
      if fail_sensor_read:
        continue
      # Append the hips orientations
      quat_array_msg.name.append('hips')
      quat_array_msg.quaternion.append(array_to_quaternion([0,0,0,1]))
      # Publish the Orientations msg
      quat_array_msg.header.frame_id = self.frame_id
      quat_array_msg.header.stamp = rospy.Time.now()
      self.orientations_pub.publish(quat_array_msg)


if __name__ == '__main__':
  node_name = 'sensor_orientations'
  rospy.init_node(node_name)
  rospy.loginfo('Starting [%s] node' % node_name)
  so = SensorOrientations()
  so.run()
  rospy.loginfo('Shuting down [%s] node' % node_name)
