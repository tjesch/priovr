#!/usr/bin/env python
# General imports
import rospy, sys, math
# Messages
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Quaternion
# Quaternions tools
import numpy as np
import tf.transformations as tr
# Threespace Python API
import threespace as ts_api


SENSOR_NAMES = ['chest','head','l_upper_arm','l_lower_arm','l_hand','r_upper_arm','r_lower_arm','r_hand']

JOINTS =  { 'spine':      {'parent':'hips',        'child':'chest'},
            'neck':       {'parent':'chest',        'child':'head'},
            'l_shoulder': {'parent':'chest',        'child':'l_upper_arm'},
            'l_elbow':    {'parent':'l_upper_arm',  'child':'l_lower_arm'},
            'l_wrist':    {'parent':'l_lower_arm',  'child':'l_hand'},
            'r_shoulder': {'parent':'chest',        'child':'r_upper_arm'},
            'r_elbow':    {'parent':'r_upper_arm',  'child':'r_lower_arm'},
            'r_wrist':    {'parent':'r_lower_arm',  'child':'r_hand'}
          }

NO_JOINT = 'no_joint'


# Helper functions
def detect_present_sensors():
  pvr_system = ts_api.PVRSystem(com_port='/dev/ttyACM0', baudrate=921600)
  found = []
  for id_number in range(20):
    if pvr_system.getAllRawComponentSensorData(id_number):
      found.append(id_number)
  return found


# The Class
class GetJointStates(object):
  def __init__(self):
    # Read from parameter server
    if not rospy.has_param('~sensors'):
      rospy.logerr('Parameter [~sensors] not found')
      sys.exit(1)
    if not rospy.has_param('~mapping'):
      rospy.logerr('Parameter [~mapping] not found')
      sys.exit(1)
    self.tare_quaternions = self.read_parameter('~tare_quaternions', dict())
    self.reference_frame = self.read_parameter('~reference_frame', 'world')
    self.com_port = self.read_parameter('~com_port', '/dev/ttyACM0')
    self.baudrate = self.read_parameter('~baudrate', 921600)
    self.mapping = rospy.get_param('~mapping', dict())
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
    # Check consistency of the mapping dictionary
    for sensor_name in self.mapping.keys():
      if sensor_name not in JOINTS.keys():
        rospy.logwarn('Invalid priovr sensor name [/mapping/%s]' % sensor_name)
        del self.mapping[sensor_name]
    # Check consistency of the tare_quaternions dictionary
    for key in self.tare_quaternions.keys():
      if key not in self.sensors.keys():
        rospy.logwarn('Deleting tare quaternion for [%s]' % key)
        del self.tare_quaternions[key]
    # Tare the sensors defined in tare_quaternions
    for name, quat in self.tare_quaternions.items():
      id_number = self.sensors[name]
      self.pvr_system.tareWithCurrentOrientation(id_number, quat)
      rospy.loginfo('[%s] tared with: %s' % (name, quat))
    
    # Set-up publishers/subscribers
    self.state_pub = rospy.Publisher('joint_states', JointState)
    
    # Initialize dictionaries
    self.link_orientations = dict()
    self.joint_orientations = dict()
  
  def run(self):
    while not rospy.is_shutdown():
      fail_sensor_read = False
      self.link_orientations['hips'] = [0,0,0,1]
      for name, id_number in self.sensors.items():
        # Get the tared orientation
        try:
          q_tared = self.pvr_system.getTaredOrientationAsQuaternion(id_number)
        except:
          fail_sensor_read = True
          break
        if q_tared:
          self.link_orientations[name] = q_tared
        else:
          rospy.logwarn('Failed to read quaternion from [%s]' % (name))
          fail_sensor_read = True
      
      # Skip reading if we missed any sensor reading
      if fail_sensor_read:
        continue
      
      # Populate the JointState msg
      state_msg = JointState()
      for name in self.mapping.keys():
        parent = JOINTS[name]['parent']
        child = JOINTS[name]['child']
        q_parent = self.link_orientations[parent]
        q_child = self.link_orientations[child]
        # This orientation is the relative between parent and child
        self.joint_orientations[name] = tr.quaternion_multiply(tr.quaternion_inverse(q_parent), q_child)
        # Split it in 3 DoF (roll, pitch and yaw)
        rpy = list(tr.euler_from_quaternion(self.joint_orientations[name], 'rxyz'))
        for i,joint in enumerate(self.mapping[name]):
          if joint == 'no_joint':
            continue
          joint_name = joint
          # Invert sign if required in the mapping dictionary
          if '-' == joint[0]:
            rpy[i] *= -1.0
            joint_name = joint[1:]
          state_msg.position.append(rpy[i])
          state_msg.name.append(joint_name)
      # Publish the JointState msg
      state_msg.header.frame_id = self.reference_frame
      state_msg.header.stamp = rospy.Time.now()
      self.state_pub.publish(state_msg)

  def read_parameter(self, name, default):
    if not rospy.has_param(name):
      rospy.logwarn('Parameter [%s] not found, using default: %s' % (name, default))
    return rospy.get_param(name, default)
    

if __name__ == '__main__':
  rospy.init_node('get_joint_states')
  get_js = GetJointStates()
  get_js.run()


