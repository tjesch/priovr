#!/usr/bin/env python
import rospy, serial, sys, math
# Messages
from baxter_core_msgs.msg import JointCommand
# Quaternions tools
import numpy as np
import tf.transformations as tr
# Threespace Python API
import threespace as ts_api


LINK_NAMES = ['chest','head','l_upper_arm','l_lower_arm','l_hand','r_upper_arm','r_lower_arm','r_hand']

LINKS = { 'hips':         {'parent':None},
          'chest':        {'parent':'hips'},
          'head':         {'parent':'chest'},
          'l_upper_arm':  {'parent':'chest'},
          'l_lower_arm':  {'parent':'l_upper_arm'},
          'l_hand':       {'parent':'l_lower_arm'},
          'r_upper_arm':  {'parent':'chest'},
          'r_lower_arm':  {'parent':'r_upper_arm'},
          'r_hand':       {'parent':'r_lower_arm'}
        }
JOINTS =  { 'spine':      {'parent':'hips',        'child':'chest'},
            'neck':       {'parent':'chest',        'child':'head'},
            'l_shoulder': {'parent':'chest',        'child':'l_upper_arm'},
            'l_elbow':    {'parent':'l_upper_arm',  'child':'l_lower_arm'},
            'l_wrist':    {'parent':'l_lower_arm',  'child':'l_hand'},
            'r_shoulder': {'parent':'chest',        'child':'r_upper_arm'},
            'r_elbow':    {'parent':'r_upper_arm',  'child':'r_lower_arm'},
            'r_wrist':    {'parent':'r_lower_arm',  'child':'r_hand'}
          }

class ImuJointController(object):
  def __init__(self):
    # Read from parameter server
    if not rospy.has_param('~mapping'):
      rospy.logerr('Parameter [~mapping] not found')
      sys.exit(1)
    if not rospy.has_param('~sensors'):
      rospy.logerr('Parameter [~sensors] not found')
      sys.exit(1)
    self.sensors = rospy.get_param('~sensors', dict())
    self.com_port = self.read_parameter('~com_port', '/dev/ttyACM0')
    self.baudrate = self.read_parameter('~baudrate', 921600)
    self.tare_quaternions = self.read_parameter('~tare_quaternions', dict())
    self.mapping = rospy.get_param('~mapping', dict())
    # TODO: Should define all the sensors?
    # Validate sensors names
    for key in self.sensors.keys():
      if key not in LINK_NAMES:
        rospy.logwarn('Invalid sensor name [/sensors/%s]' % key)
        del self.sensors[key]
    # Check consistency of the tare_quaternions dictionary
    for key in self.tare_quaternions.keys():
      if key not in self.sensors.keys():
        rospy.logwarn('Invalid tare quaternion [%s]' % key)
    # TODO: This consistency should validate complete trees?
    # Check consistency of the mapping dictionary
    for key in self.mapping.keys():
      if key not in JOINTS.keys():
        rospy.logwarn('Invalid joint name [/mapping/%s]' % key)
    
    # Connect to the PVRSystem
    self.pvr_system = ts_api.PVRSystem(com_port=self.com_port, baudrate=self.baudrate)
    # Tare the sensors defined in tare_quaternions
    for name, quat in self.tare_quaternions.items():
      id_number = self.sensors[name]
      self.pvr_system.tareWithCurrentOrientation(id_number, quat)
      rospy.loginfo('[%s] tared with: %s' % (name, quat))
    
    # Set-up publishers/subscribers
    self.left_arm = rospy.Publisher('/robot/limb/left/joint_command', JointCommand)
    self.right_arm = rospy.Publisher('/robot/limb/right/joint_command', JointCommand)
    
    # Initialize dictionaries
    self.raw_orientations = dict()
    self.link_orientations = dict()
    self.joint_orientations = dict()

  def cb_state_publisher(self, event):
    pass
  
  def run(self):
    while not rospy.is_shutdown():
      # Get all the sensors orientations
      self.link_orientations['hips'] = [0,0,0,1]
      for name in LINK_NAMES:
        id_number = self.sensors[name]
        q_raw = self.pvr_system.getTaredOrientationAsQuaternion(id_number)
        self.raw_orientations[name] = q_raw
        if q_raw:
          parent = LINKS[name]['parent']
          q_parent = self.link_orientations[parent]
          self.link_orientations[name] =  tr.quaternion_multiply(q_parent, q_raw)
        else:
          rospy.logwarn('Failed to read quaternion from [%s]' % (name))
      
      if None in self.raw_orientations.values():
        continue
      
      # Define command messages
      left_msg = JointCommand()
      right_msg = JointCommand()
      left_msg.mode = JointCommand().POSITION_MODE
      right_msg.mode = JointCommand().POSITION_MODE
      
      for name in JOINTS.keys():
        parent = JOINTS[name]['parent']
        child = JOINTS[name]['child']
        q_parent = self.link_orientations[parent]
        q_child = self.link_orientations[child]
        self.joint_orientations[name] = tr.quaternion_multiply(q_parent, tr.quaternion_inverse(q_child))
        rpy = list(tr.euler_from_quaternion(self.joint_orientations[name], 'sxyz'))
        for i,joint in enumerate(self.mapping[name]):
          if joint == 'no_joint':
            continue
          joint_name = joint
          if '-' == joint[0]:
            rpy[i] *= -1.0
            joint_name = joint[1:]
          # Populate command messages
          if 'left_' in joint_name:
            left_msg.names.append(joint_name)
            left_msg.command.append(rpy[i])
          elif 'right_' in joint_name:
            right_msg.names.append(joint_name)
            right_msg.command.append(rpy[i])
      
      # Publish commands
      self.left_arm.publish(left_msg)
      self.right_arm.publish(right_msg)

  def read_parameter(self, name, default):
    if not rospy.has_param(name):
      rospy.logwarn('Parameter [%s] not found, using default: %s' % (name, default))
    return rospy.get_param(name, default)
    

if __name__ == '__main__':
  rospy.init_node('imu_joint_controller')
  imu_jc = ImuJointController()
  imu_jc.run()


