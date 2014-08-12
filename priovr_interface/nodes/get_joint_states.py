#!/usr/bin/env python

import rospy, sys, math
# Messages
from sensor_msgs.msg import JointState
from priovr_msgs.msg import QuaternionArray
# Quaternions tools
import numpy as np

from priovr_interface.utils import *

NO_JOINT = 'no_joint'


# The Class
class GetJointStates(object):
  def __init__(self):
    # Read from parameter server
    if not rospy.has_param('~mapping'):
      rospy.logerr('Parameter [~mapping] not found')
      sys.exit(1)
    self.frame_id = read_parameter('~frame_id', 'world')
    self.publish_rate = read_parameter('~publish_rate', 100)
    self.mapping = rospy.get_param('~mapping', dict())
    # Check consistency of the mapping dictionary
    for joint in self.mapping.keys():
      invalid_joint = joint not in HUMAN_JOINTS.keys()
      if invalid_joint:
        rospy.logwarn('Invalid joint defined in [/mapping/%s]' % joint)
        del self.mapping[joint]
    # Set-up publishers/subscribers
    self.state_pub = rospy.Publisher('/priovr/joint_states', JointState)
    rospy.Subscriber('/priovr/sensor_orientations', QuaternionArray, self.orientations_cb)
    # Initialize dictionaries
    self.link_orientations = dict()
    self.joint_orientations = dict()
    # Start the timer that will publish the joint states
    self.timer = rospy.Timer(rospy.Duration(1.0/self.publish_rate), self.publish_state)
    # Shutdown hookup for stoping the publish timer
    rospy.on_shutdown(self.shutdown)
    # Spin!
    rospy.spin()

  def orientations_cb(self, msg):
    self.link_orientations = dict()
    for i, name in enumerate(msg.name):
      self.link_orientations[name] = quaternion_to_array(msg.quaternion[i])

  def publish_state(self, event):
    # Populate the JointState msg
    state_msg = JointState()
    for name in self.mapping.keys():
      parent = HUMAN_JOINTS[name]['parent']
      child = HUMAN_JOINTS[name]['child']
      if not self.link_orientations.has_key(parent) or not self.link_orientations.has_key(child):
        continue
      q_parent = self.link_orientations[parent]
      q_child = self.link_orientations[child]
      # Split it in 3 DoF (roll, pitch and yaw)
      # TODO: Fucking Euler angles!!!
      rpy = calculate_joint_angles(q_parent, q_child)
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
    state_msg.header.frame_id = self.frame_id
    state_msg.header.stamp = rospy.Time.now()
    self.state_pub.publish(state_msg)

  def shutdown(self):
    # Stop the publisher timer
    self.timer.shutdown()


if __name__ == '__main__':
  node_name = 'get_joint_states'
  rospy.init_node(node_name)
  rospy.loginfo('Starting [%s] node' % node_name)
  get_js = GetJointStates()
  rospy.loginfo('Shuting down [%s] node' % node_name)
