#!/usr/bin/env python

import rospy, sys, math
# Messages
from sensor_msgs.msg import JointState
from priovr_msgs.msg import ImuArray, QuaternionArray
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
    rospy.Subscriber('/priovr/sensor_imus', ImuArray, self.imus_cb)
    # Initialize dictionaries
    self.link_orientations = dict()
    self.angular_velocity = dict()
    # Start the timer that will publish the joint states
    self.timer = rospy.Timer(rospy.Duration(1.0/self.publish_rate), self.publish_state)
    # Shutdown hookup for stoping the publish timer
    rospy.on_shutdown(self.shutdown)
    # Spin!
    rospy.spin()
  
  def imus_cb(self, msg):
    self.angular_velocity = dict()
    for i, name in enumerate(msg.name):
      self.angular_velocity[name] = vector_to_array(msg.angular_velocity[i])

  def orientations_cb(self, msg):
    self.link_orientations = dict()
    for i, name in enumerate(msg.name):
      self.link_orientations[name] = quaternion_to_array(msg.quaternion[i])

  def publish_state(self, event):
    # Lock a copy to avoid using deleted objects
    sensor_orientations = dict(self.link_orientations)
    angular_velocity = dict(self.angular_velocity)
    # Populate the JointState msg
    state_msg = JointState()
    for name in self.mapping.keys():
      parent = HUMAN_JOINTS[name]['parent']
      child = HUMAN_JOINTS[name]['child']
      position_available = sensor_orientations.has_key(parent) and sensor_orientations.has_key(child)
      velocity_available = angular_velocity.has_key(parent) and angular_velocity.has_key(child)
      # Noting available
      if not (position_available or velocity_available):
        continue
      if position_available:
        q_parent = sensor_orientations[parent]
        q_child = sensor_orientations[child]
        # Split it in 3 DoF (roll, pitch and yaw)
        # TODO: Fucking Euler angles!!!
        rpy = calculate_joint_angles(q_parent, q_child)
      else:
        rpy = np.zeros(3)
      if velocity_available:
        ang_vel = angular_velocity[parent] - angular_velocity[child]
      else:
        ang_vel = np.zeros(3)
      for i,joint in enumerate(self.mapping[name]):
        if joint == 'no_joint':
          continue
        joint_name = joint
        # Invert sign if required in the mapping dictionary
        if '-' == joint[0]:
          rpy[i] *= -1.0
          ang_vel[i] *= -1.0
          joint_name = joint[1:]
        state_msg.position.append(rpy[i])
        state_msg.velocity.append(ang_vel[i])
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
