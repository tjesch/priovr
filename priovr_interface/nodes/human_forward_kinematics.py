#!/usr/bin/env python

import rospy, sys, math
# Messages
from geometry_msgs.msg import Point, Pose, PoseArray, PoseStamped
from priovr_msgs.msg import QuaternionArray
from visualization_msgs.msg import Marker
# Quaternions tools
import numpy as np
import PyKDL
from tf_conversions import posemath

from priovr_interface.utils import *

# The Class
class PriovrFK(object):
  def __init__(self):
    # Read from parameter server
    self.frame_id = read_parameter('~frame_id', 'world')
    self.publish_rate = read_parameter('~publish_rate', 100)
    # Set-up publishers/subscribers
    self.pose_pub = dict()
    for joint in JOINT_NAMES:
      self.pose_pub[joint] = rospy.Publisher('/priovr/%s_pose' % joint, PoseStamped)
    self.vis_pub = rospy.Publisher('visualization_marker', Marker)
    rospy.Subscriber('/priovr/sensor_orientations', QuaternionArray, self.orientations_cb)
    # Initial values
    self.scale = 2.6
    self.sensor_orientations = dict()
    self.frames = dict()
    # Start the timer that will publish the joint states
    self.timer = rospy.Timer(rospy.Duration(1.0/self.publish_rate), self.publish_state)
    # Shutdown hookup for stoping the publish timer
    rospy.on_shutdown(self.shutdown)
    # Spin!
    rospy.spin()

  def draw_robot(self):
    frames = dict(self.frames)
    marker = Marker()
    marker.id = 0;
    marker.type = marker.LINE_LIST
    marker.ns = 'skeleton'
    marker.action = marker.ADD
    # Populate the points with the frames positions
    marker.points.append(Point(0,0,0))   # Origin
    for joint in JOINT_NAMES:
      if self.frames.has_key(joint):
        # It needs to be added twice because it will draw a line 
        # between each pair of points, so 0-1, 2-3, 4-5, ...
        marker.points.append(Point(*frames[joint].p))
        marker.points.append(Point(*frames[joint].p))
    marker.points.pop()
    # Appearance Settings
    marker.scale.x = 0.01
    marker.color.a = 0.5
    marker.color.r = 1.0
    marker.color.g = 1.0
    marker.color.b = 0.2
    # Publish
    marker.header.frame_id = self.frame_id
    marker.header.stamp = rospy.Time.now()
    self.vis_pub.publish(marker)

  def orientations_cb(self, msg):
    # Populate sensor orientations dict
    self.sensor_orientations = dict()
    for i, name in enumerate(msg.name):
      self.sensor_orientations[name] = kdl_quaternion_from_msg(msg.quaternion[i])
    # Build Frames
    for joint in JOINT_NAMES:
      parent = HUMAN_JOINTS[joint]['parent']
      child = HUMAN_JOINTS[joint]['child']
      p_joint = HUMAN_JOINTS[joint]['parent_joint']
      if (parent in msg.name) and (child in msg.name):
        q_parent = self.sensor_orientations[parent]
        q_child = self.sensor_orientations[child]
        rot = BONE_RATIOS[child][1] * q_parent.Inverse() * q_child * BONE_RATIOS[child][2]
        pos = BONE_RATIOS[child][0] * self.scale
        self.frames[joint] = PyKDL.Frame(rot, pos)
        if p_joint:
          self.frames[joint] = self.frames[p_joint] * self.frames[joint]

  def publish_state(self, event):
    for joint, frame in self.frames.items():
      msg = PoseStamped()
      # Populate the msg
      msg.pose = posemath.toMsg(frame)
      # Publish the msg
      msg.header.frame_id = self.frame_id
      msg.header.stamp = rospy.Time.now()
      self.pose_pub[joint].publish(msg)
    # Draw the skeleton
    self.draw_robot()

  def shutdown(self):
    # Stop the publisher timer
    self.timer.shutdown()


if __name__ == '__main__':
  node_name = 'human_forward_kinematics'
  rospy.init_node(node_name)
  rospy.loginfo('Starting [%s] node' % node_name)
  pfk = PriovrFK()
  rospy.loginfo('Shuting down [%s] node' % node_name)
