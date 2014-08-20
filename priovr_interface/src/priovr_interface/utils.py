#! /usr/bin/env python

import rospy, math
# Messages
from geometry_msgs.msg import Quaternion
# Quaternions tools
import numpy as np
import tf.transformations as tr
import PyKDL
from PyKDL import Rotation, Vector, dot


### VARIABLES ###

# Joysticks
LEFT_Z_BUTTON = 0
LEFT_C_BUTTON = 1
RIGHT_Z_BUTTON = 2
RIGHT_C_BUTTON = 3
LEFT_X_AXIS = 0
LEFT_Y_AXIS = 1
RIGHT_X_AXIS = 2
RIGHT_Y_AXIS = 3

HUMAN_JOINTS = {'spine':      {'parent':'hips',         'child':'chest',        'parent_joint': None},
                'neck':       {'parent':'chest',        'child':'head',         'parent_joint': 'spine'},
                'l_shoulder': {'parent':'chest',        'child':'l_upper_arm',  'parent_joint': 'spine'},
                'l_elbow':    {'parent':'l_upper_arm',  'child':'l_lower_arm',  'parent_joint': 'l_shoulder'},
                'l_wrist':    {'parent':'l_lower_arm',  'child':'l_hand',       'parent_joint': 'l_elbow'},
                'r_shoulder': {'parent':'chest',        'child':'r_upper_arm',  'parent_joint': 'spine'},
                'r_elbow':    {'parent':'r_upper_arm',  'child':'r_lower_arm',  'parent_joint': 'r_shoulder'},
                'r_wrist':    {'parent':'r_lower_arm',  'child':'r_hand',       'parent_joint': 'r_elbow'}
              }

JOINT_NAMES = ['spine', 'neck', 'l_shoulder', 'l_elbow', 'l_wrist', 'r_shoulder', 'r_elbow', 'r_wrist']

SENSOR_NAMES = ['chest','head','l_upper_arm','l_lower_arm','l_hand','r_upper_arm','r_lower_arm','r_hand']

#                     Position wrt parent   Rotation wrt to Parent          Rotation correction
BONE_RATIOS = {
    'chest':        [ Vector(0.042, 0, 0.115),  Rotation.RotZ(3.0*math.pi/2.0), Rotation.Identity()],
    'head':         [ Vector(0, 0, 0.103),  Rotation.RotY(math.pi),         Rotation.Identity()],
    'l_upper_arm':  [ Vector(0, 0, 0.159),  Rotation.Identity(),            Rotation.Identity()],
    'l_lower_arm':  [ Vector(0, 0, 0.143),  Rotation.Identity(),            Rotation.Identity()],
    'l_hand':       [ Vector(0, 0, 0.107),  Rotation.Identity(),            Rotation.Identity()],
    'r_upper_arm':  [ Vector(0.099, 0, 0),  Rotation.RotY(-math.pi/2.0),    Rotation.Identity()],
    'r_lower_arm':  [ Vector(0, 0, -0.159), Rotation.Identity(),            Rotation.Identity()],
    'r_hand':       [ Vector(0, 0, -0.143), Rotation.Identity(),            Rotation.RotX(math.pi) * Rotation.RotZ(-math.pi/2.0)]
}


# Functions
def array_to_kdl_quaternion(q):
  return Rotation.Quaternion(*q)


def array_to_quaternion(q):
  return Quaternion(*q)


def calculate_joint_angles(parent_quat, child_quat):
  UNIT_X = Vector(1,0,0)
  UNIT_Y = Vector(0,1,0)
  UNIT_Z = Vector(0,0,1)

  quat0 = Rotation.Quaternion(*parent_quat)
  quat1 = Rotation.Quaternion(*child_quat)
  forward0 = quat0 * UNIT_Z
  up0 = quat0 * UNIT_Y
  right0 = quat0 * UNIT_X

  forward1 = quat1 * UNIT_Z
  up1 = quat1 * UNIT_Y
  right1 = quat1 * UNIT_X

  ## Calculate the angle between the right vectors and the axis vector perpendicular to them
  angle = math.acos(max(min(dot(right1,right0), 1.0), -1.0))
  axis = right1 * right0
  axis.Normalize()

  ## Transform the forward vector of the child bone so that it is on the same horizontal plane as the forward vector of the parent bone  
  transformed_forward1 = Rotation.Rot(axis, angle) * forward1
  transformed_forward1.Normalize()

  ## Calculate the angle between the transformed forward vector and the forward vector of the parent bone
  ## This is the angle of the X-axis
  x_angle = math.acos(max(min(dot(transformed_forward1,forward0), 1.0), -1.0))

  ## Calculate a vector perpendicular to the transformed forward vector and the forward vector of the parent bone
  axis = transformed_forward1 * forward0
  axis.Normalize()

  ## Transform the forward vector of the child bone so that it is on the same vertical plane as the forward vector of the parent bone
  ## and to transform the up vector of the child bone to be used in a later calculation
  axis_ang = Rotation.Rot(axis, x_angle)
  transformed_forward1 = axis_ang * forward1
  transformed_forward1.Normalize()
  transformed_up1 = axis_ang * up1
  transformed_up1.Normalize()

  ## Set the sign of y_angle using the axis calculated and the right vector of the parent bone
  x_angle = math.copysign(x_angle, dot(axis, right0))

  ## Calculate the angle between the transformed forward vector and the forward vector of the parent bone
  ## This is the angle of the Y-axis
  y_angle = math.acos(max(min(dot(transformed_forward1, forward0), 1.0), -1.0))

  ## Calculate a vector perpendicular to the transformed forward vector and the forward vector of the parent bone
  axis = transformed_forward1 * forward0
  axis.Normalize()

  ## Transform the transformed up vector so that it is on the same vertical plane as the up vector of the parent bone
  transformed_up1 = Rotation.Rot(axis, x_angle) * transformed_up1
  transformed_up1.Normalize()

  ## Set the sign of x_angle using the axis calculated and the up vector of the parent bone
  y_angle = math.copysign(y_angle, dot(axis, up0))

  ## Calculate the angle between the transformed up vector and the up vector of the parent bone
  ## This is the angle of the Z-axis
  z_angle = math.acos(max(min(dot(transformed_up1, up0), 1.0), -1.0))
  axis = transformed_up1 * up0
  axis.Normalize()

  ## Set the sign of z_angle using the axis calculated and the forward vector of the parent bone
  z_angle = math.copysign(z_angle, dot(axis, forward0))

  return [x_angle, y_angle, z_angle]


def euler_from_matrix(parent_quat, child_quat):
  q_relative = tr.quaternion_multiply(tr.quaternion_inverse(parent_quat), child_quat)
  R = tr.quaternion_matrix(q_relative)
  if not nearly_equal(abs(R[2][0]), 1.0):
    p1 = -math.asin(R[2][0])
    p2 = math.pi - p1
    r1 = math.atan2(R[2][1]/math.cos(p1), R[2][2]/math.cos(p1))
    r2 = math.atan2(R[2][1]/math.cos(p2), R[2][2]/math.cos(p2))
    y1 = math.atan2(R[1][0]/math.cos(p1), R[1][0]/math.cos(p1))
    y2 = math.atan2(R[1][0]/math.cos(p2), R[1][0]/math.cos(p2))
    r = r2
    p = p2
    y = y2
  else:
    y = 0
    if nearly_equal(R[2][0], -1.0):
      p = math.pi/2
      r = y + math.atan2(R[0][1], R[0][2])
    else:
      p = -math.pi/2
      r = -y + math.atan2(-R[0][1], -R[0][2])
  return [r,p,y]


def get_euler_angles(parent_quat, child_quat):
  q_relative = tr.quaternion_multiply(tr.quaternion_inverse(parent_quat), child_quat)
  qw = q_relative[3]
  qx = q_relative[0]
  qy = q_relative[1]
  qz = q_relative[2]
  test = qx*qy + qz*qw
  if (test > 0.499):
    roll = 2.0 * math.atan2(qx,qw)
    pitch = math.pi/2
    yaw = 0
  elif (test < -0.499):
    roll = -2.0 * math.atan2(qx, qw)
    pitch = -math.pi/2
    yaw = 0
  else:
    sqx = qx**2;
    sqy = qy**2;
    sqz = qz**2;
    roll = math.atan2(2*qy*qw-2*qx*qz , 1 - 2*sqy - 2*sqz)
    pitch = math.asin(2*test)
    yaw = math.atan2(2*qx*qw-2*qy*qz , 1 - 2*sqx - 2*sqz)
  return [roll, pitch, yaw]


def kdl_quaternion_from_msg(q):
  return Rotation.Quaternion(q.x, q.y, q.z, q.w)


def nearly_equal(a, b , sig_fig=5):
  return (a==b or int(a*10**sig_fig) == int(b*10**sig_fig))


def quaternion_to_array(q):
  return np.array([q.x, q.y, q.z, q.w])


def read_parameter(name, default):
  if not rospy.has_param(name):
    rospy.logwarn('Parameter [%s] not found, using default: %s' % (name, default))
  return rospy.get_param(name, default)

def vector_to_array(vect):
  return np.array([vect.x, vect.y, vect.z])
