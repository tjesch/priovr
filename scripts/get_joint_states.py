#!/usr/bin/env python
import rospy, serial, sys
from sensor_msgs.msg import Imu, JointState
from geometry_msgs.msg import Quaternion

# Quaternions tools
import numpy as np
import tf.transformations as tr
from tf_conversions import posemath

# Threespace Python API
import threespace as ts_api

VALID_SENSOR_NAMES = ('chest','head','l_upper_arm','l_lower_arm','l_hand','r_upper_arm','r_lower_arm','r_hand')
MALE_BONE_RATIOS = {'hips':0.086,
                    'chest':0.172,
                    'neck':0.103,
                    'head':4,
                    'l_shoulder':0.099,
                    'l_upper_arm':0.159,
                    'l_lower_Arm':0.143,
                    'L_Hand':0.107, 
                    'R_Shoulder':0.079,
                    'R_Upper_Arm':0.159,
                    'R_Lower_Arm':0.143, 
                    'R_Hand':0.107}

class GetJointStates(object):
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
    # Validate sensors names
    for key in self.sensors.keys():
      if key not in VALID_SENSOR_NAMES:
        rospy.logwarn('Invalid sensor name [/sensors/%s]' % key)
        del self.sensors[key]
    # Check consistency of the tare_quaternions dictionary
    for key in self.tare_quaternions.keys():
      if key not in self.sensors.keys():
        rospy.logwarn('Invalid tare quaternion [%s]' % key)
    # Check consistency of the mapping dictionary
    for key in self.mapping.keys():
      if key not in self.sensors.keys():
        rospy.logwarn('Invalid sensor name [/mapping/%s]' % key)
    
    # Connect to the PVRSystem
    self.pvr_system = ts_api.PVRSystem(com_port=self.com_port, baudrate=self.baudrate)
    # Tare the sensors defined in tare_quaternions
    for name, quat in self.tare_quaternions.items():
      id_number = self.sensors[name]
      self.pvr_system.tareWithCurrentOrientation(id_number, quat)
      rospy.loginfo('[%s] tared with: %s' % (name, quat))
      
    # Setup the KDL frames for the upper body
    self.frames['hips'] = PyKDL.Frame(PyKDL.Rotation.RPY(0,1,0), PyKDL.Vector(3,2,4))
    self.frames['chest'] = PyKDL.Frame(PyKDL.Rotation.RPY(0,1,0), PyKDL.Vector(3,2,4))
    self.frames['neck'] = PyKDL.Frame(PyKDL.Rotation.RPY(0,1,0), PyKDL.Vector(3,2,4))
    self.frames['head'] = PyKDL.Frame(PyKDL.Rotation.RPY(0,1,0), PyKDL.Vector(3,2,4))
    self.frames['chest'] = PyKDL.Frame(PyKDL.Rotation.RPY(0,1,0), PyKDL.Vector(3,2,4))
    self.frames['chest'] = PyKDL.Frame(PyKDL.Rotation.RPY(0,1,0), PyKDL.Vector(3,2,4))
    self.frames['chest'] = PyKDL.Frame(PyKDL.Rotation.RPY(0,1,0), PyKDL.Vector(3,2,4))
    self.frames['chest'] = PyKDL.Frame(PyKDL.Rotation.RPY(0,1,0), PyKDL.Vector(3,2,4))
    
    # Set-up publishers/subscribers
    self.state_pub = rospy.Publisher('joint_states', JointState)
    self.imu_pub = rospy.Publisher('chest', Imu)

  def cb_state_publisher(self, event):
    pass
  
  def run(self):
    while not rospy.is_shutdown():
      state_msg = JointState()
      for sensor_name, id_number in self.sensors.items():
        quat = self.pvr_system.getTaredOrientationAsQuaternion(id_number)
        if quat:
          #~ imu_msg.header = state_msg.header
          #~ imu_msg.orientation = Quaternion(*quat)
          #~ self.imu_pub.publish(imu_msg)
          rpy = tr.euler_from_quaternion(quat, axes='sxyz')
          for i,joint in enumerate(self.mapping[sensor_name]):
            if joint != '':
              state_msg.position.append(rpy[i])
              state_msg.name.append(joint)
      state_msg.header.stamp = rospy.Time.now()
      state_msg.header.frame_id = 'world'
      self.state_pub.publish(state_msg)


def calculate_joint_angles(self, orient0, orient1):  
  forward0 = orient0 * Vector3(UNIT_Z)
  up0 = orient0 * Vector3(UNIT_Y)
  right0 = orient0 * Vector3(UNIT_X)
  
  forward1 = orient1 * Vector3(UNIT_Z)
  up1 = orient1 * Vector3(UNIT_Y)
  right1 = orient1 * Vector3(UNIT_X)
  
  ## Calculate the angle between the right vectors and the axis vector perpendicular to them
  angle = math.acos(max(min(right1.dot(right0), 1.0), -1.0))
  axis = right1.cross(right0)
  axis.normalize()
  
  ## Transform the forward vector of the child bone so that it is on the same horizontal plane as the forward vector of the parent bone
  transformed_forward1 = AxisAngle(axis.asArray() + [angle]) * forward1
  transformed_forward1.normalize()
  
  ## Calculate the angle between the transformed forward vector and the forward vector of the parent bone
  ## This is the angle of the X-axis
  x_angle = math.acos(max(min(transformed_forward1.dot(forward0), 1.0), -1.0))
  
  ## Calculate a vector perpendicular to the transformed forward vector and the forward vector of the parent bone
  axis = transformed_forward1.cross(forward0)
  axis.normalize()
  
  ## Transform the forward vector of the child bone so that it is on the same vertical plane as the forward vector of the parent bone
  ## and to transform the up vector of the child bone to be used in a later calculation
  axis_ang = AxisAngle(axis.asArray() + [x_angle])
  transformed_forward1 = axis_ang * forward1
  transformed_forward1.normalize()
  transformed_up1 = axis_ang * up1
  transformed_up1.normalize()
  
  ## Set the sign of y_angle using the axis calculated and the right vector of the parent bone
  x_angle = math.copysign(x_angle, axis.dot(right0))
  
  ## Calculate the angle between the transformed forward vector and the forward vector of the parent bone
  ## This is the angle of the Y-axis
  y_angle = math.acos(max(min(transformed_forward1.dot(forward0), 1.0), -1.0))
  
  ## Calculate a vector perpendicular to the transformed forward vector and the forward vector of the parent bone
  axis = transformed_forward1.cross(forward0)
  axis.normalize()
  
  ## Transform the transformed up vector so that it is on the same vertical plane as the up vector of the parent bone
  transformed_up1 = AxisAngle(axis.asArray() + [x_angle]) * transformed_up1
  transformed_up1.normalize()
  
  ## Set the sign of x_angle using the axis calculated and the up vector of the parent bone
  y_angle = math.copysign(y_angle, axis.dot(up0))
  
  ## Calculate the angle between the transformed up vector and the up vector of the parent bone
  ## This is the angle of the Z-axis
  z_angle = math.acos(max(min(transformed_up1.dot(up0), 1.0), -1.0))
  axis = transformed_up1.cross(up0)
  axis.normalize()
  
  ## Set the sign of z_angle using the axis calculated and the forward vector of the parent bone
  z_angle = math.copysign(z_angle, axis.dot(forward0))
  
  return [x_angle, y_angle, z_angle]

  def read_parameter(self, name, default):
    if not rospy.has_param(name):
      rospy.logwarn('Parameter [%s] not found, using default: %s' % (name, default))
    return rospy.get_param(name, default)
    

if __name__ == '__main__':
  rospy.init_node('get_joint_states')
  get_js = GetJointStates()
  get_js.run()
