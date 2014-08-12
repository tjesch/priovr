#!/usr/bin/env python
# General imports
import rospy, sys, math
from math import pi, cos, atan2, asin
# Messages
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Quaternion
from priovr_msgs.msg import QuaternionArray
# Quaternions tools
import numpy as np
import tf.transformations as tr
from PyKDL import Rotation, Vector, dot
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

UNIT_X = Vector(1,0,0)
UNIT_Y = Vector(0,1,0)
UNIT_Z = Vector(0,0,1)


# Helper functions
def detect_present_sensors():
  pvr_system = ts_api.PVRSystem(com_port='/dev/ttyACM0', baudrate=921600)
  found = []
  for id_number in range(20):
    if pvr_system.getAllRawComponentSensorData(id_number):
      found.append(id_number)
  return found
  
def nearly_equal(a,b,sig_fig=5):
    return ( a==b or 
             int(a*10**sig_fig) == int(b*10**sig_fig)
           )


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
    self.debug = rospy.get_param('~debug', False)
    
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
    for joint in self.mapping.keys():
      invalid_joint = joint not in JOINTS.keys()
      invalid_parent = JOINTS[joint]['parent'] not in self.sensors.keys() + ['hips']
      invalid_child = JOINTS[joint]['child'] not in self.sensors.keys()
      if invalid_joint or invalid_parent or invalid_child:
        rospy.logdebug('Invalid joint defined in [/mapping/%s]' % joint)
        del self.mapping[joint]
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
    self.state_pub = rospy.Publisher('/priovr/joint_states', JointState)
    if self.debug:
      self.debug_pub = rospy.Publisher('/priovr/quaternion_array', QuaternionArray)
    
    # Start streaming to receive the joysticks data
    self.pvr_system.startStreaming()
    # Shutdown hookup for stoping PVR streaming
    rospy.on_shutdown(self.shutdown)
    
    # Initialize dictionaries
    self.link_orientations = dict()
    self.joint_orientations = dict()
    
  def shutdown(self):
    if self.pvr_system:
      self.pvr_system.stopStreaming()
  
  def run(self):
    while not rospy.is_shutdown():
      fail_sensor_read = False
      self.link_orientations['hips'] = [0,0,0,1]
      quat_array_msg = QuaternionArray()
      for name, id_number in self.sensors.items():
        # Get the tared orientation
        try:
          q_tared = self.pvr_system.getTaredOrientationAsQuaternion(id_number)
        except:
          fail_sensor_read = True
          break
        if q_tared:
          self.link_orientations[name] = q_tared
          quat_array_msg.name.append(name)
          quat_array_msg.quaternion.append(Quaternion(*q_tared))
        else:
          rospy.logdebug('Failed getTaredOrientationAsQuaternion from [%s]' % (name))
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
        # Split it in 3 DoF (roll, pitch and yaw)
        # TODO: Fucking Euler angles!!!
        rpy = list(self.calculate_joint_angles(q_parent, q_child))
        self.joint_orientations[name] = tr.quaternion_multiply(tr.quaternion_inverse(q_parent), q_child)
        rpy_alt = list(tr.euler_from_quaternion(self.joint_orientations[name], 'rxyz'))
        #~ rpy[2] = rpy_alt[2]
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
      # Publish the QuaternionArray msg
      if self.debug:
        quat_array_msg.header = state_msg.header
        self.debug_pub.publish(quat_array_msg)

  def calculate_joint_angles(self, parent_quat, child_quat):
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
    
  def get_euler_angles(self, parent_quat, child_quat):
    q_relative = tr.quaternion_multiply(tr.quaternion_inverse(parent_quat), child_quat)
    qw = q_relative[3]
    qx = q_relative[0]
    qy = q_relative[1]
    qz = q_relative[2]
    test = qx*qy + qz*qw
    if (test > 0.499):
      roll = 2.0 * atan2(qx,qw)
      pitch = pi/2
      yaw = 0
    elif (test < -0.499):
      roll = -2.0 * atan2(qx, qw)
      pitch = -pi/2
      yaw = 0
    else:
      sqx = qx**2;
      sqy = qy**2;
      sqz = qz**2;
      roll = atan2(2*qy*qw-2*qx*qz , 1 - 2*sqy - 2*sqz)
      pitch = asin(2*test)
      yaw = atan2(2*qx*qw-2*qy*qz , 1 - 2*sqx - 2*sqz)
    return [roll, pitch, yaw]
  
  def euler_from_matrix(self, parent_quat, child_quat):
    q_relative = tr.quaternion_multiply(tr.quaternion_inverse(parent_quat), child_quat)
    R = tr.quaternion_matrix(q_relative)
    if not nearly_equal(abs(R[2][0]), 1.0):
      p1 = -asin(R[2][0])
      p2 = pi - p1
      r1 = atan2(R[2][1]/cos(p1), R[2][2]/cos(p1))
      r2 = atan2(R[2][1]/cos(p2), R[2][2]/cos(p2))
      y1 = atan2(R[1][0]/cos(p1), R[1][0]/cos(p1))
      y2 = atan2(R[1][0]/cos(p2), R[1][0]/cos(p2))
      r = r2
      p = p2
      y = y2
    else:
      y = 0
      if nearly_equal(R[2][0], -1.0):
        p = pi/2
        r = y + atan2(R[0][1], R[0][2])
      else:
        p = -pi/2
        r = -y + atan2(-R[0][1], -R[0][2])
    
    return [r,p,y]

  def read_parameter(self, name, default):
    if not rospy.has_param(name):
      rospy.logwarn('Parameter [%s] not found, using default: %s' % (name, default))
    return rospy.get_param(name, default)
    

if __name__ == '__main__':
  rospy.init_node('get_joint_states')
  get_js = GetJointStates()
  get_js.run()


