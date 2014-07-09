#!/usr/bin/env python
import rospy, serial, sys, math
from sensor_msgs.msg import Imu, JointState
from geometry_msgs.msg import Quaternion

# Quaternions tools
from PyKDL import Rotation, Vector, dot
import numpy as np
import tf.transformations as tr
from tf_conversions import posemath

UNIT_X = Vector(1,0,0)
UNIT_Y = Vector(0,1,0)
UNIT_Z = Vector(0,0,1)

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
    #~ self.frames['hips'] = Frame(Rotation.RPY(0,1,0), Vector(3,2,4))
    #~ self.frames['chest'] = Frame(Rotation.RPY(0,1,0), Vector(3,2,4))
    #~ self.frames['neck'] = Frame(Rotation.RPY(0,1,0), Vector(3,2,4))
    #~ self.frames['head'] = Frame(Rotation.RPY(0,1,0), Vector(3,2,4))
    #~ self.frames['chest'] = Frame(Rotation.RPY(0,1,0), Vector(3,2,4))
    #~ self.frames['chest'] = Frame(Rotation.RPY(0,1,0), Vector(3,2,4))
    #~ self.frames['chest'] = Frame(Rotation.RPY(0,1,0), Vector(3,2,4))
    #~ self.frames['chest'] = Frame(Rotation.RPY(0,1,0), Vector(3,2,4))
    
    # Set-up publishers/subscribers
    self.state_pub = rospy.Publisher('joint_states', JointState)
    self.imu_pub = rospy.Publisher('chest', Imu)

  def cb_state_publisher(self, event):
    pass
  
  def run(self):
    while not rospy.is_shutdown():
      state_msg = JointState()
      #~ for sensor_name, id_number in self.sensors.items():
      hips_quat = [0,0,0,1]
      chest_quat = self.pvr_system.getTaredOrientationAsQuaternion(self.sensors['chest'])
      head_quat = self.pvr_system.getTaredOrientationAsQuaternion(self.sensors['head'])
      if hips_quat and chest_quat and head_quat:
        #~ imu_msg.header = state_msg.header
        #~ imu_msg.orientation = Quaternion(*quat)
        #~ self.imu_pub.publish(imu_msg)
        #~ rpy = tr.euler_from_quaternion(quat, axes='sxyz')
        #~ for i,joint in enumerate(self.mapping[sensor_name]):
          #~ if joint != '':
            #~ state_msg.position.append(rpy[i])
            #~ state_msg.name.append(joint)
        state_msg.name = ['hips_roll','hips_pitch','hips_yaw','head_pitch','head_roll','head_yaw']
        #~ rpy_spine = self.calculate_joint_angles(hips_quat, chest_quat)
        #~ rpy_neck = self.calculate_joint_angles(chest_quat, head_quat)
        q_spine = tr.quaternion_multiply(hips_quat, tr.quaternion_inverse(chest_quat))
        q_neck = tr.quaternion_multiply(chest_quat, tr.quaternion_inverse(head_quat))
        rpy_spine = tr.euler_from_quaternion(q_spine, 'sxyz')
        rpy_neck = tr.euler_from_quaternion(q_neck, 'sxyz')
        #~ rpy_neck[0] *= -1.0
        state_msg.position = rpy_spine + rpy_neck
      state_msg.header.stamp = rospy.Time.now()
      state_msg.header.frame_id = 'world'
      self.state_pub.publish(state_msg)

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

  def read_parameter(self, name, default):
    if not rospy.has_param(name):
      rospy.logwarn('Parameter [%s] not found, using default: %s' % (name, default))
    return rospy.get_param(name, default)
    

if __name__ == '__main__':
  rospy.init_node('get_joint_states')
  get_js = GetJointStates()
  get_js.run()
