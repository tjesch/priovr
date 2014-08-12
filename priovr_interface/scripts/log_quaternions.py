#!/usr/bin/env python
import rospy, os, datetime
import numpy as np
import scipy.io as sio
# Messages
from priovr_msgs.msg import QuaternionArray

class LogQuaternions(object):
  def __init__(self):
    # Read from parameter server
    self.topic = rospy.get_param('~topic', '/priovr/sensor_orientations')
    self.data_key = rospy.get_param('~data_key', 'range')
    # Initial values
    self.whole_data = dict()
    self.msg_count = 0
    # Shutdown hookup for saving the matfile
    rospy.on_shutdown(self.shutdown)
    # Set-up publishers/subscribers
    rospy.Subscriber(self.topic, QuaternionArray, self.quaternions_cb)
    rospy.loginfo('Waiting for [%s] topic' % self.topic)
    rospy.spin()
  
  def quaternions_cb(self, msg):
    for i,name in enumerate(msg.name):
      quat_matrix = self.quaternion2matrix(msg.quaternion[i])
      if self.msg_count == 0:
        self.whole_data[name] = quat_matrix
      else:
        self.whole_data[name] = np.append(self.whole_data[name], quat_matrix, axis=0)
    if (self.msg_count % 100  == 0):
      rospy.loginfo('Received %d messages from [%s]' % (self.msg_count + 1, self.topic))
    self.msg_count += 1
  
  def shutdown(self):
    # Add timestamp to the filename
    timestamp = datetime.datetime.now().strftime('%Y-%m-%d-%H-%M-%S')
    filename = os.path.expanduser('~/benchmark/priovr_%s-%s.mat' % (self.data_key, timestamp))
    sio.savemat(filename, self.whole_data, oned_as='column')
    rospy.loginfo('Saved %d msgs to: %s' % (self.msg_count, filename))
  
  def quaternion2matrix(self, quaternion):
    return np.matrix([quaternion.x, quaternion.y, quaternion.z, quaternion.w])

if __name__ == '__main__':
  node_name = 'log_quaternions'
  rospy.init_node(node_name)
  rospy.loginfo('Starting the %s node' % node_name)
  lq = LogQuaternions()
  rospy.loginfo('Shuting down the %s node' % node_name)
