#! /usr/bin/env python
"""
Notes
-----
Calculations are carried out with numpy.float64 precision.

This Python implementation is not optimized for speed.

Angles are in radians unless specified otherwise.

Quaternions ix+jy+kz+w are represented as [x, y, z, w].
"""

# Quaternions tools
import numpy as np
import tf.transformations as tr
from math import pi

if __name__ == '__main__':
  tare_quaternions = dict()
  # chest
  tare_quaternions['chest'] = np.array([0.,0.,0.,1.])
  # r_upper
  r_upper_from_matrix = tr.quaternion_from_matrix(np.array([[ -1.,  0.,  0.,  0.], [ 0.,  -1.,  0.,  0.], [ 0.,  0.,  1.,  0.], [ 0.,  0.,  0.,  1.]]))
  r_upper_from_euler = tr.quaternion_from_euler(0, 0, pi)
  if np.allclose(r_upper_from_matrix, r_upper_from_euler):
    tare_quaternions['r_upper'] = r_upper_from_matrix
  # r_lower and r_hand
  r_lower_from_matrix = tr.quaternion_from_matrix(np.array([[ 0.,  0.,  1.,  0.], [ 0.,  1.,  0.,  0.], [ -1.,  0.,  0.,  0.], [ 0.,  0.,  0.,  1.]]))
  r_lower_from_euler = tr.quaternion_from_euler(0, pi/2, 0)
  if np.allclose(r_lower_from_matrix, r_lower_from_euler):
    tare_quaternions['r_lower'] = r_lower_from_matrix
    tare_quaternions['r_hand'] = r_lower_from_matrix
    
  for key, value in tare_quaternions.items():
    print('%s: %s' % (key, list(value)))
