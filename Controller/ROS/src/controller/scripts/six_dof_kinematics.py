#!/bin/python3

# DH notation

'''       

      2y |          | 3y
         |     l3   |
         0-->-------0--> 3x
        /  2x        \
   y1| / l2        l4 \ |4y
     |/                \|
  1z 0-->1x          4z 0-->4x 
     |                 ----
     | l1              |  |
    /_\
    \ /
     |
_____|_____

  i  |  ai  |  Li  |  Ei  |  Oi  |
----------------------------------
  1  |   0  | pi/2 |  l1  |  O1  |
----------------------------------
  2  |  l2  |  0   |   0  |  O2  |
----------------------------------
  3  |  l3  |  0   |   0  |  O3  |
----------------------------------
  4  |  l4  |  0   |   0  |  O4  |
----------------------------------

Rotation matrixes
Rt(x, L):
    [[1          0        0   ]
     [0        cos(L)  -sin(L)]
     [0        sin(L)   cos(L)]]

    [[1,0,0]
     [0,cos(L),-sin(L)]
     [0,sin(L),cos(L)]]

Rt(y, B):
    [[cos(B),0,sin(B)]
     [0,1,0]
     [-sin(B),0,cos(B)]]

Rt(z, G):
    [[cos(G),-sin(G),0]
     [sin(G),cos(G),0]
     [0,0,1]]

'''

import numpy.matlib
import numpy as np
import math
from sympy import sin, cos, pi

# supress printing enormous small numbers like 0.123e-16
np.set_printoptions(suppress=True)

def rotation_matrix(rot_joint, angle, size = 3):
    identity_of_size = np.identity(size)
    rot_mtx = np.identity(size-1)
    if rot_joint == 'x':
        rot_mtx = np.matlib.array([[1,0,0],
                                   [0,cos(angle),-sin(angle)],
                                   [0,sin(angle),cos(angle)]])
    elif rot_joint == 'y':
        rot_mtx = np.matlib.array([[cos(angle),0,sin(angle)],
                                  [0,1,0],
                                  [-sin(angle),0,cos(angle)]])
    elif rot_joint == 'z':
        rot_mtx = np.matlib.array([[cos(angle),-sin(angle),0],
                                   [sin(angle),cos(angle),0],
                                   [0,0,1]])
    # if size is greater that rot_mtx shape make the rotation matrix part of identity_of_size beggining from the first element
    if identity_of_size.shape[0] != rot_mtx.shape[0]:
        identity_of_size[0:size-1,0:size-1] = rot_mtx
        return identity_of_size
    return rot_mtx
'''
print('rotation_matrix (x): ')
print(rotation_matrix('x', pi))
print('rotation_matrix (y): ')
print(rotation_matrix('y', pi))
print('rotation_matrix (z): ')
print(rotation_matrix('z', pi))
print()
'''


'''
Rt - rotation matrix, identity matrix if no rotation
Tr - transform matrix

Tr(V) = [[Rt V], [Zeros 1]]
Tr([x y z]) = [[1 0 0 x], 
               [0 1 0 y], 
               [0 0 1 z], 
               [0 0 0 1]]

DH_i-1_i = Rt(Z, Oi) * Tr([0, 0, Ei]^T) * Tr([ai, 0, 0]^T) * Rt(X, Li)
'''
# vect = position vector
# rtm  = rotation matrix, 3x3 identity matrix if no angle transforamtion
def translation_matrix(vect, rtm = np.identity(3)):
    if len(vect) != 3:
        raise 'Incorrect number of vector elements, vector dimension should be (1, 3) -> [x y z]'
    else:
        return np.matlib.array([[rtm.item((0,0)), rtm.item((0,1)), rtm.item((0,2)), vect[0]], 
                                [rtm.item((1,0)), rtm.item((1,1)), rtm.item((1,2)), vect[1]], 
                                [rtm.item((2,0)), rtm.item((2,1)), rtm.item((2,2)), vect[2]], 
                                [     0,               0,               0,             1  ]])
'''
print('translation_matrix (position vector only): ')
print(translation_matrix([1, 2, 3]))
print('translation_matrix (position vector and x rotation): ')
print(translation_matrix([1, 2, 3], rotation_matrix('x', pi)))
print('translation_matrix (position vector and y rotation): ')
print(translation_matrix([4, 5, 6], rotation_matrix('y', pi)))
print('translation_matrix (position vector and z rotation): ')
print(translation_matrix([7, 8, 9], rotation_matrix('z', pi)))
print()
'''

def prev_to_curr_joint_transform_matrix(theta_i, epislon_i, a_i, alpha_i):
    size_of_mtx = 4
    rot_mtx_z_theta = rotation_matrix('z', theta_i, size_of_mtx)
    tr_mtx_epsilon = translation_matrix([0, 0, epislon_i])
    tr_mtx_a = translation_matrix([a_i, 0, 0])
    rot_mtx_z_alpha = rotation_matrix('x', alpha_i, size_of_mtx)
    dh_i = np.matmul(np.matmul(rot_mtx_z_theta, tr_mtx_epsilon), np.matmul(tr_mtx_a, rot_mtx_z_alpha))
    return dh_i
'''
print('prev_to_curr_joint_transform_matrix: ')
print(prev_to_curr_joint_transform_matrix(pi/2, 3, 5, pi))
print()

teta_1 = pi/2
el_1 = 3
el_2 = 5
tet_arr = np.matlib.array([[cos(teta_1),  sin(teta_1),    0,        el_2 * cos(teta_1)], 
                           [sin(teta_1), -cos(teta_1),    0,        el_2 * sin(teta_1)], 
                           [     0,            0,        -1,        el_1              ], 
                           [     0,            0,         0,                1         ]])
print(tet_arr)
print(tet_arr == prev_to_curr_joint_transform_matrix(pi/2, 3, 5, pi))
print()
'''