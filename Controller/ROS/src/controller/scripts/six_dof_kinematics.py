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
     [0,math.cos(L),-math.sin(L)]
     [0,math.sin(L),math.cos(L)]]

Rt(y, B):
    [[math.cos(B),0,math.sin(B)]
     [0,1,0]
     [-math.sin(B),0,math.cos(B)]]

Rt(z, G):
    [[math.cos(G),-math.sin(G),0]
     [math.sin(G),math.cos(G),0]
     [0,0,1]]

'''

import numpy.matlib
import numpy as np
import math

def joint_rotation_mtx(rot_joint, angle):
    if rot_joint == 'x':
        return np.matlib.array([[1,0,0],
                                [0,math.cos(angle),-math.sin(angle)],
                                [0,math.sin(angle),math.cos(angle)]])
    elif rot_joint == 'y':
        return np.matlib.array([[math.cos(angle),0,math.sin(angle)],
                                [0,1,0],
                                [-math.sin(angle),0,math.cos(angle)]])
    elif rot_joint == 'z':
        return np.matlib.array([[math.cos(angle),-math.sin(angle),0],
                                [math.sin(angle),math.cos(angle),0],
                                [0,0,1]])

print(joint_rotation_mtx('x', math.pi))
print(joint_rotation_mtx('y', math.pi))
print(joint_rotation_mtx('z', math.pi))

# i-1 H 1 -> DH matrix
'''
R1 - unit matrix
Tr - transformed matrix

Tr(V) = [[R1 V], [Zeros 1]]
Tr([x y z]) = [[1 0 0 x], 
               [0 1 0 y], 
               [0 0 1 z], 
               [0 0 0 1]]

H_i-1_i = Rt(Z, Oi) Tr([0, 0, Ei]) Tr([ai, 0, 0]) Rt(X, Li)

'''

def transform_matrix(vect):
	if len(vect) != 3:
		raise 'Incorrect number of vector elements, vector dimension should be (1, 3) -> [x y z]'
	else:
		return np.matlib.array([ [1, 0, 0, vect[0]], [0, 1, 0, vect[1]], [0, 0, 1, vect[2]], [0, 0, 0, 1] ])

print(transform_matrix([1, 2, 3]))