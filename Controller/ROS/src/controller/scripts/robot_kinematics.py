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
Rotation matrixes:
Rt(x, L):
    [[1,         0,       0   ]
     [0,       cos(L), -sin(L)]
     [0,       sin(L),  cos(L)]]
Rt(y, B):
    [[cos(B),    0,     sin(B)]
     [0,         1,       0   ]
     [-sin(B),   0,     cos(B)]]
Rt(z, G):
    [[cos(G), -sin(G),    0   ]
     [sin(G),  cos(G),    0   ]
     [0,         0,       1   ]]
'''

import numpy.matlib
import numpy as np
from sympy import sin, cos, pi, sqrt, atan2, acos
import math

# supress printing enormous small numbers like 0.123e-16
np.set_printoptions(suppress=True)

# Rotation matrix
# rot_joint -> rotation joint -> 'x', 'y' or 'z'
# angle -> rotation angle in radians
# size -> dimention of square matrix, defualt minimum is 3
def rotation_matrix(rot_joint, angle, size = 3):
    if (angle < -2*pi) or (angle > 2*pi):
        raise Exception('Error, angle limits are from -2pi to 2pi')
    if size < 3:
        raise Exception('Error, rotation matrix siz should be 3 or greater')
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
    else:
        raise Exception('Unknown axis name, only x, y or z are supported')
    # if size is greater that rot_mtx shape make the rotation matrix part of identity_of_size beggining from the first element
    if size != rot_mtx.shape[0]:
        identity_of_size[0:size-1,0:size-1] = rot_mtx
        return identity_of_size
    return rot_mtx

# Translation -> move axis by vector 
# Transformation -> translation + rotation by angle
# vect = position vector
# rtm  = rotation matrix, 3x3 identity matrix if no angle given
def translation_matrix(vect, axis='', angle=0):
    if len(vect) != 3:
        raise Exception('Incorrect vector size, vector dimension should be (1, 3) -> [x y z]')
    else:
        rtm = np.identity(4) if not axis else rotation_matrix(axis, angle, 4)
        for x in range(3):
            rtm[x,3] = vect[x] # fit translated vector x into last column of rotatiom or identity matrix
        return rtm

# DH_i-1_i = Rt(Z, Oi) * Tr([0, 0, Ei]^T) * Tr([ai, 0, 0]^T) * Rt(X, Li)
def prev_to_curr_joint_transform_matrix(theta_i, epsilon_i, a_i, alpha_i):
    size_of_mtx = 4
    rot_mtx_z_theta = rotation_matrix('z', theta_i, size_of_mtx)
    tr_mtx_epsilon = translation_matrix([0, 0, epsilon_i])
    tr_mtx_a = translation_matrix([a_i, 0, 0])
    rot_mtx_z_alpha = rotation_matrix('x', alpha_i, size_of_mtx)
    dh_i = rot_mtx_z_theta.dot(tr_mtx_epsilon).dot(tr_mtx_a).dot(rot_mtx_z_alpha)
    return np.array(dh_i)

# Combine all computations into forward kinematics
def forward_kinematics(thetas, epsilons, ais, alphas):
    if not all(x == len(thetas) for x in (len(thetas), len(epsilons), len(ais), len(alphas))):
        raise Exception('All homogenous matrix arguments size should be equal to robot DOF')
    allmtx = []
    allmtx.append(prev_to_curr_joint_transform_matrix(thetas[0], epsilons[0], ais[0], alphas[0]))
    for elem in range(len(thetas)-1):
        nextMatrix = allmtx[elem].dot(prev_to_curr_joint_transform_matrix(thetas[elem+1], epsilons[elem+1], ais[elem+1], alphas[elem+1]))
        allmtx.append(nextMatrix)
    return allmtx[-1], allmtx

def pow(arg, p):
    return arg ** p

# Inverse kinematics https://www.actamechanica.sk/pdfs/ams/2016/03/07.pdf -> inverse kinematics
# equatios to 3rd joint, if theta4 will be added it wouldn't be possible to find it due to 3 equations and 4 variables,
# for x add '+ (l4 * cos(thetas[1] + thetas[2] + thetas[3])'
# for y add '+ (l4 * cos(thetas[1] + thetas[2] + thetas[3])'
# for z add '+ (l4 * sin(thetas[1] + thetas[2] + thetas[3])
# Unique implementation for every robot!
def roboarm_inverse_kinematics(joint_distances, eff_pos = [1, 1, 1]):
    # if not all(x == len(thetas) for x in (len(thetas), len(epsilons))):
    #     raise Exception('Input vectors should be length of 4!')

    # Trigonometric equations for forward kinematics                                                           #   Inverse kinematics cannot be calculated for all joints analyticaly 
    #                                                                                                          #   due too many varaibles (4) and too little equations (3)
    # x0 = float(cos(thetas[0]) * ((epsilons[1] * cos(thetas[1])) + (epsilons[2] * cos(thetas[1] + thetas[2])) # + (epsilons[3] * cos(thetas[1] + thetas[2] + thetas[3]))))
    # y0 = float(sin(thetas[0]) * ((epsilons[1] * cos(thetas[1])) + (epsilons[2] * cos(thetas[1] + thetas[2])) # + (epsilons[3] * cos(thetas[1] + thetas[2] + thetas[3]))))
    # z0 = float(epsilons[0] + (epsilons[1] * sin(thetas[1])) + (epsilons[2] * sin(thetas[1] + thetas[2]))     # + (epsilons[3] * sin(thetas[1] + thetas[2] + thetas[3])))
    
    # Inverse kinematics till 3rd joint   

    xc = eff_pos[0]
    yc = eff_pos[1]
    z  = eff_pos[2]
    e1 = 2
    e2 = 2
    e3 = 2

    cos_j3 = (pow(xc,2) + pow(yc,2) + pow((z - e1),2) - pow(e2,2) - pow(e3,2)) / (2 * e2 * e3)
    sin_j3 = sqrt(1 - pow(cos_j3,2))

    theta_3 = atan2(sin_j3, cos_j3)

    theta_1 = atan2(yc, xc)

    cos_j2 = ((xc/cos(theta_1)) * (-e2-(e3*cos_j3)) + ((z - e1)*e3*sin_j3)) / (-pow(e2,2) - pow(e3,2) - (2 * e2 * e3 * cos_j3))
    sin_j2 = ((sqrt(1-pow(cos_j2,2))))

    theta_2 = atan2(sin_j2, cos_j2)

    theta_4 = 0 # now just assume always 0 for last joint -> todo

    return [float(theta_1), float(theta_2), float(theta_3), float(theta_4)]

tout, _ = forward_kinematics([2, 3, -3], [2, 0, 0,], [0, 2, 2], [pi/2, 0, 0])
position = [tout[0, 3], tout[1, 3], tout[2, 3]]
print(position)
inv_k = roboarm_inverse_kinematics([2, 2, 2], position)
print(inv_k)