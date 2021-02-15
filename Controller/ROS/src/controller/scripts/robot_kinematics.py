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
from math import sin, cos, pi, sqrt, atan2, acos, e

import matplotlib.pyplot as plt

# supress printing enormous small numbers like 0.123e-16
np.set_printoptions(suppress=True)

# Keep some prints, but sho them only if necessary
def PRINT_MSG(msg, VERBOSE=True):
    if(VERBOSE):
        print(msg)

# strange things happen to math.pi trigonometric funcs
def round(num):
    if (num < (e ** -16)):
        return 0
    else: return num

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
                                   [0,round(cos(angle)),-round(sin(angle))],
                                   [0,round(sin(angle)),round(cos(angle))]])
    elif rot_joint == 'y':
        rot_mtx = np.matlib.array([[round(cos(angle)),0,round(sin(angle))],
                                  [0,1,0],
                                  [-round(sin(angle)),0,round(cos(angle))]])
    elif rot_joint == 'z':
        rot_mtx = np.matlib.array([[round(cos(angle)),-round(sin(angle)),0],
                                   [round(sin(angle)),round(cos(angle)),0],
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
    for elem in range(len(thetas) - 1):
        nextMatrix = allmtx[elem].dot(prev_to_curr_joint_transform_matrix(thetas[elem+1], epsilons[elem+1], ais[elem+1], alphas[elem+1]))
        allmtx.append(nextMatrix)
    return allmtx[-1], allmtx

'''
# Inverse kinematics https://www.actamechanica.sk/pdfs/ams/2016/03/07.pdf -> inverse kinematics geaometric approach (4 thetas, 3 equations, not easly solvable without specific approach)
# Trigonometric approach for forward kinematics  
def forward_kinematics_trig(init_thetas, distances):
    x = float(cos(init_thetas[0]) * ((distances[1] * cos(init_thetas[1])) + (distances[2] * cos(init_thetas[1] + init_thetas[2])) + (distances[3] * cos(init_thetas[1] + init_thetas[2] + init_thetas[3]))))
    y = float(sin(init_thetas[0]) * ((distances[1] * cos(init_thetas[1])) + (distances[2] * cos(init_thetas[1] + init_thetas[2])) + (distances[3] * cos(init_thetas[1] + init_thetas[2] + init_thetas[3]))))
    z = float(distances[0]        +  (distances[1] * sin(init_thetas[1])) + (distances[2] * sin(init_thetas[1] + init_thetas[2])) + (distances[3] * sin(init_thetas[1] + init_thetas[2] + init_thetas[3])))
    return [x, y, z]
'''

def pow(arg, p):
    return arg ** p

class Point:
    x = 0
    y = 0
    z = 0

    def __init__(self, xyz):
        self.x = xyz[0]
        self.y = xyz[1]
        self.z = xyz[2]

    def distance_to_point(self, p):
        ret = sqrt(pow((self.x - p.x), 2) + pow((self.y - p.y), 2) + pow((self.z - p.z), 2))
        return ret

    def get_point_between(self, end_point, distance):
        x_01 = self.x + ((distance/self.distance_to_point(end_point))*(end_point.x - self.x))
        y_01 = self.y + ((distance/self.distance_to_point(end_point))*(end_point.y - self.y))
        z_01 = self.z + ((distance/self.distance_to_point(end_point))*(end_point.z - self.z))
        return Point([x_01, y_01, z_01])

    def __str__(self):
        return str([self.x, self.y, self.z])

    def __repr__(self):
        return str(self)

    def to_list(self):
        return [self.x, self.y, self.z]

# FABRIK stands from forward and backward reaching inverse kinematics -> https://www.youtube.com/watch?v=UNoX65PRehA&feature=emb_title -> most commonly used this times
class Fabrik:

    init_joints_positions = []
    joint_distances = []
    err_margin = 0.0
    max_iter_num = 0

    def __init__(self, init_joints_positions, joint_distances, err_margin = 0.0001, max_iter_num = 10):
        self.joint_distances = joint_distances
        self.init_joints_positions = init_joints_positions
        self.err_margin = err_margin
        self.max_iter_num = max_iter_num

    def plot(self, start_point, goal_point, joints_final_positions):
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.scatter(start_point.x, start_point.y, start_point.z, color='blue')
        ax.scatter([x.to_list()[0] for x in self.init_joints_positions], [x.to_list()[1] for x in self.init_joints_positions], [x.to_list()[2] for x in self.init_joints_positions], color='green')
        ax.plot3D([x.to_list()[0] for x in self.init_joints_positions], [x.to_list()[1] for x in self.init_joints_positions], [x.to_list()[2] for x in self.init_joints_positions], color='green')
        ax.scatter([x.to_list()[0] for x in joints_final_positions], [x.to_list()[1] for x in joints_final_positions], [x.to_list()[2] for x in joints_final_positions], color='pink')
        ax.plot3D([x.to_list()[0] for x in joints_final_positions], [x.to_list()[1] for x in joints_final_positions], [x.to_list()[2] for x in joints_final_positions], color='pink')
        ax.scatter(goal_point.x, goal_point.y, goal_point.z, color='orange')
        plt.show()
        
    # Compute backward iteration
    def backward(self, points, goal_point):
        # Compute backward joint positions -> from goal position to point close to the start joint
        # Backward iteration omit last effector position and length and begins computations from goal_point
        first = goal_point
        points_to_ret = []
        points_to_ret.append(first) # goal point should be the last returned point
        pos = list(reversed(points[:-1]))
        distances = list(reversed(self.joint_distances[:-1]))
        for x in range(len(pos)):
            point_prim = first.get_point_between(pos[x], distances[x])
            first = point_prim
            points_to_ret.append(point_prim)
        return list(reversed(points_to_ret))

    # Compute forward iteration
    def forward(self, points, start_point):
        # Compute forward joint positions -> from start position to point close to the goal position
        # Forward iteration omit first effector position and length and begins computations from start_point
        first = start_point
        points_to_ret = []
        points_to_ret.append(first) # start point should be the first returned point
        pos = points[1:]
        distances = self.joint_distances[1:]
        for x in range(len(pos)):
            point_prim = first.get_point_between(pos[x], distances[x])
            first = point_prim
            points_to_ret.append(point_prim)
        return points_to_ret

    def compute_ik(self, goal_eff_pos, verbose=False):
        if not all(x == len(self.init_joints_positions) for x in (len(self.init_joints_positions), len(self.joint_distances))):
            raise Exception('Input vectors should have equal lengths!')

        current_join_positions = self.init_joints_positions
        goal_joints_positions = []
        start_point = self.init_joints_positions[0]
        goal_point = Point([goal_eff_pos[0], goal_eff_pos[1], goal_eff_pos[2]])
        start_error = 1
        goal_error = 1
        iter_cnt = 0

        while (((start_error > self.err_margin) or (goal_error > self.err_margin)) and (self.max_iter_num > iter_cnt)):
            retb = self.backward(current_join_positions, goal_point)
            start_error = retb[0].distance_to_point(start_point)
            retf = self.forward(retb, start_point)
            goal_error = retf[-1].distance_to_point(goal_point)
            current_join_positions = retf
            goal_joints_positions = current_join_positions
            PRINT_MSG('Iteration {} -> start position error = {}, goal position error = {}'.format(iter_cnt, start_error, goal_error), verbose)
            iter_cnt = iter_cnt + 1

        if len(goal_joints_positions) == 0:
           return []

        if(verbose):
            self.plot(start_point, goal_point, goal_joints_positions)

        # TODO: compute angles from cosine theorem and return them instead of positions

        return goal_joints_positions


'''
# test
if __name__ == '__main__':
    # Compute positions of all joints in robot init (base) position
    dest_point = [0, -4.137, 0.8346]
    theta_1 = float(atan2(-dest_point[1], -dest_point[0])) # compute theta_1
    dh_matrix = [[theta_1, pi/2, pi/3, pi/4], [2, 0, 0, 0], [0, 2, 2, 2], [pi/2, 0, 0, 0]]
    def get_robot_init_joints_position_fk(dh_matrix):
        _, fk_all = forward_kinematics(dh_matrix[0], dh_matrix[1], dh_matrix[2], dh_matrix[3])
        joints_init_positions = []
        for jfk in fk_all:
            joints_init_positions.append(Point([jfk[0][3], jfk[1][3], jfk[2][3]]))
        return joints_init_positions

    init_joints_positions = get_robot_init_joints_position_fk(dh_matrix)

    PRINT_MSG('Initial joints positions: ' + str(init_joints_positions))

    fab = Fabrik(init_joints_positions, [2, 2, 2, 2])
    out = fab.compute_ik(dest_point, True)

    PRINT_MSG('Goal joints positions:    ' + str(out))
'''
