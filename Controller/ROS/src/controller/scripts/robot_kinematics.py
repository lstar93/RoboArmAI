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
from math import e, sin, cos, pi, sqrt, atan2, acos

from mpl_toolkits.mplot3d import Axes3D # <--- This is important for 3d plotting 
import matplotlib.pyplot as plt

# supress printing enormous small numbers like 0.123e-16
np.set_printoptions(suppress=True)

# Keep some prints, but sho them only if necessary
VERBOSE = False
def PRINT_MSG(msg, v=VERBOSE):
    if(v):
        print(msg)

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
    # if size is greater that rot_mtx shape make the rotation matrix part of identity_of_size beginning from the first element
    if size != rot_mtx.shape[0]:
        identity_of_size[0:size-1,0:size-1] = rot_mtx
        return identity_of_size
    return rot_mtx

# Translation -> move axis by vector 
# Transformation -> translation + rotation by angle
# vect = position vector
def translation_matrix(vect, axis='', angle=0):
    MATRIX_SIZE = 4
    if len(vect) != 3:
        raise Exception('Incorrect vector size, vector dimension should be (1, 3) -> [x y z]')
    else:
        # rtm -> rotation matrix, 4x4 identity matrix if no angle given
        rtm = np.identity(MATRIX_SIZE) if not axis else rotation_matrix(axis, angle, MATRIX_SIZE)
        for x in range(len(vect)):
            rtm[x,3] = vect[x] # repalce first 3 elems of matrix last column with translated vector x
        return rtm

# DH_i-1_i = Rt(Z, Oi) * Tr([0, 0, Ei]^T) * Tr([ai, 0, 0]^T) * Rt(X, Li)
def prev_to_curr_joint_transform_matrix(theta_i, epsilon_i, a_i, alpha_i):
    MATRIX_SIZE = 4
    rot_mtx_z_theta = rotation_matrix('z', theta_i, MATRIX_SIZE)
    tr_mtx_epsilon = translation_matrix([0, 0, epsilon_i])
    tr_mtx_a = translation_matrix([a_i, 0, 0])
    rot_mtx_z_alpha = rotation_matrix('x', alpha_i, MATRIX_SIZE)
    dh_i = rot_mtx_z_theta.dot(tr_mtx_epsilon).dot(tr_mtx_a).dot(rot_mtx_z_alpha)
    return np.array(dh_i)

# Combine all computations into forward kinematics
def forward_kinematics(thetas, epsilons, ais, alphas):
    if not all(x == len(thetas) for x in (len(thetas), len(epsilons), len(ais), len(alphas))):
        raise Exception('All homogenous matrix arguments size should be equal each other and robot DOF')
    allmtx = []
    allmtx.append(prev_to_curr_joint_transform_matrix(thetas[0], epsilons[0], ais[0], alphas[0])) # initial matrix
    for elem in range(len(thetas) - 1):
        nextMatrix = allmtx[elem].dot(prev_to_curr_joint_transform_matrix(thetas[elem+1], epsilons[elem+1], ais[elem+1], alphas[elem+1])) # multiply every transformation matrix
        allmtx.append(nextMatrix)
    return allmtx[-1], allmtx

def pow(arg, p):
    return float(arg ** p)

class Point:
    x = 0.0
    y = 0.0
    z = 0.0

    def __init__(self, xyz):
        self.x = xyz[0]
        self.y = xyz[1]
        self.z = xyz[2]

    def distance_to_point(self, p):
        return sqrt(pow((self.x - p.x), 2) + pow((self.y - p.y), 2) + pow((self.z - p.z), 2))

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

# matplotlib cannot resize all axes to the same scale so very small numbers make plots impossible to analyze 
# thus all very small numbers (e-10 <> -e-10) will be rounded to 0 for plotting purposes only
def round(num):
    if num > -1*e**-10 and num < 1*e**-10:
        return 0
    return num

def plot_roboarm(joints, points = []):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    colors = [list(x) for x in numpy.random.rand(len(joints) + len(points),3)] 
    for j,c in zip(joints,colors[0:len(joints)]):
        ax.scatter([round(x.x) for x in j], [round(x.y) for x in j], [round(x.z) for x in j], color=c)
        ax.plot3D([round(x.x) for x in j], [round(x.y) for x in j], [round(x.z) for x in j], color=c)

    for p,c in zip(points,colors[len(joints):]):
        ax.scatter(round(p.x), round(p.y), round(p.z), color=c)

    plt.show()

# FABRIK stands from forward and backward reaching inverse kinematics -> https://www.youtube.com/watch?v=UNoX65PRehA&feature=emb_title
# TODO: add angles limits
class Fabrik:

    init_joints_positions = []
    joint_distances = []
    err_margin = 0.0
    max_iter_num = 0

    def __init__(self, init_joints_positions, joint_distances, err_margin = 0.001, max_iter_num = 100):
        self.joint_distances = joint_distances
        self.init_joints_positions = init_joints_positions
        self.err_margin = err_margin
        self.max_iter_num = max_iter_num
        
    # Compute backward iteration
    def backward(self, points, goal_point):
        # Compute backward joint positions -> from goal position to point close to the start joint
        # Backward iteration omit last joint and begins computations from goal_point
        points_to_ret = [goal_point] # goal point should be the last point in array
        positions = list(reversed(points[:-1]))
        distances = list(reversed(self.joint_distances[:-1]))
        for p, d in zip(positions, distances):
            points_to_ret.append(goal_point.get_point_between(p, d))
            goal_point = points_to_ret[-1] # next element is last computed element
        return list(reversed(points_to_ret))

    # Compute forward iteration
    def forward(self, points, start_point):
        # Compute forward joint positions -> from start position to point close to the goal position
        # Forward iteration omit first joint and begins computations from start_point
        points_to_ret = [start_point] # start point should be the first point in array
        positions = points[1:]
        distances = self.joint_distances[1:]
        for p, d in zip(positions, distances):
            points_to_ret.append(start_point.get_point_between(p, d))
            start_point = points_to_ret[-1] # next element is last computed element
        return points_to_ret

    def compute_goal_joints_positions(self, goal_eff_pos, verbose=False):
        if not all(x == len(self.init_joints_positions) for x in (len(self.init_joints_positions), len(self.joint_distances))):
            raise Exception('Input vectors should have equal lengths!')

        current_join_positions = self.init_joints_positions
        goal_joints_positions = []
        start_point = self.init_joints_positions[0]
        goal_point = Point([x for x in goal_eff_pos])
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
            iter_cnt += 1

        if verbose and not len(goal_joints_positions) == 0:
            base_point = Point([0, 0, 0])
            base = [base_point, goal_joints_positions[0]]
            plot_roboarm([base, self.init_joints_positions, goal_joints_positions], [base_point, goal_point, start_point])

        return goal_joints_positions

# Robo Arm inverse kinematics class
class InverseKinematics:
    # Compute angles from cosine theorem
    # IMPORTANT: function works only for RoboArm manipulator and FABRIK method!
    def FABRIK_compute_roboarm_joints_angles(self, gp, verbose=False):
        A = Point([0, 0, 0])
        B = Point([gp[0].x, gp[0].y, gp[0].z])
        C = Point([gp[1].x, gp[1].y, gp[1].z])
        D = Point([gp[2].x, gp[2].y, gp[2].z])
        E = Point([gp[3].x, gp[3].y, gp[3].z])

        base = [A, B]

        AB = A.distance_to_point(B)
        BC = B.distance_to_point(C)
        CD = C.distance_to_point(D)
        DE = D.distance_to_point(E)

        # first triangle
        ftr = [A, C]
        AC = A.distance_to_point(C)
        if C.x >= 0:
            theta_2 = (pi/2 - acos((pow(AB,2) + pow(BC,2) - pow(AC,2)) / (2 * AB * BC))) * -1
        else:
            theta_2 = (pi + pi/2 - acos((pow(AB,2) + pow(BC,2) - pow(AC,2)) / (2 * AB * BC)))

        # second triangle
        sectr = [B, D]
        BD = B.distance_to_point(D)
        theta_3 = (pi - acos((pow(BC,2) + pow(CD,2) - pow(BD,2)) / (2 * BC * CD))) * -1
        if D.x < 0:
            theta_3 = theta_3 * -1

        # third triangle
        thrdtr = [C, E]
        CE = C.distance_to_point(E)
        theta_4 = (pi - acos((pow(CD,2) + pow(DE,2) - pow(CE,2)) / (2 * CD * DE))) * -1
        if E.x < 0:
            theta_4 = theta_4 * -1

        theta_1 = float(atan2(gp[3].y, gp[3].x))

        return [theta_1, theta_2, theta_3, theta_4], [base, ftr, sectr, thrdtr]

    def ANN_compute_roboarm_joints_angles(self, gp, verbose=False):
        pass

    # use one of methods to compute inverse kinematics
    def compute_roboarm_ik(self, method, dest_point, dh_matrix, joints_lengths, joints_limits, reach_limit, first_rev_joint_point, max_err = 0.001, max_iterations_num = 100, verbose = False):
        # Some basic limits check
        if any(dp < limitv[1][0] or dp > limitv[1][1] for dp, limitv in zip(dest_point, joints_limits.items())):
            raise Exception("Point is out of RoboArm reach area! Limits: {}, ".format(joints_limits))
        
        # Roboarm reach distance check
        if first_rev_joint_point.distance_to_point(Point(dest_point)) > reach_limit:
            raise Exception("Point is out of RoboArm reach area! Reach limit is {}, but the distance to point is {}".format(reach_limit, first_rev_joint_point.distance_to_point(Point(dest_point))))

        if method.lower() == "fabrik":
            # FABRIK 
            theta_1 = float(atan2(dest_point[1], dest_point[0])) # compute theta_1 to omit horizontal move in FABRIK
            dh_matrix[0][0] = theta_1 # replace initial theta_1

            # Compute initial xyz possition of every robot joint
            _, fk_all = forward_kinematics(*dh_matrix)
            init_joints_positions = [Point([x[0][3], x[1][3], x[2][3]]) for x in fk_all]
            PRINT_MSG('Initial joints positions:    ' + str(init_joints_positions))

            # Compute joint positions using FABRIK
            fab = Fabrik(init_joints_positions, joints_lengths, max_err, max_iterations_num)
            goal_joints_positions = fab.compute_goal_joints_positions(dest_point, VERBOSE)
            PRINT_MSG('Goal joints positions:    ' + str(goal_joints_positions))

            # Compute roboarm angles from FABRIK computed positions
            ik_angles, joints_triangles = self.FABRIK_compute_roboarm_joints_angles(goal_joints_positions, verbose)
            
            # print robot arm
            if verbose:
                plot_roboarm([*joints_triangles, init_joints_positions, goal_joints_positions], [init_joints_positions[0], goal_joints_positions[-1]])
        
            return ik_angles
        
        raise Exception('Unknown method!')

'''
# test
if __name__ == '__main__':
    # Compute positions of all joints in robot init (base) position
    dh_matrix = [[0, pi/2, 0, 0], [2, 0, 0, 0], [0, 2, 2, 2], [pi/2, 0, 0, 0]]
    dest_point = [1, 5, 3]
    joints_lengths = [2, 2, 2, 2]
    robo_arm_joint_limits = {'x_limits': [0,6], 'y_limits': [-6,6], 'z_limits': [0,6]} # assumed limits
    robo_arm_reach_limit = 6 # lenght of 3 joint is the limit
    first_rev_joint_point = Point([0,0,2]) # first revolute joint, from this point reach limit will be computed
    fkine = InverseKinematics()
    ik_angles = []
    try:
        ik_angles = fkine.compute_roboarm_ik('FABRIK', dest_point, dh_matrix, joints_lengths, robo_arm_joint_limits, robo_arm_reach_limit, first_rev_joint_point, 0.001, 100, VERBOSE)
        print('IK angles: ' + str(ik_angles))
        dh_matrix_out = [ik_angles, [2, 0, 0, 0], [0, 2, 2, 2], [pi/2, 0, 0, 0]]
        fk, _ = forward_kinematics(dh_matrix_out[0], dh_matrix_out[1], dh_matrix_out[2], dh_matrix_out[3])
        print(fk)
    except Exception as e:
        print(e)
'''