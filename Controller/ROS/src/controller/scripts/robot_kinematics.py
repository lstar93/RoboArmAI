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

import random as rand
from keras import callbacks
from numpy.core.fromnumeric import shape
from numpy.lib.utils import _set_function_name
import numpy.matlib
import numpy as np
from math import e, sin, cos, pi, sqrt, atan2, acos
import sklearn
from sklearn import model_selection
import keras
import tensorflow as tf
import keras.backend as K

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
        self.x, self.y, self.z = xyz

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
# thus all very small numbers will be rounded to 0 for plotting purposes only
def plot_roboarm(joints, points = []):
    PRECISION = 10
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    colors = [list(x) for x in numpy.random.rand(len(joints) + len(points),3)] 
    for j,c in zip(joints,colors[0:len(joints)]):
        ax.scatter([round(x.x, PRECISION) for x in j], [round(x.y, PRECISION) for x in j], [round(x.z, PRECISION) for x in j], color=c)
        ax.plot3D([round(x.x, PRECISION) for x in j], [round(x.y, PRECISION) for x in j], [round(x.z, PRECISION) for x in j], color=c)

    for p,c in zip(points,colors[len(joints):]):
        ax.scatter(round(p.x, PRECISION), round(p.y, PRECISION), round(p.z, PRECISION), color=c)

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

from scipy.stats import truncnorm

def plot_points_3d(points):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    colors = [list(x) for x in numpy.random.rand(len(points),3)] 

    for p,c in zip(points,colors):
        ax.scatter(round(p.x, 1), round(p.y, 5), round(p.z, 5), color=c)

    plt.show()

# neural network IK approach
class ANN:
    effector_workspace_limits = {}
    angles_limits = {}
    dh_matrix = []
    in_data_skaler = sklearn.preprocessing.MinMaxScaler(feature_range=(-1,1))
    out_data_skaler = sklearn.preprocessing.MinMaxScaler(feature_range=(-1,1))
    model = None

    def __init__(self, angles_limits, effector_workspace_limits, dh_matrix):
        self.angles_limits = angles_limits
        self.effector_workspace_limits = effector_workspace_limits
        self.dh_matrix = dh_matrix

    def get_truncated_normal_distribution(self, mean=0, sd=1, low=0, upp=10):
        return truncnorm((low - mean) / sd, (upp - mean) / sd, loc=mean, scale=sd)

    # prepare trainig data
    def generate_trainig_data(self, distribution = 'normal', no_of_samples = 100):
        samples_x = [] # output samples -> angles
        for _, limitv in self.angles_limits.items():
            if distribution == 'normal':
                val = list(self.get_truncated_normal_distribution(mean=0, sd=0.5, low=limitv[0], upp=limitv[1]).rvs(no_of_samples))
                samples_x.append(val)
            elif distribution == 'uniform':
                samples_x.append([rand.uniform(*limitv) for x in range(no_of_samples)])
        samples_x[0] = [0 for x in range(len(samples_x[0]))]

        # plt.hist(samples_x, bins = no_of_samples)
        # plt.show()

        # use FK to generate positions
        pos_x, pos_y, pos_z = [], [], []
        input_data_list = [] # raw list of lists for input data
        output_data_list = [] # raw list of lists for output data
        for x in range(no_of_samples):
            dest_dh_matrix = self.dh_matrix[:]
            dest_dh_matrix[0] = [samples_x[i][x] for i in range(len(samples_x))]
            number_of_joints = len(dest_dh_matrix[0])
            fk, _ = forward_kinematics(*dest_dh_matrix)
            in_reach = not any(dp < limitv[1][0] or dp > limitv[1][1] for dp, limitv in \
                           zip([fk[0,3], fk[1,3], fk[2,3]], self.effector_workspace_limits.items()))
            if in_reach:
                for i in range(number_of_joints):
                    if len(output_data_list) < number_of_joints:
                        output_data_list.append([])
                    output_data_list[i].append(dest_dh_matrix[0][i])
                pos_x.append(fk[0,3])
                pos_y.append(fk[1,3])
                pos_z.append(fk[2,3])
        input_data_list = [pos_x, pos_y, pos_z]

        # create input and output array then transpose it to create array of shape [SAMPLES x FEATURES] 
        output_data = np.transpose(np.asarray(output_data_list, dtype=np.float32)) # training input data -> thetas
        input_data = np.transpose(np.array([x for x in input_data_list])) # training output data -> coordinates

        # training_data = [*inpud_data, *output_data]
        # print data in xyz diagram
        # coords = list(map(list, zip(*output_data))) # map to list all first positions elements and cast map to list
        # plot_points_3d([Point([*x]) for x in coords]) # create Points from lists

        # split data into training (70%), test (15%) and evaluation (15%)
        input, input_test_eval, output, output_test_eval = model_selection.train_test_split(input_data, output_data, test_size=0.3, random_state=42)
        input_test, input_eval, output_test, output_eval = model_selection.train_test_split(input_test_eval, output_test_eval, test_size=0.5, random_state=42) # split test and eval to 15% of whole dataset each

        # fit data to [-1,1] using scaler
        input_scaled = self.in_data_skaler.fit_transform(input)
        output_scaled = self.out_data_skaler.fit_transform(output)
        input_test_scaled = self.in_data_skaler.fit_transform(input_test)
        output_test_scaled = self.out_data_skaler.fit_transform(output_test)
        input_eval_scaled = self.in_data_skaler.fit_transform(input_eval)
        output_eval_scaled = self.out_data_skaler.fit_transform(output_eval)

        return input_scaled, output_scaled, input_test_scaled, output_test_scaled, input_eval_scaled, output_eval_scaled

    def customloss(self, yTrue, yPred, no_of_samples):
        return (keras.backend.sum((yTrue - yPred)**2))/no_of_samples

    def train_model(self, no_of_samples=10000, epochs=1000):
        self.model = keras.Sequential()
        self.model.add(keras.layers.Dense(3, use_bias=True, activation='linear')) # x, y, z -> input layer
        self.model.add(keras.layers.Dense(100, use_bias=True, activation='sigmoid')) # hidden layer 100 neurons
        self.model.add(keras.layers.Dense(4, use_bias=True, activation='linear')) # theta1, theta2, theta3, theta4 -> output layer
        self.model.compile(optimizer=keras.optimizers.Adam(lr=1e-5), loss=[lambda yTrue, yPred: self.customloss(yTrue, yPred, no_of_samples)], metrics=['accuracy'])
        data_in, data_out, data_test_in, data_test_out, data_eval_in, data_eval_out = self.generate_trainig_data(no_of_samples=no_of_samples)
        history = self.model.fit(data_in, data_out, epochs=epochs, validation_data=(data_eval_in, data_eval_out), callbacks=[keras.callbacks.TensorBoard()]) # callbacks=[keras.callbacks.TensorBoard()],
        loss, mae = self.model.evaluate(data_test_in, data_test_out, verbose=0)
        return history, mae, loss, self.model

    def predict_ik(self, position):
        arraynp = np.array(position)
        position_scaled = self.in_data_skaler.fit_transform(arraynp)
        predictions = self.model.predict(position_scaled)
        self.out_data_skaler.inverse_transform(predictions)
        return predictions

'''
 
'''

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
    def compute_roboarm_ik(self, method, dest_point, dh_matrix, joints_lengths, workspace_limits, first_rev_joint_point, max_err = 0.001, max_iterations_num = 100, verbose = False):
        # Some basic limits check
        if any(dp < limitv[1][0] or dp > limitv[1][1] for dp, limitv in zip(dest_point, workspace_limits.items())):
            raise Exception("Point is out of RoboArm reach area! Limits: {}, ".format(workspace_limits))

        effector_reach_limit = workspace_limits['x_limits'][1]

        # Roboarm reach distance check
        if first_rev_joint_point.distance_to_point(Point(dest_point)) > effector_reach_limit:
            raise Exception("Point is out of RoboArm reach area! Reach limit is {}, but the distance to point is {}".format(effector_reach_limit, first_rev_joint_point.distance_to_point(Point(dest_point))))

        if method.lower() == "fabrik":
            # FABRIK 
            theta_1 = float(atan2(dest_point[1], dest_point[0])) # compute theta_1 to omit horizontal move in FABRIK
            dh_matrix[0][0] = theta_1 # replace initial theta_1

            # Compute initial xyz possition of every robot joint
            _, fk_all = forward_kinematics(*dh_matrix)
            init_joints_positions = [Point([x[0][3], x[1][3], x[2][3]]) for x in fk_all]
            PRINT_MSG('Initial joints positions:    ' + str(init_joints_positions))

            PRINT_MSG('Dest joints positions:    ' + str(dest_point))

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

# test ANN
if __name__ == '__main__':
    dh_matrix = [[0, pi/2, 0, 0], [2, 0, 0, 0], [0, 2, 2, 2], [pi/2, 0, 0, 0]]
    effector_workspace_limits = {'x_limits': [1,6], 'y_limits': [-6,6], 'z_limits': [0,6]} # assumed effector workspace limits
    joints_angles_limits = {'theta_1': [-pi/2,pi/2], 'theta_2': [-pi/4,pi/2], 'theta_3': [-pi/2,pi/2], 'theta_4': [-pi/2,pi/2]} # assumed joints angles limits
    max_reach_distance = 6
    ann = ANN(joints_angles_limits, effector_workspace_limits, dh_matrix)
    ann.train_model(10000, 1000)
    ik_angles_ann = list(ann.predict_ik([[4, 0, 3]]))
    print('ANN    IK angles: ' + str(ik_angles_ann))

    # FABRIK
    dest_point = [4, 0, 3]
    joints_lengths = [2, 2, 2, 2]
    effector_workspace_limits = {'x_limits': [1,6], 'y_limits': [-6,6], 'z_limits': [0,6]} # assumed limits
    first_rev_joint_point = Point([0,0,2]) # first revolute joint, from this point reach limit will be computed
    fkine = InverseKinematics()
    ik_angles = fkine.compute_roboarm_ik('FABRIK', dest_point, dh_matrix, joints_lengths, effector_workspace_limits, first_rev_joint_point, 0.001, 100, False)
    print('FABRIK IK angles: ' + str(ik_angles))

# test FABRIK
'''
def plot_trajectory(points):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    colors = [list(x) for x in numpy.random.rand(len(points),3)] 

    for p,c in zip(points,colors):
        ax.scatter(round(p.x, 1), round(p.y, 5), round(p.z, 5), color=c)

    plt.show()

if __name__ == '__main__':
    # Compute positions of all joints in robot init (base) position
    dh_matrix = [[0, pi/2, 0, 0], [2, 0, 0, 0], [0, 2, 2, 2], [pi/2, 0, 0, 0]]
    dest_point = [1, 5, 3]
    joints_lengths = [2, 2, 2, 2]
    effector_workspace_limits = {'x_limits': [1,6], 'y_limits': [-6,6], 'z_limits': [0,6]} # assumed limits
    first_rev_joint_point = Point([0,0,2]) # first revolute joint, from this point reach limit will be computed
    fkine = InverseKinematics()
    ik_angles = []
    
    # generate circle trajectory:
    dest_points_circle=[]
    for t in range(10):
        r=2
        x=3
        y=r*sin(t)
        z=3+r*cos(t)
        dest_points_circle.append([x, y, z])
        print([x, y, z])

    fk_trajectory_points = []
    plot_trajectory([Point(x) for x in dest_points_circle]) # plot trajectory
    for dest_point in dest_points_circle:
        try:
            ik_angles = fkine.compute_roboarm_ik('FABRIK', dest_point, dh_matrix, joints_lengths, effector_workspace_limits, first_rev_joint_point, 0.001, 100, VERBOSE)
            print('IK angles: ' + str(ik_angles))
            dh_matrix_out = [ik_angles, [2, 0, 0, 0], [0, 2, 2, 2], [pi/2, 0, 0, 0]]
            fk, _ = forward_kinematics(*dh_matrix_out)
            print('IK destination: ' + str(dest_point))
            print('FK destination: ' + str([round(fk[0,3], 3), fk[1,3], fk[2,3]]))
            err_list = [abs(round(fk[0,3], 3) - dest_point[0]), abs(fk[1,3] - dest_point[1]), abs(fk[2,3] - dest_point[2])]
            # print('ER destination: ' + str(err_list))
            if any(x > 0.001 for x in err_list):
                raise Exception('position error greater than 0.001: ' + str(err_list))
            fk_trajectory_points.append([fk[0,3], fk[1,3], fk[2,3]])
            print()
        except Exception as e:
            print(e)
    
    plot_trajectory([Point(x) for x in fk_trajectory_points]) # plot trajectory from fk
'''