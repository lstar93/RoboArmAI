#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64MultiArray
from math import atan2, pi, cos, sin
from robot_kinematics import *

# Check how to fix: cp ./src/controller/scripts/robot_kinematics.py ./devel/lib/controller/robot_kinematics.py

# IK/FK consts
dh_matrix = [[0, pi/2, 0, 0], [2, 0, 0, 0], [0, 2, 2, 2], [pi/2, 0, 0, 0]]
joints_lengths = [2, 2, 2, 2]
robo_arm_joint_limits = {'x_limits': [0,6], 'y_limits': [-6,6], 'z_limits': [0,6]} # assumed limits
robo_arm_reach_limit = 6 # lenght of 3 last joints is the limit
first_rev_joint_point = Point([0,0,2]) # first revolute joint, from this point reach limit will be computed

# Compute positions of all joints in robot init (base) position
def get_angles_ik(dest_point):
    fkine = InverseKinematics()
    return fkine.compute_roboarm_ik('FABRIK', dest_point, dh_matrix, joints_lengths, robo_arm_joint_limits, robo_arm_reach_limit, first_rev_joint_point, 0.001, 100)

def robot_configuration_callback():
    return None

# Fit IK joint angles into gazebo model joint angles
def kinematics_poses_to_gazebo(pos_arr):
    not_to_move = [0, 2, 3] # some joints angles doesnt need to be changed
    signs = [-1, -1, -1, 1, -1] # some joints sings must be changed
    for i in range(len(pos_arr)):
        if i in not_to_move:
            pos_arr[i] = pos_arr[i] * signs[i]
        else:
            pos_arr[i] = (pos_arr[i] - pi/2) * signs[i]
    return pos_arr

def joint_controller():
    pub = rospy.Publisher('/my_robot/all_joints_positions', Float64MultiArray, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(0.10) # 0.1hz

    # rectangle path
    dest_points = [[2, 2, 4], [2, -2, 4], [2, -2, 4], [2, -2, 2]] # 4 points trajectory

    # generate circle trajectory:
    dest_points_circle=[]
    for t in range(30):
        r=2
        y=3+r*cos(t)
        z=3+r*sin(t)
        dest_points_circle.append([4, y, z])

    while not rospy.is_shutdown():
        cnt = 0
        # Move through the destination points -> circle
        for dest in dest_points_circle:
            angles = get_angles_ik(dest)
            angles_with_wrist = [angles[0], angles[1], angles[2], angles[3], 0.0]
            gazebo_angles = kinematics_poses_to_gazebo(angles_with_wrist[:])
            print("Setting pose_{}".format(cnt))
            print('angles: ' + str(angles_with_wrist))
            pose = Float64MultiArray()
            pose.data = gazebo_angles
            pub.publish(pose)
            cnt += 1
            rate.sleep()

        '''
        # Move through the destination points
        for dest in dest_points:
            angles = get_angles_ik(dest)
            angles_with_wrist = [angles[0], angles[1], angles[2], angles[3], 0.0]
            gazebo_angles = kinematics_poses_to_gazebo(angles_with_wrist[:])
            print("Setting pose_{}".format(cnt))
            print('angles: ' + str(angles_with_wrist))
            pose = Float64MultiArray()
            pose.data = gazebo_angles
            pub.publish(pose)
            cnt += 1
            rate.sleep()
        '''

if __name__ == '__main__':
    try:
        joint_controller()
    except rospy.ROSInterruptException:
        pass