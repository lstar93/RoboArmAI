#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64MultiArray
from math import atan2, pi
from robot_kinematics import *

# Check how to fix: cp ./src/controller/scripts/robot_kinematics.py ./devel/lib/controller/robot_kinematics.py

# Compute positions of all joints in robot init (base) position
def get_angles_ik(dest_point):
    # Compute positions of all joints in robot init (base) position
    dh_matrix = [[0, pi/2, 0, 0], [2, 0, 0, 0], [0, 2, 2, 2], [pi/2, 0, 0, 0]]
    joints_lengths = [2, 2, 2, 2]
    robo_arm_joint_limits = {'x_limits': [0,6], 'y_limits': [-6,6], 'z_limits': [0,6]} # assumed limits
    robo_arm_reach_limit = 6 # lenght of 3 joint is the limit
    first_rev_joint_point = Point([0,0,2]) # first revolute joint, from this point reach limit will be computed
    fkine = InverseKinematics()
    ik_angles = []
    try:
        ik_angles = fkine.compute_roboarm_ik('FABRIK', dest_point, dh_matrix, joints_lengths, robo_arm_joint_limits, robo_arm_reach_limit, first_rev_joint_point, 0.001, 100)
        # dh_matrix_out = [ik_angles, [2, 0, 0, 0], [0, 2, 2, 2], [pi/2, 0, 0, 0]]
        # fk, _ = forward_kinematics(dh_matrix_out[0], dh_matrix_out[1], dh_matrix_out[2], dh_matrix_out[3])
        # print([fk[0,3], fk[1,3], fk[2,3]])
        return ik_angles
    except Exception as e:
        print(e)

def robot_configuration_callback():
    return None

# Fit IK joint angles into gazebo model joint angles
def kinematics_poses_to_gazebo(pos_arr):
    not_to_move = [0, 2, 3]
    signs = [-1, -1, -1, 1, -1]
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

    dest_points = [[2, 2, 4], [2, -2, 4], [2, -2, 4], [2, -2, 2]]
    steps = 4
    while not rospy.is_shutdown():
        cnt = 0
        for dest in dest_points:
            if cnt == steps:
                continue
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

if __name__ == '__main__':
    try:
        joint_controller()
    except rospy.ROSInterruptException:
        pass