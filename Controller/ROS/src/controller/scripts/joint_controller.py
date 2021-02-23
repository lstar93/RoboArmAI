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
    fkine = InverseKinematics()
    ik_angles = fkine.compute_roboarm_ik('FABRIK', dest_point, dh_matrix, 0.001, 100)

    return ik_angles

def robot_configuration_callback():
    return None

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
    
    dest_point0 = [5, 1, 1]
    pos0_anglees = get_angles_ik(dest_point0)
  
    pos0_arr_mtlb = [pos0_anglees[0], pos0_anglees[1], pos0_anglees[2], pos0_anglees[3], 0.0]

    pos0_arr = pos0_arr_mtlb[:]
    
    kinematics_poses_to_gazebo(pos0_arr)
    print('ik pose: ' + str(pos0_arr))

    # pos0_arr_mtlb = [0.0, 1.227148865655292, 1.0329403678444162, -1.6755963382038974, 0]
    # pos0_arr = pos0_arr_mtlb[:]
    # kinematics_poses_to_gazebo(pos0_arr, [0, 2, 3])

    # pos0_arr_mtlb = [0, pi/2, 0, pi/4, 0]
    # pos0_arr = pos0_arr_mtlb[:]
    # kinematics_poses_to_gazebo(pos0_arr)

    pos1_arr_mtlb = [0, pi/2, 0, -pi/4, 0]
    pos1_arr = pos1_arr_mtlb[:]
    kinematics_poses_to_gazebo(pos1_arr)

    pos2_arr_mtlb = [0, pi/2, 0, -pi/4, 0]
    pos2_arr = pos2_arr_mtlb[:]
    kinematics_poses_to_gazebo(pos2_arr)

    pos3_arr_mtlb = [0.0, pi/2, 0, pi/4, 0]
    pos3_arr = pos3_arr_mtlb[:]
    kinematics_poses_to_gazebo(pos3_arr)

    pos4_arr_mtlb = [0, pi, 0, 0, 0]
    pos4_arr = pos4_arr_mtlb[:]
    kinematics_poses_to_gazebo(pos4_arr)

    positions = [(pos0_arr, pos0_arr_mtlb), (pos1_arr, pos1_arr_mtlb), (pos2_arr, pos2_arr_mtlb), (pos3_arr, pos3_arr_mtlb), (pos4_arr, pos4_arr_mtlb)]
    steps = 1
    while not rospy.is_shutdown():
        cnt = 0
        for nextpose in positions:
            if cnt == steps:
                continue
            to_gazebo, from_math = nextpose
            print("Setting pose_{}".format(cnt))
            print('angles: ' + str(from_math))
            pose = Float64MultiArray()
            pose.data = to_gazebo
            pub.publish(pose)
            cnt += 1
            rate.sleep()

if __name__ == '__main__':
    try:
        joint_controller()
    except rospy.ROSInterruptException:
        pass