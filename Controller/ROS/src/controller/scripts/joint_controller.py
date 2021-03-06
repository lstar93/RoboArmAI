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
    rate = rospy.Rate(40) # 40hz

    # plot_trajectory([Point(x) for x in dest_points_circle]) # plot trajectory

    cnt = 0.01
    while not rospy.is_shutdown(): 
        r=2
        x=3
        y=r*sin(cnt)
        z=3+r*cos(cnt)
        # Move through the destination points -> circle
        angles = get_angles_ik([x, y, z])
        angles_with_wrist = [*angles, 0.0] # all angles plus wrist
        gazebo_angles = kinematics_poses_to_gazebo(angles_with_wrist[:])
        print("Setting pose_{}".format(cnt))
        print('angles: ' + str(angles_with_wrist))
        print('destination: ' + str([x, y, z]))
        pose = Float64MultiArray()
        pose.data = gazebo_angles
        pub.publish(pose)
        cnt += 0.01
        rate.sleep()
        

if __name__ == '__main__':
    try:
        joint_controller()
    except rospy.ROSInterruptException:
        pass