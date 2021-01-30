#!/usr/bin/env python

# license removed for brevity
import rospy
from std_msgs.msg import Float64MultiArray

def robot_configuration_callback():
    return None

def joint_controller():
    pub = rospy.Publisher('/my_robot/all_joints_positions', Float64MultiArray, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(0.25) # 0.25hz
    toggler=0
    while not rospy.is_shutdown():
        pose_0 = Float64MultiArray()
        pose_0.data = [1.0, 1.0, 1.0, 1.0, 1.0]

        pose_1 = Float64MultiArray()
        pose_1.data = [0.5, 0.5, 0.5, 0.5, 0.5]

        if (toggler%2) == 0:
            pub.publish(pose_0)
            print("Setting pose_0")
        else:
            pub.publish(pose_1)
            print("Setting pose_1")
        toggler = toggler + 1

        rate.sleep()

if __name__ == '__main__':
    try:
        joint_controller()
    except rospy.ROSInterruptException:
        pass