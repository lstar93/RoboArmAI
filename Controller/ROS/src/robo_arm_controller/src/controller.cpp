#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64MultiArray.h"

#include <vector>

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
    ros::init(argc, argv, "talker");

    ros::NodeHandle n;

    ros::Publisher chatter_pub = n.advertise<std_msgs::Float64MultiArray>("/my_robot/all_joints_positions", 1000);

    ros::Rate loop_rate(0.25);

    /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
    int count = 0;
    // Create two sets of arm possitions
    std::vector<double> pose_0{1.0, 1.0, 1.0, 1.0, 1.0};
    std::vector<double> pose_1{0.5, 0.5, 0.5, 0.5, 0.5};

    uint8_t toggler = 0;
    while (ros::ok())
    {
        std_msgs::Float64MultiArray msg;
        if (toggler % 2)
        {
            msg.data = pose_0;
        }
        else
        {
            msg.data = pose_1;
        }
        toggler++;

        // ROS_INFO("%s", msg.data.c_str());

        chatter_pub.publish(msg);

        ros::spinOnce();

        loop_rate.sleep();
        ++count;
    }

    return 0;
}