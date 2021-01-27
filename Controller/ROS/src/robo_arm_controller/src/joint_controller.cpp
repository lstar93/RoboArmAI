#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64MultiArray.h"
#include <vector>
#include <thread>
#include <chrono>
#include "json/json.h"

Json::Value configuration;
bool configurationReady = false;
void roboArmConfigurationCallback(const std_msgs::String::ConstPtr &inStr)
{
    std::string strJson = inStr->data.c_str();

    ROS_INFO("strJson: %s", strJson.c_str());

    const auto strJsonLength = static_cast<int>(strJson.length());
    JSONCPP_STRING err;

    Json::CharReaderBuilder builder;
    const std::unique_ptr<Json::CharReader> reader(builder.newCharReader());
    if (!reader->parse(strJson.c_str(), strJson.c_str() + strJsonLength, &configuration, &err))
    {
        ROS_INFO("roboArmConfigurationCallback ERROR");
        return;
    }
    configurationReady = true;
}

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
    std::vector<double> pose_0;
    std::vector<double> pose_1;

    // Create robot configuration subscriber
    // example configuration JSON: {"joints_configuration":{"number_of_joints":5},"name":"RoboArm"}
    auto roboArmConfigurationSubscriber = n.subscribe("/my_robot/configuration", 1000, roboArmConfigurationCallback);

    while (!configurationReady)
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    int numberOfJoints = configuration["joints_configuration"]["number_of_joints"].asInt();
    ROS_INFO("numberOfJoints: %d", numberOfJoints);
    for (int i = 0; i < numberOfJoints; ++i)
    {
        pose_0.push_back(1.0); // fill pose 0 with 1.0 poses
        pose_1.push_back(0.5); // fill pose 0 with 0.5 poses
    }

    if (numberOfJoints == -1)
    {
        std::cout << "Ups, something went wrong with configuration subscriber, numberOfJoints is -1!" << std::endl;
    }

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