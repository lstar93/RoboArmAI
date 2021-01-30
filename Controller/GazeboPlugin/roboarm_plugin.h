#ifndef _ROBOARM_PLUGIN_H_
#define _ROBOARM_PLUGIN_H_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <thread>
#include <memory>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float64MultiArray.h"
#include <sstream>
#include "json/json.h"

namespace gazebo
{
    struct RoboArmJoint
    {
        const uint8_t id;
        const std::string name;
        bool clockWisePositive = true;
        physics::JointPtr _this;
        double basePosition, currentPosition, targetPosition;
        double baseVelocity, currentVelocity, targetVelocity;
        RoboArmJoint(physics::JointPtr j, const std::string &_name, const uint8_t _id, double _basePosition = 0.0)
            : _this(j), name(_name), id(_id), basePosition(_basePosition), currentPosition(_basePosition) {}

        explicit operator physics::JointPtr() const { return _this; }
    };

    /// \brief A plugin to control a ROBOARM sensor.
    class RoboArmPlugin : public ModelPlugin
    {
        /// \brief Constructor
    public:
        RoboArmPlugin() {}
        ~RoboArmPlugin()
        {
            // stop configuration publisher thread loop
            terminatePublisher = true;

            // if publisher worker thread is still alive wait to its end
            if (confPublisherWorkerThreadPtr.joinable())
            {
                confPublisherWorkerThreadPtr.join();
            }
        }

        /// \brief The load function is called by Gazebo when the plugin is
        /// inserted into simulation
        /// \param[in] _model A pointer to the model that this plugin is
        /// attached to.
        /// \param[in] _sdf A pointer to the plugin's SDF element.
        virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
        void InitializeJointsTopology();
        int SetJointsPositions(const std::vector<double> &positions);

        // set position pid
        void SetJointPositionPID(physics::JointPtr p_joint, common::PID &r_pid);
        void SetJointPositionPID(RoboArmJoint &p_joint, common::PID &r_pid);
        void SetJointPositionPID(const std::string &joint_name, common::PID &r_pid);

        // set velocity pid
        void SetJointVelocityPID(physics::JointPtr p_joint, common::PID &r_pid);
        void SetJointVelocityPID(RoboArmJoint &p_joint, common::PID &r_pid);
        void SetJointVelocityPID(const std::string &joint_name, common::PID &r_pid);

        // position control
        void SetJointTargetPosition(physics::JointPtr p_joint, double pos_in_radians, double min_pos = -1.57, double max_pos = 1.57);
        void SetJointTargetPosition(RoboArmJoint &p_joint, double pos_in_radians, double min_pos = -1.57, double max_pos = 1.57);
        void SetJointTargetPosition(const std::string &joint_name, double pos_in_radians, double min_pos = -1.57, double max_pos = 1.57); // ROS interface

        // velocity control
        void SetJointTargetVelocity(physics::JointPtr p_joint, double vel_in_meters_per_sec, double max_vel = 5.0);
        void SetJointTargetVelocity(RoboArmJoint &p_joint, double vel_in_meters_per_sec, double max_vel = 5.0);
        void SetJointTargetVelocity(const std::string &joint_name, double vel_in_meters_per_sec, double max_vel = 5.0); // ROS interface

    private:
        /// \brief Pointer to the model.
        physics::ModelPtr model;

        /// \brief vector of joints
        std::vector<RoboArmJoint> joints;
        size_t JOINTS_NUMBER = 0;

        const std::vector<uint8_t> modelJointPositions{0, 2, 6, 9, 12}; // hardcoded from gazebo my_robot XML
        const std::vector<bool> modelJointClockWisePositive{true, true, true, false, true}; // hardcoded from gazebo my_robot XML // clock wise negative means that joint need negative radians to move in a clock wise direction

        /// \brief A PID controller for the joint.
        common::PID position_pid;

        // GAZEBO EXTERNAL INTERFACE
        /// \brief A node used for transport
        transport::NodePtr node;
        /// \brief A subscriber to a named topic.
        transport::SubscriberPtr subscriber;
        /// \brief Callback for ros topic
        void JointPositionCallback(ConstVector3dPtr &_msg);

        // ROS INTERFACE
        /// \brief A node use for ROS transport
        std::unique_ptr<ros::NodeHandle> rosNode;
        /// \brief A ROS subscriber
        ros::Subscriber singleJointPositionSubscriber, allJointsPositionSubscriber;
        /// \brief A ROS callbackqueue that helps process messages
        ros::CallbackQueue rosQueue;
        /// \brief A thread the keeps running the rosQueue
        std::thread rosQueueThread;
        void SingleJointPositionCallbackROS(const std_msgs::Float64MultiArrayConstPtr &_RosMsg);
        void AllJointsPositionCallbackROS(const std_msgs::Float64MultiArrayConstPtr &_RosMsg);
        void QueueThread();
        ros::Publisher jointsConfigurationPublisher;
        void JointsConfigurationPublisherWorker();
        std::thread confPublisherWorkerThreadPtr;
        bool terminatePublisher = false;
    };

    // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
    GZ_REGISTER_MODEL_PLUGIN(RoboArmPlugin)
} // namespace gazebo
#endif