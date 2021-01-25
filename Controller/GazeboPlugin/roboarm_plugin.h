#ifndef _ROBOARM_PLUGIN_HH_
#define _ROBOARM_PLUGIN_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

namespace gazebo
{
    struct RoboArmJoint
    {
        const uint8_t id;
        const std::string name;
        physics::JointPtr _this;
        double basePosition, currentPosition, targetPosition;
        double baseVelocity, currentVelocity, targetVelocity;
        RoboArmJoint(physics::JointPtr j, const std::string &_name, const uint8_t _id, double _basePosition = 0.0) : _this(j), name(_name), id(_id), basePosition(_basePosition), currentPosition(_basePosition) {}

        explicit operator physics::JointPtr() const { return _this; }
    };

    /// \brief A plugin to control a ROBOARM sensor.
    class RoboArmPlugin : public ModelPlugin
    {
        /// \brief Constructor
    public:
        RoboArmPlugin() {}

        /// \brief The load function is called by Gazebo when the plugin is
        /// inserted into simulation
        /// \param[in] _model A pointer to the model that this plugin is
        /// attached to.
        /// \param[in] _sdf A pointer to the plugin's SDF element.
        virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
        void InitializeJointsTopology();
        int SetManipulatorBasePosition(const std::vector<double>& positions);

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

        /// \brief Pointer to the joint.
        // physics::JointPtr joint_0;

        /// \brief vector of joints
        std::vector<RoboArmJoint> joints;
        std::map<const std::string, uint8_t> joints_map;
        size_t JOINTS_NUMBER = 0;

        const std::vector<uint8_t> modelJointPositions{0, 2, 6, 9, 12};

        /// \brief A PID controller for the joint.
        common::PID position_pid;
    };

    // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
    GZ_REGISTER_MODEL_PLUGIN(RoboArmPlugin)
} // namespace gazebo
#endif