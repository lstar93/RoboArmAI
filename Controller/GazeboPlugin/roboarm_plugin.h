#ifndef _ROBOARM_PLUGIN_HH_
#define _ROBOARM_PLUGIN_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

namespace gazebo
{
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

    void SetJointPositionPID(physics::JointPtr p_joint, common::PID &r_pid);

    void SetJointTargetPosition(physics::JointPtr p_joint, double pos_in_radians);

  private:
    /// \brief Pointer to the model.
    physics::ModelPtr model;

    /// \brief Pointer to the joint.
    // physics::JointPtr joint_0;

    const std::vector<uint8_t> modelJointIDs{0, 2, 6, 9, 12};

    /// \brief A PID controller for the joint.
    common::PID position_pid;
  };

  // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
  GZ_REGISTER_MODEL_PLUGIN(RoboArmPlugin)
} // namespace gazebo
#endif