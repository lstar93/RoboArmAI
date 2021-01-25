#ifndef _VELODYNE_PLUGIN_HH_
#define _VELODYNE_PLUGIN_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

namespace gazebo
{
  /// \brief A plugin to control a Velodyne sensor.
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
    virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
      // Safety check
      if (_model->GetJointCount() == 0)
      {
        std::cerr << "Invalid joint count, Velodyne plugin not loaded\n";
        return;
      }

      // Store the model pointer for convenience.
      this->model = _model;

      // Get the first joint. We are making an assumption about the model
      // having one joint that is the rotational joint.
      this->position_pid = common::PID(20, 1, 20); // Setup a PID-controller.
      for (const auto &j_num : modelJointIDs)
      {
        auto joint = _model->GetJoints()[j_num];
        setJointPositionPID(joint, position_pid);
        SetJointTargetPosition(joint, 0.75);
      }

      /*
      // set first joint
      joint_0 = _model->GetJoints()[0];
      
      // Setup a PID-controller.
      // this->pid = common::PID(20, 1, 20);

      // Apply the PID-controller to the joint.
      this->model->GetJointController()->SetPositionPID(
          this->joint_0->GetScopedName(), this->pid);
      
      // Set the joint's target velocity. This target velocity is just
      // for demonstration purposes.
      this->model->GetJointController()->SetPositionTarget(
          this->joint_0->GetScopedName(), 0.75);
      */
    }

    void setJointPositionPID(physics::JointPtr p_joint, common::PID &r_pid)
    {
      this->model->GetJointController()->SetPositionPID(
          p_joint->GetScopedName(), r_pid);
    }

    void SetJointTargetPosition(physics::JointPtr p_joint, double pos_in_radians)
    {
      if (pos_in_radians > 1.57)
      {
        std::cout << "ERROR: target postion can't be larger than 1.57" << std::endl;
        pos_in_radians = 1.57;
      }
      else if (pos_in_radians < -1.57)
      {
        std::cout << "ERROR: target postion can't be less than -1.57" << std::endl;
        pos_in_radians = -1.57;
      }
      this->model->GetJointController()->SetPositionTarget(
          p_joint->GetScopedName(), pos_in_radians);
    }

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