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
      this->joint_0 = _model->GetJoints()[0];
      this->joint_1 = _model->GetJoints()[2];
      this->joint_2 = _model->GetJoints()[6];
      this->joint_3 = _model->GetJoints()[9];
      this->joint_4 = _model->GetJoints()[12];

      // Setup a PID-controller.
      this->pid = common::PID(20, 1, 20);

      // Apply the PID-controller to the joint.
      this->model->GetJointController()->SetPositionPID(
          this->joint_0->GetScopedName(), this->pid);
      this->model->GetJointController()->SetPositionPID(
          this->joint_1->GetScopedName(), this->pid);
      this->model->GetJointController()->SetPositionPID(
          this->joint_2->GetScopedName(), this->pid);
      this->model->GetJointController()->SetPositionPID(
          this->joint_3->GetScopedName(), this->pid);
      this->model->GetJointController()->SetPositionPID(
          this->joint_4->GetScopedName(), this->pid);

      // Set the joint's target velocity. This target velocity is just
      // for demonstration purposes.
      this->model->GetJointController()->SetPositionTarget(
          this->joint_0->GetScopedName(), 0.75);
      this->model->GetJointController()->SetPositionTarget(
          this->joint_1->GetScopedName(), -0.75);
      this->model->GetJointController()->SetPositionTarget(
          this->joint_2->GetScopedName(), 0.75);
      this->model->GetJointController()->SetPositionTarget(
          this->joint_3->GetScopedName(), 0.75);
      this->model->GetJointController()->SetPositionTarget(
          this->joint_4->GetScopedName(), 0.75);
    }

  private:
    /// \brief Pointer to the model.
    physics::ModelPtr model;

    /// \brief Pointer to the joint.
    physics::JointPtr joint_0;
    physics::JointPtr joint_1;
    physics::JointPtr joint_2;
    physics::JointPtr joint_3;
    physics::JointPtr joint_4;

    /// \brief A PID controller for the joint.
    common::PID pid;
  };

  // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
  GZ_REGISTER_MODEL_PLUGIN(RoboArmPlugin)
} // namespace gazebo
#endif