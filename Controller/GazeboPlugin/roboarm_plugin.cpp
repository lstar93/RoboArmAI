#include "roboarm_plugin.h"

namespace gazebo
{
  void RoboArmPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
  {
    // Safety check
    if (_model->GetJointCount() == 0)
    {
      std::cerr << "Invalid joint count, RoboArm plugin not loaded\n";
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
      SetJointPositionPID(joint, position_pid);
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

  void RoboArmPlugin::SetJointPositionPID(physics::JointPtr p_joint, common::PID &r_pid)
  {
    this->model->GetJointController()->SetPositionPID(p_joint->GetScopedName(), r_pid);
  }

  void RoboArmPlugin::SetJointVelocityPID(physics::JointPtr p_joint, common::PID &r_pid)
  {
    this->model->GetJointController()->SetVelocityPID(p_joint->GetScopedName(), r_pid);
  }

  void RoboArmPlugin::SetJointTargetPosition(physics::JointPtr p_joint, double pos_in_radians, double min_pos, double max_pos)
  {
    if (pos_in_radians > max_pos)
    {
      std::cout << "ERROR: target postion can't be larger than " << max_pos << "!" << std::endl;
      pos_in_radians = max_pos;
    }
    else if (pos_in_radians < min_pos)
    {
      std::cout << "ERROR: target postion can't be less than " << min_pos << "!" << std::endl;
      pos_in_radians = min_pos;
    }
    
    this->model->GetJointController()->SetPositionTarget(p_joint->GetScopedName(), pos_in_radians);
  }

  void RoboArmPlugin::SetJointTargetVelocity(physics::JointPtr p_joint, uint8_t vel_in_meters_per_sec, uint8_t max_vel)
  {
    if (vel_in_meters_per_sec > max_vel)
    {
      std::cout << "ERROR: target velocity can't be larger than " << max_vel << " meters per second!" << std::endl;
      vel_in_meters_per_sec = max_vel;
    }

    this->model->GetJointController()->SetPositionTarget(p_joint->GetScopedName(), vel_in_meters_per_sec);
  }
} // namespace gazebo