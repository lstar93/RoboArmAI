#ifndef _VELODYNE_PLUGIN_HH_
#define _VELODYNE_PLUGIN_HH_

#include "roboarm_plugin.h"

namespace gazebo
{
  void RoboArmPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
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

  void RoboArmPlugin::SetJointPositionPID(physics::JointPtr p_joint, common::PID &r_pid)
  {
    this->model->GetJointController()->SetPositionPID(
        p_joint->GetScopedName(), r_pid);
  }

  void RoboArmPlugin::SetJointTargetPosition(physics::JointPtr p_joint, double pos_in_radians)
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
} // namespace gazebo
#endif