#include "roboarm_plugin.h"

namespace gazebo
{
	void RoboArmPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
	{
		// Safety check
		if (_model->GetJointCount() == 0)
		{
			fprintf(stderr, "Invalid joint count, RoboArm plugin not loaded\n");
			return;
		}

		// Store the model pointer for convenience.
		this->model = _model;

		// Get the first joint. We are making an assumption about the model
		// having one joint that is the rotational joint.
		this->position_pid = common::PID(30, 5, 60); // Setup a PID-controller.
		uint8_t jId = 0;

		fprintf(stderr, "\n");
		for (const auto &j_num : modelJointPositions)
		{
			auto tmp_joint = _model->GetJoints()[j_num];
			RoboArmJoint joint(tmp_joint, tmp_joint->GetScopedName(), jId++); // start numbering the roboarm revolute joints from 0

			// push created joint
			joints.push_back(joint);
			joints_map[joint.name] = joint.id;

			// set base position
			SetJointPositionPID(joint, position_pid);
			// SetJointTargetPosition(joint, joint_base_position);

			fprintf(stderr, "Joint %s pushed with id %d\n", joint.name.c_str(), joint.id);

			// Access directly via gazebo type
			// auto joint = _model->GetJoints()[j_num];
			// SetJointPositionPID(joint, position_pid);
			// SetJointTargetPosition(joint, 0.75);
		}
		fprintf(stderr, "\n");

		JOINTS_NUMBER = joints.size(); // or _model->GetJointCount()

		auto ret = SetManipulatorBasePosition({0.75, 0.75, 0.75, 0.75, 0.75});
		if(ret == -1) {
			return;
		}
	}

	int RoboArmPlugin::SetManipulatorBasePosition(const std::vector<double>& positions)
	{
		if(positions.size() != JOINTS_NUMBER) {
			fprintf(stderr, "Incorrect number of positions in vector! Should be %ld instead of %ld\n", JOINTS_NUMBER, positions.size());
			return -1;
		}

		fprintf(stderr, "\nManipulator base positions are: \n");
		auto pos_iter = positions.begin();
		for(const auto& joint: joints) {
			SetJointTargetPosition(joint.name, *pos_iter);
			fprintf(stderr, "\t Joint %d has position %f\n", joint.id, *pos_iter);
			pos_iter++;
		}
		fprintf(stderr, "\n");

		return 0;
	}

	// PID for position control

	void RoboArmPlugin::SetJointPositionPID(physics::JointPtr p_joint, common::PID &r_pid)
	{
		this->model->GetJointController()->SetPositionPID(p_joint->GetScopedName(), r_pid);
	}

	void RoboArmPlugin::SetJointPositionPID(RoboArmJoint &p_joint, common::PID &r_pid)
	{
		this->model->GetJointController()->SetPositionPID(p_joint.name, r_pid);
	}

	void RoboArmPlugin::SetJointPositionPID(const std::string &joint_name, common::PID &r_pid)
	{
		this->model->GetJointController()->SetPositionPID(joint_name, r_pid);
	}

	// PID for velocity control

	void RoboArmPlugin::SetJointVelocityPID(physics::JointPtr p_joint, common::PID &r_pid)
	{
		this->model->GetJointController()->SetVelocityPID(p_joint->GetScopedName(), r_pid);
	}

	void RoboArmPlugin::SetJointVelocityPID(RoboArmJoint &p_joint, common::PID &r_pid)
	{
		this->model->GetJointController()->SetVelocityPID(p_joint.name, r_pid);
	}

	void RoboArmPlugin::SetJointVelocityPID(const std::string &joint_name, common::PID &r_pid)
	{
		this->model->GetJointController()->SetVelocityPID(joint_name, r_pid);
	}

	// Position

	void RoboArmPlugin::SetJointTargetPosition(const std::string &joint_name, double pos_in_radians, double min_pos, double max_pos)
	{
		if (pos_in_radians > max_pos)
		{
			fprintf(stderr, "ERROR: target postion can't be larger than %f!\n", max_pos);
			pos_in_radians = max_pos;
		}
		else if (pos_in_radians < min_pos)
		{
			fprintf(stderr, "ERROR: target postion can't be less than %f!\n", min_pos);
			pos_in_radians = min_pos;
		}

		this->model->GetJointController()->SetPositionTarget(joint_name, pos_in_radians);
	}

	void RoboArmPlugin::SetJointTargetPosition(physics::JointPtr p_joint, double pos_in_radians, double min_pos, double max_pos)
	{
		SetJointTargetPosition(p_joint->GetScopedName(), pos_in_radians, min_pos, max_pos);
	}

	void RoboArmPlugin::SetJointTargetPosition(RoboArmJoint &p_joint, double pos_in_radians, double min_pos, double max_pos)
	{
		SetJointTargetPosition(static_cast<physics::JointPtr>(p_joint)->GetScopedName(), pos_in_radians, min_pos, max_pos);
	}

	// Velocity

	void RoboArmPlugin::SetJointTargetVelocity(const std::string &joint_name, double vel_in_meters_per_sec, double max_vel)
	{
		if (vel_in_meters_per_sec > max_vel)
		{
			fprintf(stderr, "ERROR: target velocity can't be larger than %f meters per second!\n", max_vel);
			vel_in_meters_per_sec = max_vel;
		}

		this->model->GetJointController()->SetVelocityTarget(joint_name, vel_in_meters_per_sec);
	}

	void RoboArmPlugin::SetJointTargetVelocity(physics::JointPtr p_joint, double vel_in_meters_per_sec, double max_vel)
	{
		SetJointTargetVelocity(p_joint->GetScopedName(), vel_in_meters_per_sec, max_vel);
	}

	void RoboArmPlugin::SetJointTargetVelocity(RoboArmJoint &p_joint, double vel_in_meters_per_sec, double max_vel)
	{
		SetJointTargetVelocity(static_cast<physics::JointPtr>(p_joint)->GetScopedName(), vel_in_meters_per_sec, max_vel);
	}
} // namespace gazebo