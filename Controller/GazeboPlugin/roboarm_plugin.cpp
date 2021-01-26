#include "roboarm_plugin.h"

template<class SINK, class ... ARGS>
void PRINT_MESSAGE(SINK s, const std::string& msg, ARGS... args) {
	#ifdef PRINT_VERBOSE
		fprintf(s, (msg + "\n").c_str(), std::forward<ARGS>(args)...);
	#endif
}

namespace gazebo
{
	void RoboArmPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
	{
		// Safety check
		if (_model->GetJointCount() == 0)
		{
			PRINT_MESSAGE(stderr, "Invalid joint count, RoboArm plugin not loaded!");
			return;
		}

		// Store the model pointer for convenience.
		this->model = _model;

		// Get the first joint. We are making an assumption about the model
		// having one joint that is the rotational joint.
		this->position_pid = common::PID(30, 5, 60); // Setup a PID-controller.
		uint8_t jId = 0;

		PRINT_MESSAGE(stderr,""); // print new line
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

			// fprintf(stderr, "Joint %s pushed with id %d\n", joint.name.c_str(), joint.id);
			PRINT_MESSAGE(stderr,"Joint %s pushed with id %d", joint.name.c_str(), joint.id);

			// Access directly via gazebo type
			// auto joint = _model->GetJoints()[j_num];
			// SetJointPositionPID(joint, position_pid);
			// SetJointTargetPosition(joint, 0.75);
		}
		PRINT_MESSAGE(stderr,""); // print new line

		JOINTS_NUMBER = joints.size(); // or _model->GetJointCount()

		auto ret = SetJointsPositions({0.75, 0.75, 0.75, 0.75, 0.75});

		// ROS topic communication
		//for (const auto &joint : joints)
		//{
			// Create the node
			this->node = transport::NodePtr(new transport::Node());
#if GAZEBO_MAJOR_VERSION < 8
			this->node->Init(this->model->GetWorld()->GetName());
#else
			this->node->Init(this->model->GetWorld()->Name());
#endif
			// Create a topic name
			std::string topicName = "~/" + this->model->GetName() + "/position";
			PRINT_MESSAGE(stderr, "Topic name is %s", topicName.c_str());

			// Subscribe to the topic, and register a callback
			this->subscriber = this->node->Subscribe(topicName, &RoboArmPlugin::OnMsg, this);
		//}
	}

	void RoboArmPlugin::OnMsg(ConstVector3dPtr &_msg)
	{
		auto id = static_cast<int>(_msg->x());
		auto pos = static_cast<double>(_msg->y());
		PRINT_MESSAGE(stderr, "Callback invoked from external program, joint %d will be set to %f position", id, pos);
		for (auto &joint : joints)
		{
			if (joint.id == id)
			{
				SetJointTargetPosition(joint, pos);
				return;
			}
		}
	}

	int RoboArmPlugin::SetJointsPositions(const std::vector<double> &positions)
	{
		if (positions.size() != JOINTS_NUMBER)
		{
			PRINT_MESSAGE(stderr, "Incorrect number of positions in vector! Should be %ld instead of %ld", JOINTS_NUMBER, positions.size());
			return -1;
		}

		auto pos_iter = positions.begin();
		for (const auto &joint : joints)
		{
			SetJointTargetPosition(joint.name, *pos_iter);
			pos_iter++;
		}

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
			PRINT_MESSAGE(stderr, "ERROR: target postion can't be larger than %f!", max_pos);
			pos_in_radians = max_pos;
		}
		else if (pos_in_radians < min_pos)
		{
			PRINT_MESSAGE(stderr, "ERROR: target postion can't be less than %f!", min_pos);
			pos_in_radians = min_pos;
		}

		PRINT_MESSAGE(stderr, "Setting %s joint to %f position", joint_name.c_str(), pos_in_radians);

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
			PRINT_MESSAGE(stderr, "ERROR: target velocity can't be larger than %f meters per second!", max_vel);
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