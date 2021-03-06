#include "roboarm_plugin.h"

template <class SINK, class... ARGS>
void PRINT_MESSAGE(SINK s, const std::string &msg, ARGS... args)
{
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
			PRINT_MESSAGE(stderr, "ERROR: Invalid joint count, RoboArm plugin not loaded!");
			return;
		}

		// Store the model pointer for convenience.
		this->model = _model;

		// Get the first joint. We are making an assumption about the model
		// having one joint that is the rotational joint.
		this->position_pid = common::PID(30, 1, 30); // Setup a PID-controller.
		uint8_t jId = 0;

		PRINT_MESSAGE(stderr, ""); // print new line
		for (const auto &j_num : modelJointPositions)
		{
			physics::JointPtr tmp_joint = nullptr;
			
			try {
				tmp_joint = _model->GetJoints().at(j_num);
			} catch (const std::out_of_range&) {
				PRINT_MESSAGE(stderr, "ERROR: Joint position %d is out of my_robot model joints array range!", j_num);
				return;
			}
			
			RoboArmJoint joint(tmp_joint, tmp_joint->GetScopedName(), jId++); // start numbering the roboarm revolute joints from 0
			joint.clockWisePositive = modelJointClockWisePositive[joint.id];

			// push created joint
			joints.push_back(joint);

			// set base position
			SetJointPositionPID(joint, position_pid);

			PRINT_MESSAGE(stderr, "Joint %s pushed with id %d", joint.name.c_str(), joint.id);
		}
		PRINT_MESSAGE(stderr, ""); // print new line

		JOINTS_NUMBER = joints.size(); // or _model->GetJointCount()

		auto ret = SetJointsPositions({0, 0.75, 0.75, -0.75, 0.75});

		// GAZEBO external topic communication code
		/*{
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
			this->subscriber = this->node->Subscribe(topicName, &RoboArmPlugin::JointPositionCallback, this);
			//}
		}*/

		// Create our ROS node. This acts in a similar manner to the Gazebo node
		{
			// Initialize ros, if it has not already bee initialized.
			if (!ros::isInitialized())
			{
				int argc = 0;
				char **argv = NULL;
				ros::init(argc, argv, "gazebo_client", ros::init_options::NoSigintHandler);
			}

			this->rosNode.reset(new ros::NodeHandle("gazebo_client"));

			// Create a named topic, and subscribe to it.
			ros::SubscribeOptions singleJointPositionSubscriberOptions =
				ros::SubscribeOptions::create<std_msgs::Float64MultiArray>(
					"/" + this->model->GetName() + "/single_joint_position",
					1,
					std::bind(&RoboArmPlugin::SingleJointPositionCallbackROS, this, std::placeholders::_1),
					ros::VoidPtr(), &this->rosQueue);
			this->singleJointPositionSubscriber = this->rosNode->subscribe(singleJointPositionSubscriberOptions);

			// Create a named topic, and subscribe to it.
			ros::SubscribeOptions allJointsPositionSubscriberOptions =
				ros::SubscribeOptions::create<std_msgs::Float64MultiArray>(
					"/" + this->model->GetName() + "/all_joints_positions",
					1,
					std::bind(&RoboArmPlugin::AllJointsPositionCallbackROS, this, std::placeholders::_1),
					ros::VoidPtr(), &this->rosQueue);
			this->allJointsPositionSubscriber = this->rosNode->subscribe(allJointsPositionSubscriberOptions);

			// Robot configuration publisher
			this->jointsConfigurationPublisher = this->rosNode->advertise<std_msgs::String>("/my_robot/configuration", 1000);
			this->confPublisherWorkerThreadPtr = std::thread(std::bind(&RoboArmPlugin::JointsConfigurationPublisherWorker, this));

			// Spin up the queue helper thread.
			this->rosQueueThread = std::thread(std::bind(&RoboArmPlugin::QueueThread, this));
		}
	}

	void RoboArmPlugin::JointsConfigurationPublisherWorker()
	{
		Json::Value root;
		Json::Value data;
		root["name"] = "RoboArm";
		data["number_of_joints"] = static_cast<int>(JOINTS_NUMBER);
		Json::Value jointsDesc;
		int cnt = 0;
		for(const auto& j: joints) {
			Json::Value joint_data;
			joint_data["name"] = j.name;
			joint_data["clock_wise_positive"] = j.clockWisePositive;
			joint_data["id"] = j.id;
			jointsDesc[cnt++] = joint_data;
		}
		data["joints"] = jointsDesc;
		root["joints_configuration"] = data;
		
		ros::Rate loop_rate(0.25);
		while (ros::ok())
		{
			std_msgs::String msg;

			Json::FastWriter writer;
			const std::string json_file = writer.write(root);

			std::stringstream ss;
			ss << json_file.c_str();
			msg.data = ss.str();

			// PRINT_MESSAGE(stderr, "Published configuration: %s", msg.data.c_str());

			jointsConfigurationPublisher.publish(msg);

			ros::spinOnce();

			loop_rate.sleep();

			if(terminatePublisher) {
				break;
			}
		}
	}

	// External gazebo communication
	void RoboArmPlugin::JointPositionCallback(ConstVector3dPtr &_msg)
	{
		auto id = static_cast<int>(_msg->x());
		auto pos = static_cast<double>(_msg->y());

		for (auto &joint : joints)
		{
			if (joint.id == id)
			{
				SetJointTargetPosition(joint, pos);
				return;
			}
		}
		PRINT_MESSAGE(stderr, "Incorrect joint ID %d, maximum joint number is %d", id, JOINTS_NUMBER);
	}

	//// External ROS communication
	void RoboArmPlugin::SingleJointPositionCallbackROS(const std_msgs::Float64MultiArrayConstPtr &_RosMsg)
	{
		PRINT_MESSAGE(stderr, "ENTER: SingleJointPositionCallbackROS");

		if (_RosMsg->data.size() != 2)
		{
			PRINT_MESSAGE(stderr, "Incorrect position array size, should be 2 instead of %d", _RosMsg->data.size());
			return;
		}

		auto id = static_cast<int>(_RosMsg->data[0]);
		auto pos = static_cast<double>(_RosMsg->data[1]);

		for (auto &joint : joints)
		{
			if (joint.id == id)
			{
				SetJointTargetPosition(joint, pos);
				return;
			}
		}
		PRINT_MESSAGE(stderr, "Incorrect joint ID %d, maximum joint number is %d", id, JOINTS_NUMBER);
	}

	void RoboArmPlugin::AllJointsPositionCallbackROS(const std_msgs::Float64MultiArrayConstPtr &_RosMsg)
	{
		PRINT_MESSAGE(stderr, "ENTER: AllJointsPositionCallbackROS");

		if (_RosMsg->data.size() != JOINTS_NUMBER)
		{
			PRINT_MESSAGE(stderr, "Insufficient variables number, should be %d instead of %d", JOINTS_NUMBER, _RosMsg->data.size());
		}

		auto cnt = 0;
		for (auto &joint : joints)
		{
			if (joint.id == cnt)
			{
				SetJointTargetPosition(joint, _RosMsg->data[cnt++]);
				continue;
			}
			PRINT_MESSAGE(stderr, "Joint %d wasn't found!", cnt);
		}
	}

	// ROS helper function that processes messages
	void RoboArmPlugin::QueueThread()
	{
		static const double timeout = 0.01;
		while (this->rosNode->ok())
		{
			this->rosQueue.callAvailable(ros::WallDuration(timeout));
		}
	}
	////

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
	// TODO:
	//	- read min and max revolute joints position from my_robot model xml
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