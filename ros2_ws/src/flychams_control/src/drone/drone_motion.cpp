#include "flychams_control/drone/drone_motion.hpp"

using namespace flychams::core;

namespace flychams::control
{
	// ════════════════════════════════════════════════════════════════════════════
	// CONSTRUCTOR: Constructor and destructor
	// ════════════════════════════════════════════════════════════════════════════

	void DroneMotion::onInit()
	{
		// Get parameters from parameter server
		// Get update rate
		update_rate_ = RosUtils::getParameterOr<float>(node_, "drone_motion.motion_update_rate", 10.0f);
		// Get motion mode
		motion_mode_ = static_cast<MotionMode>(RosUtils::getParameterOr<uint8_t>(node_, "drone_motion.motion_mode", 0));
		// Get speed planner parameters
		float min_speed = RosUtils::getParameterOr<float>(node_, "drone_motion.min_speed", 0.5f);
		float max_speed = RosUtils::getParameterOr<float>(node_, "drone_motion.max_speed", 12.0f);
		float min_distance = RosUtils::getParameterOr<float>(node_, "drone_motion.min_distance", 0.20f);
		float max_distance = RosUtils::getParameterOr<float>(node_, "drone_motion.max_distance", 50.0f);
		float max_acceleration = RosUtils::getParameterOr<float>(node_, "drone_motion.max_acceleration", 2.0f);

		// Compute command timeout
		cmd_timeout_ = (1.0f / update_rate_) * 1.25f;

		// Initialize agent state
		curr_status_ = AgentStatus::IDLE;
		has_status_ = false;

		// Initialize agent position
		curr_position_ = PointMsg();
		has_position_ = false;

		// Initialize setpoint position
		setpoint_position_ = PointMsg();
		has_setpoint_ = false;

		// Set speed planner parameters
		speed_planner_.setParameters(min_speed, max_speed, min_distance, max_distance, max_acceleration);

		// Subscribe to status, position and setpoint topics
		status_sub_ = topic_tools_->createAgentStatusSubscriber(agent_id_,
			std::bind(&DroneMotion::statusCallback, this, std::placeholders::_1), sub_options_with_module_cb_group_);
		position_sub_ = topic_tools_->createAgentPositionSubscriber(agent_id_,
			std::bind(&DroneMotion::positionCallback, this, std::placeholders::_1), sub_options_with_module_cb_group_);
		setpoint_position_sub_ = topic_tools_->createAgentPositionSetpointSubscriber(agent_id_,
			std::bind(&DroneMotion::setpointPositionCallback, this, std::placeholders::_1), sub_options_with_module_cb_group_);

		// Set update timer
		last_update_time_ = RosUtils::now(node_);
		update_timer_ = RosUtils::createTimer(node_, update_rate_,
			std::bind(&DroneMotion::update, this), module_cb_group_);
	}

	void DroneMotion::onShutdown()
	{
		// Destroy subscribers
		status_sub_.reset();
		position_sub_.reset();
		setpoint_position_sub_.reset();
		// Destroy update timer
		update_timer_.reset();
	}

	// ════════════════════════════════════════════════════════════════════════════
	// CALLBACKS: Callback functions
	// ════════════════════════════════════════════════════════════════════════════

	void DroneMotion::statusCallback(const core::AgentStatusMsg::SharedPtr msg)
	{
		// Update current status
		curr_status_ = static_cast<AgentStatus>(msg->status);
		has_status_ = true;
	}

	void DroneMotion::positionCallback(const core::PointStampedMsg::SharedPtr msg)
	{
		// Update current position
		curr_position_ = msg->point;
		has_position_ = true;
	}

	void DroneMotion::setpointPositionCallback(const core::PointStampedMsg::SharedPtr msg)
	{
		// Update setpoint position
		setpoint_position_ = msg->point;
		has_setpoint_ = true;
	}

	// ════════════════════════════════════════════════════════════════════════════
	// UPDATE: Update motion
	// ════════════════════════════════════════════════════════════════════════════

	void DroneMotion::update()
	{
		// Check if we have a valid status, position and setpoint
		if (!has_status_ || !has_position_ || !has_setpoint_)
		{
			RCLCPP_WARN(node_->get_logger(), "Drone motion: No status, position or setpoint data received for agent %s",
				agent_id_.c_str());
			return;
		}

		// Check if we are in the correct state to move
		if (curr_status_ != AgentStatus::TRACKING)
		{
			RCLCPP_WARN(node_->get_logger(), "Drone motion: Agent %s is not in the correct state to move",
				agent_id_.c_str());
			return;
		}

		// Compute time step
		auto current_time = RosUtils::now(node_);
		float dt = (current_time - last_update_time_).seconds();
		last_update_time_ = current_time;

		// Limit dt to prevent extreme values after pauses
		dt = std::min(dt, cmd_timeout_);

		switch (motion_mode_)
		{
		case MotionMode::POSITION:
			handlePositionMotion(dt);
			break;

		case MotionMode::VELOCITY:
			handleVelocityMotion(dt);
			break;

		default:
			RCLCPP_ERROR(node_->get_logger(), "Drone motion: Invalid motion mode for agent %s",
				agent_id_.c_str());
			break;
		}
	}

	void DroneMotion::handlePositionMotion(const float& dt)
	{
		// Plan speed based on distance to goal and other criteria
		float target_speed = speed_planner_.planSpeed(curr_position_.x, curr_position_.y, curr_position_.z, setpoint_position_.x, setpoint_position_.y, setpoint_position_.z, dt);

		// Send command to move to goal position
		framework_tools_->enableControl(agent_id_, true);
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
		framework_tools_->setPosition(agent_id_, setpoint_position_.x, setpoint_position_.y, setpoint_position_.z, target_speed, dt * 1000.0f);
	}

	void DroneMotion::handleVelocityMotion(const float& dt)
	{
		// TODO: Implement velocity motion
	}

} // namespace flychams::control