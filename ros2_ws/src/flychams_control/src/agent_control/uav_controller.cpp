#include "flychams_control/agent_control/uav_controller.hpp"

using namespace std::chrono_literals;
using namespace flychams::core;

namespace flychams::control
{
	// ════════════════════════════════════════════════════════════════════════════
	// CONSTRUCTOR: Constructor and destructor
	// ════════════════════════════════════════════════════════════════════════════

	void UAVController::onInit()
	{
		// Get parameters from parameter server
		// Get update rates
		float update_rate = RosUtils::getParameterOr<float>(node_, "uav_control.control_update_rate", 200.0f);

		// Create local position publisher
		local_position_pub_ = node_->create_publisher<PoseStampedMsg>(
			"/mavros/" + agent_id_ + "/setpoint_position/local",
			rclcpp::QoS(rclcpp::KeepLast(10)));

		// Initialize agent data
		has_goal_ = false;
		local_position_msg_.header = RosUtils::createHeader(node_, tf_tools_->getGlobalFrame());
		local_position_msg_.pose.position.x = 0.0;
		local_position_msg_.pose.position.y = 0.0;
		local_position_msg_.pose.position.z = 0.0;
		local_position_msg_.pose.orientation.x = 0.0;
		local_position_msg_.pose.orientation.y = 0.0;
		local_position_msg_.pose.orientation.z = 0.0;
		local_position_msg_.pose.orientation.w = 1.0;

		// Create callback group
		callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

		// Subscribe to goal topic
		goal_sub_ = topic_tools_->createAgentGoalSubscriber(agent_id_,
			std::bind(&UAVController::goalCallback, this, std::placeholders::_1),
			callback_group_);

		// Set update timer
		control_timer_ = RosUtils::createTimerByRate(node_, update_rate,
			std::bind(&UAVController::update, this),
			callback_group_);
	}

	void UAVController::onShutdown()
	{
		// Destroy subscriber
		goal_sub_.reset();
		// Destroy publisher
		local_position_pub_.reset();
		// Destroy update timer
		control_timer_.reset();
	}

	// ════════════════════════════════════════════════════════════════════════════
	// CALLBACKS: Callback functions
	// ════════════════════════════════════════════════════════════════════════════

	void UAVController::goalCallback(const core::AgentGoalMsg::SharedPtr msg)
	{
		// Get target position
		local_position_msg_.pose.position = MsgConversions::fromMsg(msg->position);
		has_goal_ = true;
	}

	// ════════════════════════════════════════════════════════════════════════════
	// UPDATE: Update PID controllers
	// ════════════════════════════════════════════════════════════════════════════

	void UAVController::update()
	{
		// Check if goal is set
		if (!has_goal_)
		{
			RCLCPP_WARN(node_->get_logger(), "UAV controller: No goal set, skipping update");
			return;
		}

		// Publish local position
		local_position_pub_->publish(local_position_msg_);
	}

	// ════════════════════════════════════════════════════════════════════════════
	// METHODS: Helper methods
	// ════════════════════════════════════════════════════════════════════════════

	Vector3r UAVController::computeVelocityCommand(const Vector3r& curr_pos, const Vector3r& target_pos)
	{
		// Get current time
		auto curr_time = RosUtils::getTimeNow(node_);
		float dt = (curr_time - prev_time_).seconds();
		prev_time_ = curr_time;

		// Update PIDs
		Vector3r vel_cmd(
			pid_x_.update(curr_pos.x(), target_pos.x(), dt),
			pid_y_.update(curr_pos.y(), target_pos.y(), dt),
			pid_z_.update(curr_pos.z(), target_pos.z(), dt)
		);

		return vel_cmd;
	}

} // namespace flychams::control