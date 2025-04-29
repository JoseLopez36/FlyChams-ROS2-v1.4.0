#include "flychams_control/head/head_control.hpp"

using namespace flychams::core;

namespace flychams::control
{
	// ════════════════════════════════════════════════════════════════════════════
	// CONSTRUCTOR: Constructor and destructor
	// ════════════════════════════════════════════════════════════════════════════

	void HeadControl::onInit()
	{
		// Get parameters from parameter server
		// Get update rate
		update_rate_ = RosUtils::getParameterOr<float>(node_, "head_control.control_update_rate", 20.0f);

		// Initialize data
		agent_ = Agent();

		// Subscribe to status and head setpoints topics
		agent_.status_sub = topic_tools_->createAgentStatusSubscriber(agent_id_,
			std::bind(&HeadControl::statusCallback, this, std::placeholders::_1), sub_options_with_module_cb_group_);
		agent_.setpoints_sub = topic_tools_->createAgentTrackingSetpointsSubscriber(agent_id_,
			std::bind(&HeadControl::setpointsCallback, this, std::placeholders::_1), sub_options_with_module_cb_group_);

		// Set update timer
		update_timer_ = RosUtils::createTimer(node_, update_rate_,
			std::bind(&HeadControl::update, this), module_cb_group_);
	}

	void HeadControl::onShutdown()
	{
		// Destroy subscribers
		agent_.status_sub.reset();
		agent_.setpoints_sub.reset();
		// Destroy update timer
		update_timer_.reset();
	}

	// ════════════════════════════════════════════════════════════════════════════
	// CALLBACKS: Callback functions
	// ════════════════════════════════════════════════════════════════════════════

	void HeadControl::statusCallback(const core::AgentStatusMsg::SharedPtr msg)
	{
		// Update current status
		agent_.status = static_cast<AgentStatus>(msg->status);
		agent_.has_status = true;
	}

	void HeadControl::setpointsCallback(const core::AgentTrackingSetpointsMsg::SharedPtr msg)
	{
		// Update tracking setpoints
		agent_.setpoints = *msg;
		agent_.has_setpoints = true;
	}

	// ════════════════════════════════════════════════════════════════════════════
	// UPDATE: Update heads
	// ════════════════════════════════════════════════════════════════════════════

	void HeadControl::update()
	{
		// Check if we have a valid status and setpoints
		if (!agent_.has_status || !agent_.has_setpoints)
		{
			RCLCPP_WARN(node_->get_logger(), "Head control: No status or setpoints data received for agent %s",
				agent_id_.c_str());
			return;
		}

		// Check if we are in the correct state to move
		if (agent_.status != AgentStatus::TRACKING)
		{
			RCLCPP_WARN(node_->get_logger(), "Head control: Agent %s is not in the correct state to control heads",
				agent_id_.c_str());
			return;
		}

		// Iterate over all heads to fill vectors
		int n = static_cast<int>(agent_.setpoints.head_ids.size());
		std::vector<ID> head_ids(n);
		std::vector<QuaternionMsg> head_orientations(n);
		std::vector<float> head_fovs(n);
		for (int i = 0; i < n; i++)
		{
			// Get head parameters
			const auto& head_id = agent_.setpoints.head_ids[i];
			const auto& head_ptr = config_tools_->getHead(agent_id_, head_id);
			const float& focal = agent_.setpoints.focals[i];
			const float& sensor_width = head_ptr->camera.sensor_size(0);
			const auto& angles = agent_.setpoints.angles[i];

			// Compute command for this head
			const auto& [head_cmd_fov, head_cmd_ori] = getCommand(focal, sensor_width, angles);

			// Add command to vectors
			head_ids[i] = head_id;
			head_orientations[i] = head_cmd_ori;
			head_fovs[i] = head_cmd_fov;
		}

		// Send commands to heads
		framework_tools_->setGimbalOrientations(agent_id_, head_ids, head_orientations);
		framework_tools_->setCameraFovs(agent_id_, head_ids, head_fovs);
	}

	// ════════════════════════════════════════════════════════════════════════════
	// HEAD: Head methods
	// ════════════════════════════════════════════════════════════════════════════

	std::pair<float, core::QuaternionMsg> HeadControl::getCommand(const float& focal, const float& sensor_width, const core::Vector3Msg& rpy)
	{
		// Create output
		float fov;
		QuaternionMsg ori;

		// Compute field of view
		fov = MathUtils::computeFov(focal, sensor_width);

		// Compute orientation
		Vector3r rpy_vec = Vector3r(rpy.x, rpy.y, rpy.z);
		RosUtils::toMsg(MathUtils::eulerToQuaternion(rpy_vec), ori);

		// Return command
		return std::make_pair(fov, ori);
	}

} // namespace flychams::control