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

		// Initialize agent state
		curr_status_ = AgentStatus::IDLE;
		has_status_ = false;

		// Initialize head setpoints
		head_setpoints_ = AgentHeadSetpointsMsg();
		has_head_setpoints_ = false;

		// Compute central head command
		const auto& central_head_ptr = config_tools_->getCentralHead(agent_id_);
		const auto& central_head_rpy = Vector3r(central_head_ptr->orientation.x(), 0.0f, central_head_ptr->orientation.z());
		Vector3Msg central_head_rpy_msg;
		RosUtils::toMsg(central_head_rpy, central_head_rpy_msg);
		const auto& [central_cmd_fov, central_cmd_ori] = getCommand(central_head_ptr->ref_focal, central_head_ptr->camera.sensor_size(0), central_head_rpy_msg);
		central_cmd_.id = central_head_ptr->id;
		central_cmd_.fov = central_cmd_fov;
		central_cmd_.ori = central_cmd_ori;

		// Subscribe to status and head setpoints topics
		status_sub_ = topic_tools_->createAgentStatusSubscriber(agent_id_,
			std::bind(&HeadControl::statusCallback, this, std::placeholders::_1), sub_options_with_module_cb_group_);
		head_setpoints_sub_ = topic_tools_->createAgentHeadSetpointsSubscriber(agent_id_,
			std::bind(&HeadControl::headSetpointsCallback, this, std::placeholders::_1), sub_options_with_module_cb_group_);

		// Set update timer
		update_timer_ = RosUtils::createTimer(node_, update_rate_,
			std::bind(&HeadControl::update, this), module_cb_group_);
	}

	void HeadControl::onShutdown()
	{
		// Destroy subscribers
		status_sub_.reset();
		head_setpoints_sub_.reset();
		// Destroy update timer
		update_timer_.reset();
	}

	// ════════════════════════════════════════════════════════════════════════════
	// CALLBACKS: Callback functions
	// ════════════════════════════════════════════════════════════════════════════

	void HeadControl::statusCallback(const core::AgentStatusMsg::SharedPtr msg)
	{
		// Update current status
		curr_status_ = static_cast<AgentStatus>(msg->status);
		has_status_ = true;
	}

	void HeadControl::headSetpointsCallback(const core::AgentHeadSetpointsMsg::SharedPtr msg)
	{
		// Update head setpoints
		head_setpoints_ = *msg;
		has_head_setpoints_ = true;
	}

	// ════════════════════════════════════════════════════════════════════════════
	// UPDATE: Update heads
	// ════════════════════════════════════════════════════════════════════════════

	void HeadControl::update()
	{
		// Check if we have a valid status
		if (!has_status_)
		{
			RCLCPP_WARN(node_->get_logger(), "Head control: No status data received for agent %s",
				agent_id_.c_str());
			return;
		}

		// Check if we are in the correct state to move
		if (curr_status_ != AgentStatus::TRACKING)
		{
			RCLCPP_WARN(node_->get_logger(), "Head control: Agent %s is not in the correct state to control heads",
				agent_id_.c_str());
			return;
		}

		// Create command vectors and initialize with central head command
		std::vector<ID> head_ids = { central_cmd_.id };
		std::vector<QuaternionMsg> head_orientations = { central_cmd_.ori };
		std::vector<float> head_fovs = { central_cmd_.fov };

		// Check if head setpoints are set
		if (has_head_setpoints_)
		{
			int n = static_cast<int>(head_setpoints_.head_ids.size());
			for (int i = 0; i < n; i++)
			{
				// Get head parameters
				const auto& head_id = head_setpoints_.head_ids[i];
				const auto& head_ptr = config_tools_->getHead(agent_id_, head_id);
				const float& sensor_width = head_ptr->camera.sensor_size(0);

				// Compute command for this head
				const auto& [head_cmd_fov, head_cmd_ori] = getCommand(head_setpoints_.focal_setpoints[i], sensor_width, head_setpoints_.rpy_setpoints[i]);

				// Add command to vectors
				head_ids.push_back(head_id);
				head_orientations.push_back(head_cmd_ori);
				head_fovs.push_back(head_cmd_fov);
			}
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