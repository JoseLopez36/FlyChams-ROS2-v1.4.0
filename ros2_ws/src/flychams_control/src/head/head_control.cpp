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
		central_cmd_.id = central_head_ptr->id;
		const auto& central_head_rpy = Vector3r(central_head_ptr->orientation.x(), 0.0f, central_head_ptr->orientation.z());
		RosUtils::toMsg(MathUtils::eulerToQuaternion(central_head_rpy), central_cmd_.ori);
		central_cmd_.fov = MathUtils::computeFov(central_head_ptr->ref_focal, central_head_ptr->camera.sensor_width);

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
				// Add head setpoints to vectors
				head_ids.push_back(head_setpoints_.head_ids[i]);
				head_orientations.push_back(head_setpoints_.quat_setpoints[i]);
				head_fovs.push_back(head_setpoints_.fov_setpoints[i]);
			}
		}

		// Send commands to heads
		framework_tools_->setGimbalOrientations(agent_id_, head_ids, head_orientations);
		framework_tools_->setCameraFovs(agent_id_, head_ids, head_fovs);
	}

} // namespace flychams::control