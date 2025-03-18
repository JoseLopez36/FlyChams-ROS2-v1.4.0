#include "flychams_control/agent_control/head_controller.hpp"

using namespace std::chrono_literals;
using namespace flychams::core;

namespace flychams::control
{
	// ════════════════════════════════════════════════════════════════════════════
	// CONSTRUCTOR: Constructor and destructor
	// ════════════════════════════════════════════════════════════════════════════

	void HeadController::onInit()
	{
		// Get parameters from parameter server
		// Get update rates
		float update_rate = RosUtils::getParameterOr<float>(node_, "head_control.control_update_rate", 20.0f);

		// Initialize agent data
		goal_ = TrackingGoalMsg();
		has_goal_ = false;

		// Get central camera parameters
		const auto& central_head_id = config_tools_->getAgent(agent_id_)->central_head_id;
		const auto& central_camera_params = config_tools_->getCameraParameters(agent_id_, central_head_id);

		// Calculate central head fixed orientation and fov
		const auto& central_head_config = config_tools_->getHead(agent_id_, central_head_id);
		const auto& central_head_rpy = Vector3r(0.0f, 0.0f, 0.0f);
		QuaternionMsg central_head_orientation;
		MsgConversions::toMsg(MathUtils::eulerToQuaternion(central_head_rpy), central_head_orientation);
		float central_head_fov = CameraUtils::computeFov(central_camera_params.f_ref, central_camera_params.sensor_width);

		// Set central camera goals (only first time)
		ext_tools_->setGimbalOrientations(agent_id_, { central_head_id }, { central_head_orientation });
		ext_tools_->setCameraFovs(agent_id_, { central_head_id }, { central_head_fov });

		// Subscribe to goal topic
		goal_sub_ = topic_tools_->createAgentTrackingGoalSubscriber(agent_id_,
			std::bind(&HeadController::goalCallback, this, std::placeholders::_1));

		// Set update timer
		control_timer_ = RosUtils::createTimerByRate(node_, update_rate,
			std::bind(&HeadController::update, this));
	}

	void HeadController::onShutdown()
	{
		// Lock mutex
		std::lock_guard<std::mutex> lock(mutex_);

		// Destroy subscribers
		goal_sub_.reset();
		// Destroy update timer
		control_timer_.reset();
	}

	// ════════════════════════════════════════════════════════════════════════════
	// CALLBACKS: Callback functions
	// ════════════════════════════════════════════════════════════════════════════

	void HeadController::goalCallback(const core::TrackingGoalMsg::SharedPtr msg)
	{
		// Lock mutex
		std::lock_guard<std::mutex> lock(mutex_);

		goal_ = *msg;
		has_goal_ = true;
	}

	// ════════════════════════════════════════════════════════════════════════════
	// UPDATE: Update PID controllers
	// ════════════════════════════════════════════════════════════════════════════

	void HeadController::update()
	{
		// Lock mutex
		std::lock_guard<std::mutex> lock(mutex_);

		// Check if tracking goal is set
		if (!has_goal_)
			return;

		// Command tracking heads to move to given angles and fovs
		if (goal_.head_ids.size() >= 1)
		{
			ext_tools_->setGimbalOrientations(agent_id_, goal_.head_ids, goal_.orientations);
			ext_tools_->setCameraFovs(agent_id_, goal_.head_ids, goal_.fovs);
		}
	}

} // namespace flychams::control