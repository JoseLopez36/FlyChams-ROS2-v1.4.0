#pragma once

// Standard includes
#include <mutex>

// PID controller include
#include "flychams_control/agent_control/pid_controller.hpp"

// Base module include
#include "flychams_core/base/base_module.hpp"

namespace flychams::control
{
	/**
	 * ════════════════════════════════════════════════════════════════
	 * @brief Controller for UAV agent position and yaw
	 *
	 * @details
	 * This class is responsible for controlling the position and yaw
	 * of UAV agents using PID controllers. It manages multiple agents
	 * and their respective control parameters.
	 *
	 * ════════════════════════════════════════════════════════════════
	 * @author Jose Francisco Lopez Ruiz
	 * @date 2025-01-29
	 * ════════════════════════════════════════════════════════════════
	 */
	class UAVController : public core::BaseModule
	{
	public: // Constructor/Destructor
		UAVController(const core::ID& agent_id, core::NodePtr node, core::ConfigTools::SharedPtr config_tools, core::ExternalTools::SharedPtr ext_tools, core::TopicTools::SharedPtr topic_tools, core::TfTools::SharedPtr tf_tools)
			: BaseModule(node, config_tools, ext_tools, topic_tools, tf_tools), agent_id_(agent_id)
		{
			init();
		}

	protected: // Overrides
		void onInit() override;
		void onShutdown() override;

	public: // Types
		using SharedPtr = std::shared_ptr<UAVController>;

	private: // Parameters
		core::ID agent_id_;

	private: // Data
		// Position message
		bool has_goal_;
		core::PoseStampedMsg local_position_msg_;

	private: // Methods
		// Callbacks
		void goalCallback(const core::AgentGoalMsg::SharedPtr msg);
		// Update control
		void update();

	private:
		// Callback group
		core::CallbackGroupPtr callback_group_;
		// Subscriber
		core::SubscriberPtr<core::AgentGoalMsg> goal_sub_;
		// Publisher
		core::PublisherPtr<core::PoseStampedMsg> local_position_pub_;
		// Timer
		core::TimerPtr control_timer_;
	};

} // namespace flychams::control