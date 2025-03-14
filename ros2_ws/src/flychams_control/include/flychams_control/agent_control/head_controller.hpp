#pragma once

// Standard includes
#include <mutex>

// Base module include
#include "flychams_core/base/base_module.hpp"

namespace flychams::control
{
	/**
	 * ════════════════════════════════════════════════════════════════
	 * @brief Controller for UAV heads
	 *
	 * @details
	 * This class is responsible for controlling the position and yaw
	 * of UAV heads using PID controllers. It manages multiple heads
	 * and their respective control parameters.
	 *
	 * ════════════════════════════════════════════════════════════════
	 * @author Jose Francisco Lopez Ruiz
	 * @date 2025-03-04
	 * ════════════════════════════════════════════════════════════════
	 */
	class HeadController : public core::BaseModule
	{
	public: // Constructor/Destructor
		HeadController(const core::ID& agent_id, core::NodePtr node, core::ConfigTools::SharedPtr config_tools, core::ExternalTools::SharedPtr ext_tools, core::TopicTools::SharedPtr topic_tools, core::TfTools::SharedPtr tf_tools)
			: BaseModule(node, config_tools, ext_tools, topic_tools, tf_tools), agent_id_(agent_id)
		{
			init();
		}

	protected: // Overrides
		void onInit() override;
		void onShutdown() override;

	public: // Types
		using SharedPtr = std::shared_ptr<HeadController>;

	private: // Parameters
		// Agent parameters
		core::ID agent_id_;

		// Central head parameters
		core::ID central_head_id_;
		core::QuaternionMsg central_head_orientation_;
		float central_head_fov_;

	private: // Data
		// Goal
		core::TrackingGoalMsg goal_;
		bool has_goal_;
		// State, odom and goal mutex
		std::mutex mutex_;

	private: // Methods
		// Callbacks
		void goalCallback(const core::TrackingGoalMsg::SharedPtr msg);
		// Update
		void updateControl();

	private:
		// Subscribers
		core::SubscriberPtr<core::TrackingGoalMsg> goal_sub_;
		// Timers
		core::TimerPtr control_timer_;
	};

} // namespace flychams::control