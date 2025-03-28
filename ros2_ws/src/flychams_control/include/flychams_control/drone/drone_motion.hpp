#pragma once

// Control includes
#include "flychams_control/drone/speed_planner.hpp"

// Base module include
#include "flychams_core/base/base_module.hpp"

namespace flychams::control
{
	/**
	 * ════════════════════════════════════════════════════════════════
	 * @brief Motion manager for UAV drones
	 *
	 * @details
	 * This class is responsible for managing the motion of UAV drones.
	 * It handles the motion of the drone, mainly the speed and the goal
	 * position to ensure the drone moves smoothly and reaches the goal
	 * position.
	 *
	 * ════════════════════════════════════════════════════════════════
	 * @author Jose Francisco Lopez Ruiz
	 * @date 2025-01-29
	 * ════════════════════════════════════════════════════════════════
	 */
	class DroneMotion : public core::BaseModule
	{
	public: // Constructor/Destructor
		DroneMotion(const core::ID& agent_id, core::NodePtr node, core::ConfigTools::SharedPtr config_tools, core::FrameworkTools::SharedPtr framework_tools, core::TopicTools::SharedPtr topic_tools, core::TransformTools::SharedPtr transform_tools, core::CallbackGroupPtr module_cb_group)
			: BaseModule(node, config_tools, framework_tools, topic_tools, transform_tools, module_cb_group), agent_id_(agent_id)
		{
			init();
		}

	protected: // Overrides
		void onInit() override;
		void onShutdown() override;

	public: // Types
		using SharedPtr = std::shared_ptr<DroneMotion>;
		enum class MotionMode
		{
			POSITION,
			VELOCITY
		};

	private: // Parameters
		core::ID agent_id_;
		float update_rate_;
		float cmd_timeout_;
		// Motion mode
		MotionMode motion_mode_;

	private: // Data
		// Current status
		core::AgentStatus curr_status_;
		bool has_status_;
		// Current position
		core::PointMsg curr_position_;
		bool has_position_;
		// Setpoint position
		core::PointMsg setpoint_position_;
		bool has_setpoint_;
		// Speed planner
		SpeedPlanner speed_planner_;
		// Step time
		core::Time last_update_time_;

	private: // Callbacks
		void statusCallback(const core::AgentStatusMsg::SharedPtr msg);
		void positionCallback(const core::PointStampedMsg::SharedPtr msg);
		void setpointPositionCallback(const core::PointStampedMsg::SharedPtr msg);

	private: // Motion management
		void update();
		void handlePositionMotion(const float& dt);
		void handleVelocityMotion(const float& dt);

	private:
		// Subscribers
		core::SubscriberPtr<core::AgentStatusMsg> status_sub_;
		core::SubscriberPtr<core::PointStampedMsg> position_sub_;
		core::SubscriberPtr<core::PointStampedMsg> setpoint_position_sub_;
		// Timer
		core::TimerPtr update_timer_;
	};

} // namespace flychams::control