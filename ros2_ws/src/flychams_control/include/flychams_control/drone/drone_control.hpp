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
	 * ════════════════════════════════════════════════════════════════
	 * @author Jose Francisco Lopez Ruiz
	 * @date 2025-01-29
	 * ════════════════════════════════════════════════════════════════
	 */
	class DroneControl : public core::BaseModule
	{
	public: // Constructor/Destructor
		DroneControl(const core::ID& agent_id, core::NodePtr node, core::ConfigTools::SharedPtr config_tools, core::FrameworkTools::SharedPtr framework_tools, core::TopicTools::SharedPtr topic_tools, core::TransformTools::SharedPtr transform_tools, core::CallbackGroupPtr module_cb_group)
			: BaseModule(node, config_tools, framework_tools, topic_tools, transform_tools, module_cb_group), agent_id_(agent_id)
		{
			init();
		}

	protected: // Overrides
		void onInit() override;
		void onShutdown() override;

	public: // Types
		using SharedPtr = std::shared_ptr<DroneControl>;
		enum class ControlMode
		{
			POSITION,
			VELOCITY
		};
		struct Agent
		{
			// Status data
			core::AgentStatus status;
			bool has_status;
			// Position data
			core::PointMsg position;
			bool has_position;
			// Setpoint data
			core::PointMsg setpoint;
			bool has_setpoint;
			// Subscribers
			core::SubscriberPtr<core::AgentStatusMsg> status_sub;
			core::SubscriberPtr<core::PointStampedMsg> position_sub;
			core::SubscriberPtr<core::PointStampedMsg> setpoint_sub;
			// Constructor
			Agent()
				: status(), has_status(false), position(), has_position(false), setpoint(),
				has_setpoint(false), status_sub(), position_sub(), setpoint_sub()
			{
			}
		};

	private: // Parameters
		core::ID agent_id_;
		float update_rate_;
		float cmd_timeout_;
		// Control mode
		ControlMode control_mode_;

	private: // Data
		// Agent
		Agent agent_;
		// Speed planner
		SpeedPlanner speed_planner_;
		// Step time
		core::Time last_update_time_;

	private: // Callbacks
		void statusCallback(const core::AgentStatusMsg::SharedPtr msg);
		void positionCallback(const core::PointStampedMsg::SharedPtr msg);
		void setpointPositionCallback(const core::PointStampedMsg::SharedPtr msg);

	private: // Control management
		void update();

	private: // Control methods
		void handlePositionControl(const float& dt);
		void handleVelocityControl(const float& dt);

	private: // ROS components
		// Timer
		core::TimerPtr update_timer_;
	};

} // namespace flychams::control