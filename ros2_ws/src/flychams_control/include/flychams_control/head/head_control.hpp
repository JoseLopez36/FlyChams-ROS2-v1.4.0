#pragma once

// Base module include
#include "flychams_core/base/base_module.hpp"

namespace flychams::control
{
	/**
	 * ════════════════════════════════════════════════════════════════
	 * @brief Controller for UAV heads
	 * ════════════════════════════════════════════════════════════════
	 * @author Jose Francisco Lopez Ruiz
	 * @date 2025-03-04
	 * ════════════════════════════════════════════════════════════════
	 */
	class HeadControl : public core::BaseModule
	{
	public: // Constructor/Destructor
		HeadControl(const core::ID& agent_id, core::NodePtr node, core::ConfigTools::SharedPtr config_tools, core::FrameworkTools::SharedPtr framework_tools, core::TopicTools::SharedPtr topic_tools, core::TransformTools::SharedPtr transform_tools, core::CallbackGroupPtr module_cb_group)
			: BaseModule(node, config_tools, framework_tools, topic_tools, transform_tools, module_cb_group), agent_id_(agent_id)
		{
			init();
		}

	protected: // Overrides
		void onInit() override;
		void onShutdown() override;

	public: // Types
		using SharedPtr = std::shared_ptr<HeadControl>;
		struct Agent
		{
			// Status data
			core::AgentStatus status;
			bool has_status;
			// Setpoint data
			core::AgentTrackingSetpointsMsg setpoints;
			bool has_setpoints;
			// Subscribers
			core::SubscriberPtr<core::AgentStatusMsg> status_sub;
			core::SubscriberPtr<core::AgentTrackingSetpointsMsg> setpoints_sub;
			// Constructor
			Agent()
				: status(), has_status(false), setpoints(), has_setpoints(false), status_sub(), setpoints_sub()
			{
			}
		};

	private: // Parameters
		core::ID agent_id_;
		float update_rate_;

	private: // Data
		// Agent
		Agent agent_;

	private: // Callbacks
		void statusCallback(const core::AgentStatusMsg::SharedPtr msg);
		void setpointsCallback(const core::AgentTrackingSetpointsMsg::SharedPtr msg);

	private: // Head management
		// Update
		void update();

	private: // Head methods
		std::pair<float, core::QuaternionMsg> getCommand(const float& focal, const float& sensor_width, const core::Vector3Msg& rpy);

	private:
		// Timer
		core::TimerPtr update_timer_;
	};

} // namespace flychams::control