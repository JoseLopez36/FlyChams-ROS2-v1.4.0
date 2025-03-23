#pragma once

// Standard includes
#include <mutex>

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
		enum class State
		{
			IDLE,                // Initial state, UAV is inactive
			INITIALIZING,        // UAV is being initialized
			DISARMED,            // UAV is disarmed, safe state
			ARMED,               // UAV is armed, ready for takeoff
			TAKING_OFF,          // UAV is taking off
			HOVERING,            // UAV is hovering in place
			MOVING,              // UAV is moving to a goal
			REACHED,             // UAV has reached the goal position
			LANDING,             // UAV is landing
			ERROR                // Error state, requires reset
		};

	private: // Parameters
		core::ID agent_id_;
		float update_rate_;
		float pos_timeout_;
		float hover_altitude_;    // Altitude for hovering
		float takeoff_altitude_;  // Target altitude for takeoff
		float min_speed_;
		float max_speed_;
		float min_distance_;
		float max_distance_;
		float max_acceleration_;
		float speed_slope_;

	private: // Data
		// Current state
		State state_{ State::IDLE };
		// State transition timestamps
		core::Time state_entry_time_;
		// State transition timeouts
		float arm_timeout_{ 5.0f };
		float takeoff_timeout_{ 10.0f };
		float landing_timeout_{ 10.0f };
		// Current position message
		core::PointMsg curr_pos_;
		bool has_odom_;
		// Goal position message
		core::PointMsg goal_pos_;
		bool has_goal_;
		// Speed scheduling data
		float last_speed_;
		core::Time last_speed_update_time_;
		// Mutex
		std::mutex mutex_;

	public: // State control methods
		// Get the current state
		State getState() const { return state_; }
		// State transition requests
		bool requestArm();
		bool requestTakeoff();
		bool requestHover();
		bool requestLand();
		bool requestDisarm();
		bool requestMove();
		// Reset the UAV controller to IDLE state
		bool reset();

	private: // Methods
		// Callbacks
		void odomCallback(const core::OdometryMsg::SharedPtr msg);
		void goalCallback(const core::PositionGoalMsg::SharedPtr msg);
		// State management
		void setState(const State& new_state);
		bool isValidTransition(const State& from, const State& to) const;
		void handleStateTransition();
		// State handlers
		void handleIdleState();
		void handleInitializingState();
		void handleDisarmedState();
		void handleArmedState();
		void handleTakingOffState();
		void handleHoveringState();
		void handleMovingState();
		void handleReachedState();
		void handleLandingState();
		void handleErrorState();
		// Update control
		void update();
		// Helper methods
		float computeSpeed(const core::Vector3r& goal_pos, const core::Vector3r& curr_pos);
		bool checkGoalReached(const core::Vector3r& goal_pos, const core::Vector3r& curr_pos);

	private:
		// Callback group
		core::CallbackGroupPtr callback_group_;
		// Subscribers
		core::SubscriberPtr<core::OdometryMsg> odom_sub_;
		core::SubscriberPtr<core::PositionGoalMsg> goal_sub_;
		// Timer
		core::TimerPtr control_timer_;
	};

} // namespace flychams::control