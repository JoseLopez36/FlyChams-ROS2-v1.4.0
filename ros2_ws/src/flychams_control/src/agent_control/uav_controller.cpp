#include "flychams_control/agent_control/uav_controller.hpp"

using namespace std::chrono_literals;
using namespace flychams::core;

namespace flychams::control
{
	// ════════════════════════════════════════════════════════════════════════════
	// CONSTRUCTOR: Constructor and destructor
	// ════════════════════════════════════════════════════════════════════════════

	void UAVController::onInit()
	{
		// Get parameters from parameter server
		// Get update rates
		update_rate_ = RosUtils::getParameterOr<float>(node_, "uav_control.control_update_rate", 10.0f);
		// Get takeoff altitude
		takeoff_altitude_ = RosUtils::getParameterOr<float>(node_, "uav_control.takeoff_altitude", 5.0f);
		// Get timeouts
		takeoff_timeout_ = RosUtils::getParameterOr<float>(node_, "uav_control.takeoff_timeout", 10.0f);
		landing_timeout_ = RosUtils::getParameterOr<float>(node_, "uav_control.landing_timeout", 10.0f);
		// Get goal reach threshold
		goal_reach_threshold_ = RosUtils::getParameterOr<float>(node_, "uav_control.goal_reach_threshold", 0.5f);
		// Get velocity controller parameters
		max_velocity_ = RosUtils::getParameterOr<float>(node_, "uav_control.max_velocity", 10.0f);
		acceleration_limit_ = RosUtils::getParameterOr<float>(node_, "uav_control.acceleration_limit", 2.0f);

		// Initialize agent data
		curr_pos_ = PointMsg();
		has_odom_ = false;
		goal_pos_ = PointMsg();
		has_goal_ = false;
		pos_timeout_ = update_rate_ * 1.1f; // 10% more than update rate
		last_velocity_ = Vector3r(0.0f, 0.0f, 0.0f);
		last_velocity_update_time_ = RosUtils::getTimeNow(node_);

		// Initialize UAV states
		setState(State::INITIALIZING);
		setState(State::DISARMED);
		requestArm();                   // First, arm the UAV
		requestTakeoff();               // After arming, takeoff
		// After takeoff, the controller will automatically transition to the hover state and start attempting to move to a goal

		// Subscribe to odom and goal topic
		odom_sub_ = topic_tools_->createAgentOdomSubscriber(agent_id_,
			std::bind(&UAVController::odomCallback, this, std::placeholders::_1));
		goal_sub_ = topic_tools_->createAgentPositionGoalSubscriber(agent_id_,
			std::bind(&UAVController::goalCallback, this, std::placeholders::_1));

		// Set update timer
		control_timer_ = RosUtils::createTimerByRate(node_, update_rate_,
			std::bind(&UAVController::update, this));
	}

	void UAVController::onShutdown()
	{
		// Lock mutex
		std::lock_guard<std::mutex> lock(mutex_);

		// Destroy subscribers
		odom_sub_.reset();
		goal_sub_.reset();
		// Destroy update timer
		control_timer_.reset();
		// Set state to idle
		setState(State::IDLE);
	}

	// ════════════════════════════════════════════════════════════════════════════
	// CALLBACKS: Callback functions
	// ════════════════════════════════════════════════════════════════════════════

	void UAVController::odomCallback(const core::OdometryMsg::SharedPtr msg)
	{
		// Lock mutex
		std::lock_guard<std::mutex> lock(mutex_);

		// Get current position
		curr_pos_ = msg->pose.pose.position;
		has_odom_ = true;
	}

	void UAVController::goalCallback(const core::PositionGoalMsg::SharedPtr msg)
	{
		// Lock mutex
		std::lock_guard<std::mutex> lock(mutex_);

		// Set goal position
		goal_pos_ = msg->position;
		has_goal_ = true;

		// Transition to moving
		if (state_ == State::HOVERING)
		{
			requestMove();
		}
	}

	// ════════════════════════════════════════════════════════════════════════════
	// UPDATE: Update state machine
	// ════════════════════════════════════════════════════════════════════════════

	void UAVController::update()
	{
		// Lock mutex
		std::lock_guard<std::mutex> lock(mutex_);

		// Handle state transitions and associated actions
		handleStateTransition();
	}

	// ════════════════════════════════════════════════════════════════════════════
	// PUBLIC STATE REQUESTS: Methods for requesting state transitions
	// ════════════════════════════════════════════════════════════════════════════

	bool UAVController::requestArm()
	{
		// Check if we're in DISARMED state
		if (state_ != State::DISARMED)
		{
			RCLCPP_ERROR(node_->get_logger(), "UAV controller: Cannot arm from state %d", static_cast<int>(state_));
			return false;
		}

		// Enable OFFBOARD mode
		bool result = ext_tools_->enableControl(agent_id_, true);

		// Request arming
		RCLCPP_INFO(node_->get_logger(), "UAV controller: Arming...");
		result = result && ext_tools_->armDisarm(agent_id_, true);

		if (result)
		{
			// Transition to ARMED state
			setState(State::ARMED);
			return true;
		}
		else
		{
			RCLCPP_ERROR(node_->get_logger(), "UAV controller: Arming failed");
			return false;
		}
	}

	bool UAVController::requestTakeoff()
	{
		// Check if we're in ARMED state
		if (state_ != State::ARMED)
		{
			RCLCPP_ERROR(node_->get_logger(), "UAV controller: Cannot takeoff from state %d", static_cast<int>(state_));
			return false;
		}

		// Enable OFFBOARD mode
		bool result = ext_tools_->enableControl(agent_id_, true);

		// Request takeoff
		RCLCPP_INFO(node_->get_logger(), "UAV controller: Taking off...");
		result = result && ext_tools_->takeoff(agent_id_);

		if (result)
		{
			// Transition to TAKING_OFF state
			setState(State::TAKING_OFF);
			return true;
		}
		else
		{
			RCLCPP_ERROR(node_->get_logger(), "UAV controller: Takeoff failed");
			return false;
		}
	}

	bool UAVController::requestHover()
	{
		// Check if we're in a state where hovering is permitted
		if (state_ != State::TAKING_OFF && state_ != State::MOVING && state_ != State::REACHED)
		{
			RCLCPP_ERROR(node_->get_logger(), "UAV controller: Cannot hover from state %d", static_cast<int>(state_));
			return false;
		}

		// Enable OFFBOARD mode
		bool result = ext_tools_->enableControl(agent_id_, true);

		// Request hovering
		RCLCPP_INFO(node_->get_logger(), "UAV controller: Hovering...");
		result = result && ext_tools_->hover(agent_id_);

		if (result)
		{
			// Transition to HOVERING state
			setState(State::HOVERING);
			return true;
		}
		else
		{
			RCLCPP_ERROR(node_->get_logger(), "UAV controller: Hovering failed");
			return false;
		}
	}

	bool UAVController::requestMove()
	{
		// Check if we're in a state where movement is permitted
		if (state_ != State::HOVERING && state_ != State::REACHED)
		{
			RCLCPP_ERROR(node_->get_logger(), "UAV controller: Cannot move from state %d", static_cast<int>(state_));
			return false;
		}

		// Enable OFFBOARD mode
		bool result = ext_tools_->enableControl(agent_id_, true);

		if (result)
		{
			// Transition to MOVING state
			setState(State::MOVING);
			return true;
		}
		else
		{
			RCLCPP_ERROR(node_->get_logger(), "UAV controller: Moving failed");
			return false;
		}
	}

	bool UAVController::requestLand()
	{
		// Check if we're in a state where landing is permitted
		if (state_ != State::HOVERING && state_ != State::REACHED && state_ != State::MOVING)
		{
			RCLCPP_ERROR(node_->get_logger(), "UAV controller: Cannot land from state %d", static_cast<int>(state_));
			return false;
		}

		// Enable OFFBOARD mode
		bool result = ext_tools_->enableControl(agent_id_, true);

		// Request landing
		RCLCPP_INFO(node_->get_logger(), "UAV controller: Landing...");
		result = result && ext_tools_->land(agent_id_);

		if (result)
		{
			// Transition to LANDING state
			setState(State::LANDING);
			return true;
		}
		else
		{
			RCLCPP_ERROR(node_->get_logger(), "UAV controller: Landing failed");
			return false;
		}
	}

	bool UAVController::requestDisarm()
	{
		// Check if we're in a state where we can disarm
		if (state_ != State::ARMED)
		{
			RCLCPP_ERROR(node_->get_logger(), "UAV controller: Cannot disarm from state %d", static_cast<int>(state_));
			return false;
		}

		// Enable OFFBOARD mode
		bool result = ext_tools_->enableControl(agent_id_, true);

		// Request disarming
		RCLCPP_INFO(node_->get_logger(), "UAV controller: Disarming...");
		result = result && ext_tools_->armDisarm(agent_id_, false);

		if (result)
		{
			// Transition to DISARMED state
			setState(State::DISARMED);
			return true;
		}
		else
		{
			RCLCPP_ERROR(node_->get_logger(), "UAV controller: Disarming failed");
			return false;
		}
	}

	bool UAVController::reset()
	{
		// Attempt to safely land and disarm if needed
		if (state_ != State::DISARMED && state_ != State::IDLE)
		{
			// If we're in flight, try to land
			if (state_ == State::HOVERING || state_ == State::MOVING || state_ == State::REACHED)
			{
				ext_tools_->enableControl(agent_id_, true);
				ext_tools_->land(agent_id_);
			}

			// Wait a bit for landing
			rclcpp::sleep_for(std::chrono::seconds(2));

			// Disarm
			ext_tools_->enableControl(agent_id_, true);
			ext_tools_->armDisarm(agent_id_, false);
			ext_tools_->enableControl(agent_id_, false);
		}

		// Reset state
		setState(State::IDLE);

		// Reset data
		has_odom_ = false;
		has_goal_ = false;

		// Re-initialize
		setState(State::INITIALIZING);
		return true;
	}

	// ════════════════════════════════════════════════════════════════════════════
	// STATE MANAGEMENT: State transition and validation methods
	// ════════════════════════════════════════════════════════════════════════════

	void UAVController::setState(const State& new_state)
	{
		// Check if this is a valid transition
		if (!isValidTransition(state_, new_state))
		{
			RCLCPP_ERROR(node_->get_logger(), "UAV controller: Invalid state transition from %d to %d",
				static_cast<int>(state_), static_cast<int>(new_state));
			return;
		}

		// Set state
		State old_state = state_;
		state_ = new_state;

		// Update state entry time
		state_entry_time_ = RosUtils::getTimeNow(node_);

		// Log state transition
		RCLCPP_INFO(node_->get_logger(), "UAV controller: State transition from %d to %d",
			static_cast<int>(old_state), static_cast<int>(new_state));
	}

	bool UAVController::isValidTransition(const State& from, const State& to) const
	{
		// Check state transitions
		switch (from)
		{
		case State::IDLE:
			// From IDLE, we can only go to INITIALIZING
			return to == State::INITIALIZING;

		case State::INITIALIZING:
			// From INITIALIZING, we can go to DISARMED or ERROR
			return to == State::DISARMED || to == State::ERROR;

		case State::DISARMED:
			// From DISARMED, we can go to ARMED, IDLE, or ERROR
			return to == State::ARMED || to == State::IDLE || to == State::ERROR;

		case State::ARMED:
			// From ARMED, we can go to TAKING_OFF, DISARMED, or ERROR
			return to == State::TAKING_OFF || to == State::DISARMED || to == State::ERROR;

		case State::TAKING_OFF:
			// From TAKING_OFF, we can go to HOVERING, LANDING, or ERROR
			return to == State::HOVERING || to == State::LANDING || to == State::ERROR;

		case State::HOVERING:
			// From HOVERING, we can go to MOVING, LANDING, or ERROR
			return to == State::MOVING || to == State::LANDING || to == State::ERROR;

		case State::MOVING:
			// From MOVING, we can go to REACHED, HOVERING, LANDING, or ERROR
			return to == State::REACHED || to == State::HOVERING || to == State::LANDING || to == State::ERROR;

		case State::REACHED:
			// From REACHED, we can go to HOVERING, MOVING, LANDING, or ERROR
			return to == State::HOVERING || to == State::MOVING || to == State::LANDING || to == State::ERROR;

		case State::LANDING:
			// From LANDING, we can go to DISARMED or ERROR
			return to == State::DISARMED || to == State::ERROR;

		case State::ERROR:
			// From ERROR, we can only go to IDLE (reset)
			return to == State::IDLE;

		default:
			// Unknown state, reject transition
			return false;
		}
	}

	// ════════════════════════════════════════════════════════════════════════════
	// STATE HANDLERS: Methods for handling different states
	// ════════════════════════════════════════════════════════════════════════════

	void UAVController::handleStateTransition()
	{
		// Handle state based on current state
		switch (state_)
		{
		case State::IDLE:
			handleIdleState();
			break;

		case State::INITIALIZING:
			handleInitializingState();
			break;

		case State::DISARMED:
			handleDisarmedState();
			break;

		case State::ARMED:
			handleArmedState();
			break;

		case State::TAKING_OFF:
			handleTakingOffState();
			break;

		case State::HOVERING:
			handleHoveringState();
			break;

		case State::MOVING:
			handleMovingState();
			break;

		case State::REACHED:
			handleReachedState();
			break;

		case State::LANDING:
			handleLandingState();
			break;

		case State::ERROR:
			handleErrorState();
			break;

		default:
			RCLCPP_ERROR(node_->get_logger(), "UAV controller: Unknown state: %d", static_cast<int>(state_));
			break;
		}
	}

	// State handler implementations
	void UAVController::handleIdleState()
	{
		// Nothing to do in IDLE state
	}

	void UAVController::handleInitializingState()
	{
		// Check if initialization is complete
		if ((RosUtils::getTimeNow(node_) - state_entry_time_).seconds() > 2.0)
		{
			// Move to DISARMED state
			setState(State::DISARMED);
		}
	}

	void UAVController::handleDisarmedState()
	{
		// Nothing to do in DISARMED state, waiting for arm request
	}

	void UAVController::handleArmedState()
	{
		// Nothing to do in ARMED state, waiting for takeoff request
	}

	void UAVController::handleTakingOffState()
	{
		// Check if takeoff altitude is reached
		if (has_odom_ && curr_pos_.z >= takeoff_altitude_)
		{
			RCLCPP_INFO(node_->get_logger(), "UAV controller: Takeoff complete, starting hover. Altitude: %.2f m", curr_pos_.z);
			requestHover();
		}

		// Check if takeoff has timed out
		if ((RosUtils::getTimeNow(node_) - state_entry_time_).seconds() > takeoff_timeout_)
		{
			RCLCPP_ERROR(node_->get_logger(), "UAV controller: Takeoff timeout");
			setState(State::ERROR);
			return;
		}
	}

	void UAVController::handleHoveringState()
	{
		// In hovering state, just maintain current position
		last_velocity_ = Vector3r(0.0f, 0.0f, 0.0f);
	}

	void UAVController::handleMovingState()
	{
		// Check if goal and odom are set
		if (!has_goal_ || !has_odom_)
		{
			RCLCPP_WARN(node_->get_logger(), "UAV controller: No goal or odom set in MOVING state");
			setState(State::HOVERING);
			last_velocity_ = Vector3r(0.0f, 0.0f, 0.0f);
			return;
		}

		// Convert current position and goal position to Eigen vectors
		const auto& curr_pos_vec = Vector3r(curr_pos_.x, curr_pos_.y, curr_pos_.z);
		const auto& goal_pos_vec = Vector3r(goal_pos_.x, goal_pos_.y, goal_pos_.z);

		// Check if goal is reached
		if (checkGoalReached(goal_pos_vec, curr_pos_vec))
		{
			setState(State::REACHED);
			last_velocity_ = Vector3r(0.0f, 0.0f, 0.0f);
			return;
		}

		// Compute velocity for the commanded position
		const float velocity = computeVelocity(goal_pos_vec, curr_pos_vec);

		// Send position command to external tools
		RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000.0, "UAV controller: Moving to goal (%.2f, %.2f, %.2f) with velocity %.2f m/s...",
			goal_pos_.x, goal_pos_.y, goal_pos_.z, velocity);
		ext_tools_->setPosition(agent_id_, goal_pos_.x, goal_pos_.y, goal_pos_.z, velocity, pos_timeout_);
	}

	void UAVController::handleReachedState()
	{
		// In REACHED state, hold position briefly then go to hovering
		if ((RosUtils::getTimeNow(node_) - state_entry_time_).seconds() > 2.0)
		{
			setState(State::HOVERING);
		}
	}

	void UAVController::handleLandingState()
	{
		// Check if landing has timed out
		if ((RosUtils::getTimeNow(node_) - state_entry_time_).seconds() > landing_timeout_)
		{
			RCLCPP_ERROR(node_->get_logger(), "UAV controller: Landing timeout");
			setState(State::ERROR);
			return;
		}

		// Check if we've landed (near ground level)
		if (has_odom_ && curr_pos_.z < 0.5f)
		{
			RCLCPP_INFO(node_->get_logger(), "UAV controller: Landing complete");
			// Disarm after landing
			ext_tools_->armDisarm(agent_id_, false);
			setState(State::DISARMED);
		}
	}

	void UAVController::handleErrorState()
	{
		// In ERROR state, attempt recovery
		// For safety, try to land if we're not already on the ground
		if (has_odom_ && curr_pos_.z > 0.5f)
		{
			ext_tools_->land(agent_id_);
		}
	}

	// ════════════════════════════════════════════════════════════════════════════
	// HELPER METHODS: Methods for helper functions
	// ════════════════════════════════════════════════════════════════════════════

	bool UAVController::checkGoalReached(const Vector3r& goal_pos, const Vector3r& curr_pos)
	{
		// Calculate distance to goal
		const auto& dist = (goal_pos - curr_pos).norm();

		// Check if goal is reached
		if (dist < goal_reach_threshold_)
		{
			RCLCPP_INFO(node_->get_logger(), "UAV controller: Goal reached. Distance to goal: %.2f m", dist);
			return true;
		}
		else
		{
			return false;
		}
	}

	float UAVController::computeVelocity(const Vector3r& goal_pos, const Vector3r& curr_pos)
	{
		// Get current time and calculate time difference
		auto current_time = RosUtils::getTimeNow(node_);
		float dt = (current_time - last_velocity_update_time_).seconds();
		last_velocity_update_time_ = current_time;

		// Limit dt to prevent extreme values after pauses
		dt = std::min(dt, 1.0f / update_rate_);

		// Calculate direction vector and distance to goal
		Vector3r direction = goal_pos - curr_pos;
		float distance = direction.norm();

		// Normalize direction vector if distance is not zero
		if (distance > 1e-6) {
			direction /= distance;
		}
		else {
			// We're essentially at the goal, return zero velocity
			last_velocity_ = Vector3r(0.0f, 0.0f, 0.0f);
			return 0.0f;
		}

		// Calculate deceleration distance based on current velocity
		// Using d = v²/(2*a) where v is current velocity and a is deceleration
		float current_speed = last_velocity_.norm();
		float decel_distance = (current_speed * current_speed) / (2.0f * acceleration_limit_);

		// Calculate desired speed based on trapezoidal profile
		float desired_speed;

		if (distance < decel_distance) {
			// Deceleration phase - slow down as we approach the goal
			// v = sqrt(2*a*d) where a is deceleration and d is distance to goal
			desired_speed = std::sqrt(2.0f * acceleration_limit_ * distance);
		}
		else {
		 // Acceleration or constant velocity phase
			desired_speed = max_velocity_;
		}

		// Limit acceleration and deceleration
		float speed_diff = desired_speed - current_speed;
		float max_speed_change = acceleration_limit_ * dt;

		if (speed_diff > max_speed_change) {
			// Accelerating - limit to max acceleration
			desired_speed = current_speed + max_speed_change;
		}
		else if (speed_diff < -max_speed_change) {
		 // Decelerating - limit to max deceleration
			desired_speed = current_speed - max_speed_change;
		}

		// Calculate target velocity vector
		Vector3r v_ref = direction * desired_speed;

		// Update last velocity for next iteration
		last_velocity_ = v_ref;

		// Return smoothed velocity magnitude
		return v_ref.norm();
	}


} // namespace flychams::control