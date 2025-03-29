#include "flychams_control/drone/drone_state.hpp"

using namespace flychams::core;

namespace flychams::control
{
    // ════════════════════════════════════════════════════════════════════════════
    // CONSTRUCTOR: Constructor and destructor
    // ════════════════════════════════════════════════════════════════════════════

    void DroneState::onInit()
    {
        // Get parameters from parameter server
        // Get update rate
        update_rate_ = RosUtils::getParameterOr<float>(node_, "drone_state.control_update_rate", 5.0f);
        // Get takeoff parameters
        takeoff_altitude_ = RosUtils::getParameterOr<float>(node_, "drone_state.takeoff_altitude", 1.5f);
        takeoff_timeout_ = RosUtils::getParameterOr<float>(node_, "drone_state.takeoff_timeout", 10.0f);
        // Get landing parameters
        landing_altitude_ = RosUtils::getParameterOr<float>(node_, "drone_state.landing_altitude", 0.5f);
        landing_timeout_ = RosUtils::getParameterOr<float>(node_, "drone_state.landing_timeout", 10.0f);
        // Get hover parameters
        hover_altitude_ = RosUtils::getParameterOr<float>(node_, "drone_state.hover_altitude", 1.0f);
        hover_timeout_ = RosUtils::getParameterOr<float>(node_, "drone_state.hover_timeout", 5.0f);

        // Compute command timeout
        cmd_timeout_ = (1.0f / update_rate_) * 1.25f;

        // Set initial status as disarmed. Arming request will be necessary for further transitions
        curr_status_ = AgentStatus::DISARMED;

        // Initialize drone position
        curr_position_ = PointMsg();
        has_position_ = false;

        // Initialize messages
        status_msg_ = AgentStatusMsg();
        status_msg_.header = RosUtils::createHeader(node_, transform_tools_->getGlobalFrame());
        position_msg_ = PointStampedMsg();
        position_msg_.header = RosUtils::createHeader(node_, transform_tools_->getGlobalFrame());

        // Subscribe to agent odom topic from framework tools
        odom_sub_ = framework_tools_->createOdometrySubscriber(agent_id_,
            std::bind(&DroneState::odomCallback, this, std::placeholders::_1), sub_options_with_module_cb_group_);

        // Create publisher for agent status and position
        status_pub_ = topic_tools_->createAgentStatusPublisher(agent_id_);
        position_pub_ = topic_tools_->createAgentPositionPublisher(agent_id_);

        // Set update timer
        last_update_time_ = RosUtils::now(node_);
        status_duration_ = 0.0f;
        update_timer_ = RosUtils::createTimer(node_, update_rate_,
            std::bind(&DroneState::update, this), module_cb_group_);
    }

    void DroneState::onShutdown()
    {
        // Set status to idle
        curr_status_ = AgentStatus::IDLE;
        // Destroy subscriber
        odom_sub_.reset();
        // Destroy publisher
        status_pub_.reset();
        position_pub_.reset();
        // Destroy update timer
        update_timer_.reset();
    }

    // ════════════════════════════════════════════════════════════════════════════
    // PUBLIC METHODS: Methods for initializing and handling the state manager
    // ════════════════════════════════════════════════════════════════════════════

    bool DroneState::requestDisarm()
    {
        // Check if we're in ARMED or LANDING state
        if (curr_status_ != AgentStatus::ARMED && curr_status_ != AgentStatus::LANDING)
        {
            RCLCPP_WARN(node_->get_logger(), "Drone state: Cannot disarm agent %s from state %d", agent_id_.c_str(), static_cast<int>(curr_status_));
            return false;
        }

        // Enable OFFBOARD mode
        bool enable_result = framework_tools_->enableControl(agent_id_, true);
        if (!enable_result)
        {
            RCLCPP_ERROR(node_->get_logger(), "Drone state: Failed to enable control for agent %s", agent_id_.c_str());
            return false;
        }

        // Request disarming
        RCLCPP_INFO(node_->get_logger(), "Drone state: Disarming agent %s...", agent_id_.c_str());
        bool disarm_result = framework_tools_->armDisarm(agent_id_, false);
        if (!disarm_result)
        {
            RCLCPP_ERROR(node_->get_logger(), "Drone state: Failed to disarm agent %s", agent_id_.c_str());
            return false;
        }

        // Transition to DISARMED state
        setStatus(AgentStatus::DISARMED);
        return true;
    }

    bool DroneState::requestArm()
    {
        // Check if we're in DISARMED state
        if (curr_status_ != AgentStatus::DISARMED)
        {
            RCLCPP_WARN(node_->get_logger(), "Drone state: Cannot arm agent %s from state %d", agent_id_.c_str(), static_cast<int>(curr_status_));
            return false;
        }

        // Enable OFFBOARD mode
        bool enable_result = framework_tools_->enableControl(agent_id_, true);
        if (!enable_result)
        {
            RCLCPP_ERROR(node_->get_logger(), "Drone state: Failed to enable control for agent %s", agent_id_.c_str());
            return false;
        }

        // Request arming
        RCLCPP_INFO(node_->get_logger(), "Drone state: Arming agent %s...", agent_id_.c_str());
        bool arm_result = framework_tools_->armDisarm(agent_id_, true);
        if (!arm_result)
        {
            RCLCPP_ERROR(node_->get_logger(), "Drone state: Failed to arm agent %s", agent_id_.c_str());
            return false;
        }

        // Transition to ARMED state
        setStatus(AgentStatus::ARMED);
        return true;
    }

    bool DroneState::requestTakeoff()
    {
        // Check if we're in ARMED state
        if (curr_status_ != AgentStatus::ARMED)
        {
            RCLCPP_WARN(node_->get_logger(), "Drone state: Cannot takeoff agent %s from state %d", agent_id_.c_str(), static_cast<int>(curr_status_));
            return false;
        }

        // Enable OFFBOARD mode
        bool enable_result = framework_tools_->enableControl(agent_id_, true);
        if (!enable_result)
        {
            RCLCPP_ERROR(node_->get_logger(), "Drone state: Failed to enable control for agent %s", agent_id_.c_str());
            return false;
        }

        // Request takeoff
        RCLCPP_INFO(node_->get_logger(), "Drone state: Taking off agent %s...", agent_id_.c_str());
        bool takeoff_result = framework_tools_->takeoff(agent_id_);
        if (!takeoff_result)
        {
            RCLCPP_ERROR(node_->get_logger(), "Drone state: Failed to takeoff agent %s", agent_id_.c_str());
            return false;
        }

        // Transition to TAKING_OFF state
        setStatus(AgentStatus::TAKING_OFF);
        return true;
    }

    bool DroneState::requestHover()
    {
        // Check if we're in TAKEN_OFF or TRACKING state
        if (curr_status_ != AgentStatus::TAKEN_OFF && curr_status_ != AgentStatus::TRACKING)
        {
            RCLCPP_WARN(node_->get_logger(), "Drone state: Cannot hover agent %s from state %d", agent_id_.c_str(), static_cast<int>(curr_status_));
            return false;
        }

        // Enable OFFBOARD mode
        bool enable_result = framework_tools_->enableControl(agent_id_, true);
        if (!enable_result)
        {
            RCLCPP_ERROR(node_->get_logger(), "Drone state: Failed to enable control for agent %s", agent_id_.c_str());
            return false;
        }

        // Request hover
        RCLCPP_INFO(node_->get_logger(), "Drone state: Hovering agent %s...", agent_id_.c_str());
        bool hover_result = framework_tools_->hover(agent_id_);
        if (!hover_result)
        {
            RCLCPP_ERROR(node_->get_logger(), "Drone state: Failed to hover agent %s", agent_id_.c_str());
            return false;
        }

        // Transition to HOVERING state
        setStatus(AgentStatus::HOVERING);
        return true;
    }

    bool DroneState::requestTracking()
    {
        // Check if we're in HOVERED state
        if (curr_status_ != AgentStatus::HOVERED)
        {
            RCLCPP_WARN(node_->get_logger(), "Drone state: Cannot move agent %s from state %d", agent_id_.c_str(), static_cast<int>(curr_status_));
            return false;
        }

        // Enable OFFBOARD mode
        bool enable_result = framework_tools_->enableControl(agent_id_, true);
        if (!enable_result)
        {
            RCLCPP_ERROR(node_->get_logger(), "Drone state: Failed to enable control for agent %s", agent_id_.c_str());
            return false;
        }

        // Request move
        RCLCPP_INFO(node_->get_logger(), "Drone state: Agent %s moving to goal...", agent_id_.c_str());

        // Transition to TRACKING state
        setStatus(AgentStatus::TRACKING);
        return true;
    }

    bool DroneState::requestLand()
    {
        // Check if we're in HOVERING, HOVERED or TRACKING state
        if (curr_status_ != AgentStatus::HOVERING && curr_status_ != AgentStatus::HOVERED && curr_status_ != AgentStatus::TRACKING)
        {
            RCLCPP_WARN(node_->get_logger(), "Drone state: Cannot land agent %s from state %d", agent_id_.c_str(), static_cast<int>(curr_status_));
            return false;
        }

        // Enable OFFBOARD mode
        bool enable_result = framework_tools_->enableControl(agent_id_, true);
        if (!enable_result)
        {
            RCLCPP_ERROR(node_->get_logger(), "Drone state: Failed to enable control for agent %s", agent_id_.c_str());
            return false;
        }

        // Request landing
        RCLCPP_INFO(node_->get_logger(), "Drone state: Landing agent %s...", agent_id_.c_str());
        bool land_result = framework_tools_->land(agent_id_);
        if (!land_result)
        {
            RCLCPP_ERROR(node_->get_logger(), "Drone state: Failed to land agent %s", agent_id_.c_str());
            return false;
        }

        // Transition to LANDING state
        setStatus(AgentStatus::LANDING);
        return true;
    }

    // ════════════════════════════════════════════════════════════════════════════
    // CALLBACKS: Callback functions
    // ════════════════════════════════════════════════════════════════════════════

    void DroneState::odomCallback(const core::OdometryMsg::SharedPtr msg)
    {
        // Update current position
        curr_position_ = msg->pose.pose.position;
        has_position_ = true;

        // Publish agent position
        position_msg_.header.stamp = RosUtils::now(node_);
        position_msg_.point = curr_position_;
        position_pub_->publish(position_msg_);
    }

    // ════════════════════════════════════════════════════════════════════════════
    // STATE MANAGEMENT: State transition and validation methods
    // ════════════════════════════════════════════════════════════════════════════

    void DroneState::update()
    {
        // Check if we have a valid position. If not, we can't update the state
        if (!has_position_)
        {
            RCLCPP_WARN(node_->get_logger(), "Drone state: No position data received for agent %s", agent_id_.c_str());
            return;
        }

        // Compute time step
        auto current_time = RosUtils::now(node_);
        float dt = (current_time - last_update_time_).seconds();
        last_update_time_ = current_time;

        // Limit dt to prevent extreme values after pauses
        dt = std::min(dt, cmd_timeout_);

        // Update state duration
        status_duration_ += dt;

        // Handle state based on current state
        switch (curr_status_)
        {
        case AgentStatus::IDLE:
            handleIdle();
            break;

        case AgentStatus::DISARMED:
            handleDisarmed();
            break;

        case AgentStatus::ARMED:
            handleArmed();
            break;

        case AgentStatus::TAKING_OFF:
            handleTakingOff();
            break;

        case AgentStatus::TAKEN_OFF:
            handleTakenOff();
            break;

        case AgentStatus::HOVERING:
            handleHovering();
            break;

        case AgentStatus::HOVERED:
            handleHovered();
            break;

        case AgentStatus::TRACKING:
            handleTracking();
            break;

        case AgentStatus::LANDING:
            handleLanding();
            break;

        case AgentStatus::LANDED:
            handleLanded();
            break;

        case AgentStatus::ERROR:
            handleError();
            break;

        default:
            RCLCPP_ERROR(node_->get_logger(), "Drone state: Unknown state for agent %s: %d", agent_id_.c_str(), static_cast<int>(curr_status_));
            break;
        }

        // Publish agent status
        status_msg_.header.stamp = RosUtils::now(node_);
        status_msg_.status = static_cast<uint8_t>(curr_status_);
        status_pub_->publish(status_msg_);
    }

    void DroneState::setStatus(const AgentStatus& new_status)
    {
        // Check if this is a valid transition
        if (!isValid(curr_status_, new_status))
        {
            RCLCPP_WARN(node_->get_logger(), "Drone state: Invalid state transition from %d to %d for agent %s",
                static_cast<int>(curr_status_), static_cast<int>(new_status), agent_id_.c_str());
            return;
        }

        // Set new state
        AgentStatus old_status = curr_status_;
        curr_status_ = new_status;

        // Update state duration
        status_duration_ = 0.0f;

        // Log state transition
        RCLCPP_INFO(node_->get_logger(), "Drone state: AgentStatus transition from %d to %d for agent %s",
            static_cast<int>(old_status), static_cast<int>(new_status), agent_id_.c_str());
    }

    bool DroneState::isValid(const AgentStatus& from, const AgentStatus& to) const
    {
        // Check state transitions
        switch (from)
        {
        case AgentStatus::IDLE:
            // From IDLE, we can only go to DISARMED
            return to == AgentStatus::DISARMED;

        case AgentStatus::DISARMED:
            // From DISARMED, we can go to ARMED, IDLE, or ERROR
            return to == AgentStatus::ARMED || to == AgentStatus::IDLE || to == AgentStatus::ERROR;

        case AgentStatus::ARMED:
            // From ARMED, we can go to DISARMED, TAKING_OFF, or ERROR
            return to == AgentStatus::DISARMED || to == AgentStatus::TAKING_OFF || to == AgentStatus::ERROR;

        case AgentStatus::TAKING_OFF:
            // From TAKING_OFF, we can go to TAKEN_OFF, LANDING, or ERROR
            return to == AgentStatus::TAKEN_OFF || to == AgentStatus::LANDING || to == AgentStatus::ERROR;

        case AgentStatus::TAKEN_OFF:
            // From TAKEN_OFF, we can go to HOVERING, LANDING, or ERROR
            return to == AgentStatus::HOVERING || to == AgentStatus::LANDING || to == AgentStatus::ERROR;

        case AgentStatus::HOVERING:
            // From HOVERING, we can go to HOVERED, LANDING, or ERROR
            return to == AgentStatus::HOVERED || to == AgentStatus::LANDING || to == AgentStatus::ERROR;

        case AgentStatus::HOVERED:
            // From HOVERED, we can go to TRACKING, LANDING, or ERROR
            return to == AgentStatus::TRACKING || to == AgentStatus::LANDING || to == AgentStatus::ERROR;

        case AgentStatus::TRACKING:
            // From TRACKING, we can go to HOVERING, LANDING, or ERROR
            return to == AgentStatus::HOVERING || to == AgentStatus::LANDING || to == AgentStatus::ERROR;

        case AgentStatus::LANDING:
            // From LANDING, we can go to LANDED, or ERROR
            return to == AgentStatus::LANDED || to == AgentStatus::ERROR;

        case AgentStatus::LANDED:
            // From LANDED, we can go to DISARMED, or ERROR
            return to == AgentStatus::DISARMED || to == AgentStatus::ERROR;

        case AgentStatus::ERROR:
            // From ERROR, we can only go to IDLE (reset)
            return to == AgentStatus::IDLE;

        default:
            // Unknown state, reject transition
            return false;
        }
    }

    // ════════════════════════════════════════════════════════════════════════════
    // STATE HANDLERS: Methods for handling different states
    // ════════════════════════════════════════════════════════════════════════════

    // DroneState handler implementations
    void DroneState::handleIdle()
    {
        // Nothing to do in IDLE state
    }

    void DroneState::handleDisarmed()
    {
        // Nothing to do in DISARMED state, waiting for arm request
    }

    void DroneState::handleArmed()
    {
        // Nothing to do in ARMED state, waiting for takeoff request
    }

    void DroneState::handleTakingOff()
    {
        // Check if takeoff has timed out
        if (status_duration_ > takeoff_timeout_)
        {
            RCLCPP_WARN(node_->get_logger(), "Drone state: Takeoff timeout for agent %s, trying again...", agent_id_.c_str());
            requestTakeoff();
            return;
        }

        // Check if takeoff altitude is reached
        if (curr_position_.z >= takeoff_altitude_)
        {
            RCLCPP_INFO(node_->get_logger(), "Drone state: Takeoff complete for agent %s", agent_id_.c_str());
            setStatus(AgentStatus::TAKEN_OFF);
        }
    }

    void DroneState::handleTakenOff()
    {
        // Nothing to do in TAKEN_OFF state, waiting for hover request
    }

    void DroneState::handleHovering()
    {
        // Check if hover has timed out
        if (status_duration_ > hover_timeout_)
        {
            RCLCPP_WARN(node_->get_logger(), "Drone state: Hover timeout for agent %s, trying again...", agent_id_.c_str());
            requestHover();
            return;
        }

        // Check if hover altitude is reached
        if (curr_position_.z >= hover_altitude_)
        {
            RCLCPP_INFO(node_->get_logger(), "Drone state: Hover complete for agent %s", agent_id_.c_str());
            setStatus(AgentStatus::HOVERED);
        }
    }

    void DroneState::handleHovered()
    {
        // Nothing to do in HOVERED state, waiting for move request
    }

    void DroneState::handleTracking()
    {
        // In TRACKING state, other nodes handle the commands (such as drone motion)
    }

    void DroneState::handleLanding()
    {
        // Check if landing has timed out
        if (status_duration_ > landing_timeout_)
        {
            RCLCPP_WARN(node_->get_logger(), "Drone state: Landing timeout for agent %s, trying again...", agent_id_.c_str());
            requestLand();
            return;
        }

        // Check if landing altitude is reached
        if (curr_position_.z <= landing_altitude_)
        {
            RCLCPP_INFO(node_->get_logger(), "Drone state: Landing complete for agent %s", agent_id_.c_str());
            setStatus(AgentStatus::LANDED);
        }
    }

    void DroneState::handleLanded()
    {
        // Nothing to do in LANDED state, waiting for disarm request
    }

    void DroneState::handleError()
    {
        // In ERROR state, attempt landing
        requestLand();
    }

} // namespace flychams::control