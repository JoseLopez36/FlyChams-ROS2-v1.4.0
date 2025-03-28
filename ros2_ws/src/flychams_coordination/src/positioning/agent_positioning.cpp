#include "flychams_coordination/positioning/agent_positioning.hpp"

using namespace flychams::core;

namespace flychams::coordination
{
    // ════════════════════════════════════════════════════════════════════════════
    // CONSTRUCTOR: Constructor and destructor
    // ════════════════════════════════════════════════════════════════════════════

    void AgentPositioning::onInit()
    {
        // Get parameters from parameter server
        // Get update rate
        update_rate_ = RosUtils::getParameterOr<float>(node_, "agent_positioning.positioning_rate", 1.0f);
        // Get solver parameters
        PositionSolver::SolverMode solver_mode = static_cast<PositionSolver::SolverMode>(RosUtils::getParameterOr<uint8_t>(node_, "agent_positioning.solver_mode", 0));
        float convergence_tolerance = RosUtils::getParameterOr<float>(node_, "agent_positioning.convergence_tolerance", 1.0e-6f);
        int max_iterations = RosUtils::getParameterOr<int>(node_, "agent_positioning.max_iterations", 100);
        float eps = RosUtils::getParameterOr<float>(node_, "agent_positioning.eps", 1.0f);

        // Initialize data
        agent_ = Agent();

        // Initialize setpoint message
        agent_.setpoint.header = RosUtils::createHeader(node_, transform_tools_->getGlobalFrame());
        agent_.setpoint.point.x = 0.0;
        agent_.setpoint.point.y = 0.0;
        agent_.setpoint.point.z = 0.0;

        // Initialize solver
        solver_.reset();
        solver_.setMode(solver_mode);
        solver_.setParameters(convergence_tolerance, max_iterations, eps);
        solver_.init();

        // Get positioning parameters
        const auto& config_ptr = config_tools_->getConfig();
        const auto& agent_ptr = config_tools_->getAgent(agent_id_);
        min_height_ = config_ptr->altitude_constraint(0);
        max_height_ = std::min(config_ptr->altitude_constraint(1), agent_ptr->max_altitude);
        tracking_params_ = config_tools_->getTrackingParameters(agent_id_);

        // Create subscribers for agent status, position and clusters
        agent_.status_sub = topic_tools_->createAgentStatusSubscriber(agent_id_,
            std::bind(&AgentPositioning::statusCallback, this, std::placeholders::_1), sub_options_with_module_cb_group_);
        agent_.position_sub = topic_tools_->createAgentPositionSubscriber(agent_id_,
            std::bind(&AgentPositioning::positionCallback, this, std::placeholders::_1), sub_options_with_module_cb_group_);
        agent_.clusters_sub = topic_tools_->createAgentClustersSubscriber(agent_id_,
            std::bind(&AgentPositioning::clustersCallback, this, std::placeholders::_1), sub_options_with_module_cb_group_);

        // Create publisher for agent setpoint
        agent_.setpoint_pub = topic_tools_->createAgentPositionSetpointPublisher(agent_id_);

        // Set update timer
        update_timer_ = RosUtils::createTimer(node_, update_rate_,
            std::bind(&AgentPositioning::update, this), module_cb_group_);
    }

    void AgentPositioning::onShutdown()
    {
        // Destroy solver
        solver_.destroy();
        // Destroy agent data
        agent_.status_sub.reset();
        agent_.position_sub.reset();
        agent_.clusters_sub.reset();
        agent_.setpoint_pub.reset();
        // Destroy update timer
        update_timer_.reset();
    }

    // ════════════════════════════════════════════════════════════════════════════
    // CALLBACKS: Callback functions
    // ════════════════════════════════════════════════════════════════════════════

    void AgentPositioning::statusCallback(const AgentStatusMsg::SharedPtr msg)
    {
        // Update agent status
        agent_.status = static_cast<AgentStatus>(msg->status);
        agent_.has_status = true;
    }

    void AgentPositioning::positionCallback(const PointStampedMsg::SharedPtr msg)
    {
        // Update agent position
        agent_.position = msg->point;
        agent_.has_position = true;
    }

    void AgentPositioning::clustersCallback(const AgentClustersMsg::SharedPtr msg)
    {
        // Update agent clusters
        agent_.clusters = *msg;
        agent_.has_clusters = true;
    }

    // ════════════════════════════════════════════════════════════════════════════
    // UPDATE: Update positioning
    // ════════════════════════════════════════════════════════════════════════════

    void AgentPositioning::update()
    {
        // Check if we have a valid agent status, position and cluster assignments
        if (!agent_.has_status || !agent_.has_position || !agent_.has_clusters)
        {
            RCLCPP_WARN(node_->get_logger(), "Agent positioning: Agent %s has no status, position or clusters", agent_id_.c_str());
            return; // Skip positioning if we don't have a valid agent status, position or clusters
        }

        // Check if we are in the correct state to position
        if (agent_.status != AgentStatus::TRACKING)
        {
            RCLCPP_WARN(node_->get_logger(), "Agent positioning: Agent %s is not in the correct state to position",
                agent_id_.c_str());
            return;
        }

        // Convert messages to Eigen types
        Vector3r x0 = RosUtils::fromMsg(agent_.position);
        int n = static_cast<int>(agent_.clusters.cluster_ids.size());
        Matrix3Xr tab_P = Matrix3Xr::Zero(3, n);
        RowVectorXr tab_r = RowVectorXr::Zero(n);
        for (size_t i = 0; i < n; i++)
        {
            tab_P.col(i) = RosUtils::fromMsg(agent_.clusters.centers[i]);
            tab_r(i) = agent_.clusters.radii[i];
        }

        // Solve agent positioning
        Vector3r optimal_position = solver_.run(tab_P, tab_r, x0, min_height_, max_height_, tracking_params_);

        // Publish position
        agent_.setpoint.header.stamp = RosUtils::now(node_);
        RosUtils::toMsg(optimal_position, agent_.setpoint.point);
        agent_.setpoint_pub->publish(agent_.setpoint);
    }

} // namespace flychams::coordination