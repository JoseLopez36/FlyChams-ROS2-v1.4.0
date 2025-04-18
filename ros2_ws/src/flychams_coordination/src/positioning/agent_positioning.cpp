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
        float eps = RosUtils::getParameterOr<float>(node_, "agent_positioning.eps", 1.0e-1f);
        float convergence_tolerance = RosUtils::getParameterOr<float>(node_, "agent_positioning.convergence_tolerance", 1.0e-5f);
        int max_iterations = RosUtils::getParameterOr<int>(node_, "agent_positioning.max_iterations", 100);
        
        // Initialize data
        agent_ = Agent();

        // Initialize setpoint message
        agent_.setpoint.header = RosUtils::createHeader(node_, transform_tools_->getGlobalFrame());
        agent_.setpoint.point.x = 0.0;
        agent_.setpoint.point.y = 0.0;
        agent_.setpoint.point.z = 0.0;

        // Get positioning parameters
        const auto& config_ptr = config_tools_->getConfig();
        const auto& agent_ptr = config_tools_->getAgent(agent_id_);
        const auto& tracking_params = config_tools_->getTrackingParameters(agent_id_);
        float min_horizontal = config_ptr->horizontal_constraint(0);
        float max_horizontal = config_ptr->horizontal_constraint(1);
        float min_vertical = config_ptr->vertical_constraint(0);
        float max_vertical = std::min(config_ptr->vertical_constraint(1), agent_ptr->max_altitude);

        // Calculate space constraints
        Vector3r x_min = Vector3r(min_horizontal, min_horizontal, min_vertical);
        Vector3r x_max = Vector3r(max_horizontal, max_horizontal, max_vertical);

        // Create cost parameters for each tracking unit
        CostFunctions::Parameters cost_params;
        cost_params.n = tracking_params.n;
        cost_params.central = centralUnitParameters(tracking_params);
        cost_params.tracking = trackingUnitParameters(tracking_params);

        // Create and initialize solver
        solver_ = std::make_shared<PositionSolver>();
        PositionSolver::Parameters solver_params;
        solver_params.cost_params = cost_params;
        solver_params.x_min = x_min;
        solver_params.x_max = x_max;
        solver_params.eps = eps;
        solver_params.tol = convergence_tolerance;
        solver_params.max_iter = max_iterations;
        solver_->init(solver_mode, solver_params);

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
        solver_->destroy();
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
        float J;
        Vector3r optimal_position = solver_->run(tab_P, tab_r, x0, J);
        
        // Publish position
        agent_.setpoint.header.stamp = RosUtils::now(node_);
        RosUtils::toMsg(optimal_position, agent_.setpoint.point);
        agent_.setpoint_pub->publish(agent_.setpoint);
    }

    // ════════════════════════════════════════════════════════════════════════════
    // POSITIONING: Positioning methods
    // ════════════════════════════════════════════════════════════════════════════

    CostFunctions::TrackingUnit AgentPositioning::centralUnitParameters(const TrackingParameters& tracking_params)
    {
        CostFunctions::TrackingUnit params;

        // Tracking mode
        params.mode = tracking_params.mode;

        // Camera parameters
        params.f_min = tracking_params.central_head_params.f_min;
        params.f_max = tracking_params.central_head_params.f_max;
        params.f_ref = tracking_params.central_head_params.f_ref;

        // Window parameters
        params.lambda_min = tracking_params.central_window_params.lambda_min;
        params.lambda_max = tracking_params.central_window_params.lambda_max;
        params.lambda_ref = tracking_params.central_window_params.lambda_ref;
        params.central_f = tracking_params.central_head_params.f_ref;

        // Projection parameters
        params.s_min = tracking_params.central_window_params.s_min;
        params.s_max = tracking_params.central_window_params.s_max;
        params.s_ref = tracking_params.central_window_params.s_ref;

        // Cost function weights
        // Psi
        params.tau0 = 1.0f;
        params.tau1 = 2.0f;
        params.tau2 = 10.0f;
        // Lambda
        params.sigma0 = 1.0f;
        params.sigma1 = 2.0f;
        params.sigma2 = 10.0f;
        // Gamma
        params.mu = 1.0f;
        params.nu = 1.0f;

        return params;
    }

    std::vector<CostFunctions::TrackingUnit> AgentPositioning::trackingUnitParameters(const TrackingParameters& tracking_params)
    {
        std::vector<CostFunctions::TrackingUnit> params_vector;
        for (size_t i = 0; i < tracking_params.n; i++)
        {
            CostFunctions::TrackingUnit params;

            // Tracking mode
            params.mode = tracking_params.mode;

            // Camera parameters
            if (params.mode == TrackingMode::MultiCamera)
            {
                params.f_min = tracking_params.tracking_head_params[i].f_min;
                params.f_max = tracking_params.tracking_head_params[i].f_max;
                params.f_ref = tracking_params.tracking_head_params[i].f_ref;
                params.s_min = tracking_params.tracking_head_params[i].s_min;
                params.s_max = tracking_params.tracking_head_params[i].s_max;
                params.s_ref = tracking_params.tracking_head_params[i].s_ref;
            }

            // Window parameters
            if (params.mode == TrackingMode::MultiWindow)
            {
                params.central_f = tracking_params.central_head_params.f_ref;
                params.lambda_min = tracking_params.tracking_window_params[i].lambda_min;
                params.lambda_max = tracking_params.tracking_window_params[i].lambda_max;
                params.lambda_ref = tracking_params.tracking_window_params[i].lambda_ref;
                params.s_min = tracking_params.tracking_window_params[i].s_min;
                params.s_max = tracking_params.tracking_window_params[i].s_max;
                params.s_ref = tracking_params.tracking_window_params[i].s_ref;
            }

            // Cost function weights
            // Psi
            params.tau0 = 1.0f;
            params.tau1 = 2.0f;
            params.tau2 = 10.0f;
            // Lambda
            params.sigma0 = 1.0f;
            params.sigma1 = 2.0f;
            params.sigma2 = 10.0f;
            // Gamma
            params.mu = 1.0f;
            params.nu = 1.0f;

            params_vector.push_back(params);
        }

        return params_vector;
    }

} // namespace flychams::coordination