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
        // Get generic solver parameters
        solver_params_.eps = RosUtils::getParameterOr<float>(node_, "agent_positioning.eps", 1.0e-1f);
        solver_params_.tol = RosUtils::getParameterOr<float>(node_, "agent_positioning.convergence_tolerance", 1.0e-5f);
        solver_params_.max_iter = RosUtils::getParameterOr<int>(node_, "agent_positioning.max_iterations", 100);
        // Get PSO parameters
        solver_params_.num_particles = RosUtils::getParameterOr<int>(node_, "agent_positioning.num_particles", 50);
        solver_params_.w_max = RosUtils::getParameterOr<float>(node_, "agent_positioning.w_max", 0.4f);
        solver_params_.w_min = RosUtils::getParameterOr<float>(node_, "agent_positioning.w_min", 0.1f);
        solver_params_.c1 = RosUtils::getParameterOr<float>(node_, "agent_positioning.c1", 1.0f);
        solver_params_.c2 = RosUtils::getParameterOr<float>(node_, "agent_positioning.c2", 1.0f);
        solver_params_.stagnation_limit = RosUtils::getParameterOr<int>(node_, "agent_positioning.stagnation_limit", 5);
        // Get ALC-PSO parameters
        solver_params_.max_lifespan = RosUtils::getParameterOr<int>(node_, "agent_positioning.max_lifespan", 60);
        solver_params_.num_challenger_tests = RosUtils::getParameterOr<int>(node_, "agent_positioning.num_challenger_tests", 10);
        // Get Nesterov parameters
        solver_params_.lipschitz_constant = RosUtils::getParameterOr<float>(node_, "agent_positioning.lipschitz_constant", 0.0f);

        // Initialize data
        agent_ = Agent();

        // Initialize setpoint message
        agent_.setpoint.header = RosUtils::createHeader(node_, transform_tools_->getGlobalFrame());
        agent_.setpoint.point = PointMsg();

        // Create and initialize solvers
        for (const auto& mode : modes_)
        {
            solvers_.push_back(createSolver(agent_id_, solver_params_, mode));
        }

        // Create subscribers for agent status, position and clusters
        agent_.status_sub = topic_tools_->createAgentStatusSubscriber(agent_id_,
            std::bind(&AgentPositioning::statusCallback, this, std::placeholders::_1), sub_options_with_module_cb_group_);
        agent_.position_sub = topic_tools_->createAgentPositionSubscriber(agent_id_,
            std::bind(&AgentPositioning::positionCallback, this, std::placeholders::_1), sub_options_with_module_cb_group_);
        agent_.clusters_sub = topic_tools_->createAgentClustersSubscriber(agent_id_,
            std::bind(&AgentPositioning::clustersCallback, this, std::placeholders::_1), sub_options_with_module_cb_group_);

        // Create publisher for agent setpoint
        agent_.setpoint_pub = topic_tools_->createAgentPositionSetpointPublisher(agent_id_);
        agent_.solver_debug_pub = node_->create_publisher<flychams_interfaces::msg::SolverDebug>(
            RosUtils::replace("coordination/AGENTID/debug/solvers", "AGENTID", agent_id_), 10);

        // Set update timer
        update_timer_ = RosUtils::createTimer(node_, update_rate_,
            std::bind(&AgentPositioning::update, this), module_cb_group_);
    }

    void AgentPositioning::onShutdown()
    {
        // Destroy solver
        for (auto& solver : solvers_)
        {
            solver->destroy();
        }
        solvers_.clear();
        // Destroy agent data
        agent_.status_sub.reset();
        agent_.position_sub.reset();
        agent_.clusters_sub.reset();
        agent_.setpoint_pub.reset();
        agent_.solver_debug_pub.reset();
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
        int n = static_cast<int>(agent_.clusters.centers.size());
        Matrix3Xr tab_P = Matrix3Xr::Zero(3, n);
        RowVectorXr tab_r = RowVectorXr::Zero(n);
        for (int i = 0; i < n; i++)
        {
            tab_P.col(i) = RosUtils::fromMsg(agent_.clusters.centers[i]);
            tab_r(i) = agent_.clusters.radii[i];
        }

        // Solve agent positioning with different solvers to compare results
        Vector3r optimal_position;
        flychams_interfaces::msg::SolverDebug solver_debug_msg;
        for (auto& solver : solvers_)
        {
            float J, t;
            Vector3r x;

            // Run solver
            const auto& start = std::chrono::high_resolution_clock::now();
            x = solver->run(tab_P, tab_r, x0, J);
            const auto& end = std::chrono::high_resolution_clock::now();
            t = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();

            // Add results to solver debug message
            switch (solver->getMode())
            {
            case PositionSolver::SolverMode::ELLIPSOID_METHOD:
            {
                solver_debug_msg.j_ellipsoid = J;
                RosUtils::toMsg(x, solver_debug_msg.x_ellipsoid);
                solver_debug_msg.t_ellipsoid = t;

                // Use ellipsoid method as optimal position for moving the agent
                optimal_position = x;
                break;
            }

            case PositionSolver::SolverMode::PSO_ALGORITHM:
            {
                solver_debug_msg.j_pso = J;
                RosUtils::toMsg(x, solver_debug_msg.x_pso);
                solver_debug_msg.t_pso = t;
                break;
            }

            case PositionSolver::SolverMode::ALC_PSO_ALGORITHM:
            {
                solver_debug_msg.j_alc_pso = J;
                RosUtils::toMsg(x, solver_debug_msg.x_alc_pso);
                solver_debug_msg.t_alc_pso = t;
                break;
            }

            case PositionSolver::SolverMode::NESTEROV_ALGORITHM:
            {
                solver_debug_msg.j_nesterov = J;
                RosUtils::toMsg(x, solver_debug_msg.x_nesterov);
                solver_debug_msg.t_nesterov = t;
                break;
            }

            case PositionSolver::SolverMode::NELDER_MEAD_NLOPT:
            {
                solver_debug_msg.j_nelder_mead = J;
                RosUtils::toMsg(x, solver_debug_msg.x_nelder_mead);
                solver_debug_msg.t_nelder_mead = t;
                break;
            }

            case PositionSolver::SolverMode::L_BFGS_NLOPT:
            {
                solver_debug_msg.j_l_bfgs = J;
                RosUtils::toMsg(x, solver_debug_msg.x_l_bfgs);
                solver_debug_msg.t_l_bfgs = t;
                break;
            }
            }
        }

        // Publish position
        agent_.setpoint.header.stamp = RosUtils::now(node_);
        RosUtils::toMsg(optimal_position, agent_.setpoint.point);
        agent_.setpoint_pub->publish(agent_.setpoint);

        // Publish solver comparison results
        solver_debug_msg.header.stamp = RosUtils::now(node_);
        agent_.solver_debug_pub->publish(solver_debug_msg);
    }

    // ════════════════════════════════════════════════════════════════════════════
    // POSITIONING: Positioning methods
    // ════════════════════════════════════════════════════════════════════════════

    PositionSolver::SharedPtr AgentPositioning::createSolver(const std::string& agent_id, const PositionSolver::Parameters& solver_params, const PositionSolver::SolverMode& solver_mode)
    {
        // Create solver instance
        PositionSolver::SharedPtr solver = std::make_shared<PositionSolver>();

        // Get config
        const auto& config_ptr = config_tools_->getConfig();
        const auto& agent_ptr = config_tools_->getAgent(agent_id);
        const auto& tracking_params = config_tools_->getTrackingParameters(agent_id);

        // Get cost parameters for each tracking unit
        CostFunctions::Parameters cost_params;
        cost_params.units = createUnitParameters(tracking_params);
        cost_params.n = static_cast<int>(cost_params.units.size());
        cost_params.n_tracking = static_cast<int>(cost_params.units.size()) - 1;

        // Get space constraints
        float min_horizontal = config_ptr->horizontal_constraint(0);
        float max_horizontal = config_ptr->horizontal_constraint(1);
        float min_vertical = config_ptr->vertical_constraint(0);
        float max_vertical = std::min(config_ptr->vertical_constraint(1), agent_ptr->max_altitude);
        Vector3r x_min = Vector3r(min_horizontal, min_horizontal, min_vertical);
        Vector3r x_max = Vector3r(max_horizontal, max_horizontal, max_vertical);

        // Create solver parameters
        PositionSolver::Parameters params = solver_params;
        params.cost_params = cost_params;
        params.x_min = x_min;
        params.x_max = x_max;

        // Initialize solver
        solver->init(solver_mode, params);

        return solver;
    }

    std::vector<CostFunctions::TrackingUnit> AgentPositioning::createUnitParameters(const TrackingParameters& tracking_params)
    {
        std::vector<CostFunctions::TrackingUnit> params_vector;

        // Get tracking mode
        const auto& mode = tracking_params.mode;

        // Get unit parameters depending on tracking mode
        switch (mode)
        {
        case TrackingMode::MultiCamera:
        {
            // Iterate through heads
            for (const auto& head : tracking_params.head_params)
            {
                CostFunctions::TrackingUnit params;

                // Set tracking mode
                params.mode = mode;

                // Camera parameters
                params.f_min = head.f_min;
                params.f_max = head.f_max;
                params.f_ref = head.f_ref;
                params.s_min = head.s_min;
                params.s_max = head.s_max;
                params.s_ref = head.s_ref;

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
            break;
        }

        case TrackingMode::MultiWindow:
        {
            // Get central head
            const auto& central_head = tracking_params.head_params[0];

            // Iterate through windows
            for (const auto& window : tracking_params.window_params)
            {
                CostFunctions::TrackingUnit params;

                // Set tracking mode
                params.mode = mode;

                // Window parameters
                params.central_f = central_head.f_ref;
                params.lambda_min = window.lambda_min;
                params.lambda_max = window.lambda_max;
                params.lambda_ref = window.lambda_ref;
                params.s_min = window.s_min;
                params.s_max = window.s_max;
                params.s_ref = window.s_ref;

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
            break;
        }

        default:
            RCLCPP_ERROR(node_->get_logger(), "Agent positioning: Invalid tracking mode");
            break;
        }

        return params_vector;
    }

} // namespace flychams::coordination
