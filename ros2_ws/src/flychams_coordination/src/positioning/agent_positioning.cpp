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
        //PositionSolver::SolverMode solver_mode = static_cast<PositionSolver::SolverMode>(RosUtils::getParameterOr<uint8_t>(node_, "agent_positioning.solver_mode", 0));
        // Get generic solver parameters
        float eps = RosUtils::getParameterOr<float>(node_, "agent_positioning.eps", 1.0e-1f);
        float convergence_tolerance = RosUtils::getParameterOr<float>(node_, "agent_positioning.convergence_tolerance", 1.0e-5f);
        int max_iterations = RosUtils::getParameterOr<int>(node_, "agent_positioning.max_iterations", 100);
        // Get PSO parameters
        int num_particles = RosUtils::getParameterOr<int>(node_, "agent_positioning.num_particles", 50);
        float w_max = RosUtils::getParameterOr<float>(node_, "agent_positioning.w_max", 0.4f);
        float w_min = RosUtils::getParameterOr<float>(node_, "agent_positioning.w_min", 0.1f);
        float c1 = RosUtils::getParameterOr<float>(node_, "agent_positioning.c1", 1.0f);
        float c2 = RosUtils::getParameterOr<float>(node_, "agent_positioning.c2", 1.0f);
        int stagnation_limit = RosUtils::getParameterOr<int>(node_, "agent_positioning.stagnation_limit", 5);
        // Get ALC-PSO parameters
        int max_lifespan = RosUtils::getParameterOr<int>(node_, "agent_positioning.max_lifespan", 60);
        int num_challenger_tests = RosUtils::getParameterOr<int>(node_, "agent_positioning.num_challenger_tests", 10);
        // Get Nesterov parameters
        float lipschitz_constant = RosUtils::getParameterOr<float>(node_, "agent_positioning.lipschitz_constant", 0.0f);
        
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

        // Create solver parameters
        PositionSolver::Parameters solver_params;
        solver_params.cost_params = cost_params;
        solver_params.x_min = x_min;
        solver_params.x_max = x_max;
        // Generic solver parameters
        solver_params.eps = eps;
        solver_params.tol = convergence_tolerance;
        solver_params.max_iter = max_iterations;
        // PSO parameters
        solver_params.num_particles = num_particles;
        solver_params.w_max = w_max;
        solver_params.w_min = w_min;
        solver_params.c1 = c1;
        solver_params.c2 = c2;
        solver_params.stagnation_limit = stagnation_limit;
        // ALC-PSO parameters
        solver_params.max_lifespan = max_lifespan;
        solver_params.num_challenger_tests = num_challenger_tests;
        // Nesterov parameters
        solver_params.lipschitz_constant = lipschitz_constant;

        // Initialize solver array with several modes
        for (const auto& mode : modes_)
        {
            PositionSolver::SharedPtr solver = std::make_shared<PositionSolver>();
            solver->init(mode, solver_params);
            solvers_.push_back(solver);
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
        for (size_t i = 0; i < n; i++)
        {
            tab_P.col(i) = RosUtils::fromMsg(agent_.clusters.centers[i]);
            tab_r(i) = agent_.clusters.radii[i];
        }

        // Solve agent positioning with different solvers to compare results
        float J_ellipsoid, J_pso, J_alc_pso, J_nesterov, J_nelder_mead;
        Vector3r x_ellipsoid, x_pso, x_alc_pso, x_nesterov, x_nelder_mead;
        float t_ellipsoid, t_pso, t_alc_pso, t_nesterov, t_nelder_mead;
        for (auto& solver : solvers_)
        {
            switch (solver->getMode())
            {
                case PositionSolver::SolverMode::ELLIPSOID_METHOD:
                {
                    const auto& start = std::chrono::high_resolution_clock::now();
                    x_ellipsoid = solver->run(tab_P, tab_r, x0, J_ellipsoid);
                    const auto& end = std::chrono::high_resolution_clock::now();
                    t_ellipsoid = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
                    break;
                }

                case PositionSolver::SolverMode::PSO_ALGORITHM:
                {
                    const auto& start = std::chrono::high_resolution_clock::now();
                    x_pso = solver->run(tab_P, tab_r, x0, J_pso);
                    const auto& end = std::chrono::high_resolution_clock::now();
                    t_pso = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
                    break;
                }

                case PositionSolver::SolverMode::ALC_PSO_ALGORITHM:
                {
                    const auto& start = std::chrono::high_resolution_clock::now();
                    x_alc_pso = solver->run(tab_P, tab_r, x0, J_alc_pso);
                    const auto& end = std::chrono::high_resolution_clock::now();
                    t_alc_pso = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
                    break;
                }

                case PositionSolver::SolverMode::NESTEROV_ALGORITHM:
                {
                    const auto& start = std::chrono::high_resolution_clock::now();
                    x_nesterov = solver->run(tab_P, tab_r, x0, J_nesterov);
                    const auto& end = std::chrono::high_resolution_clock::now();
                    t_nesterov = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
                    break;
                }

                case PositionSolver::SolverMode::NELDER_MEAD:
                {
                    const auto& start = std::chrono::high_resolution_clock::now();
                    x_nelder_mead = solver->run(tab_P, tab_r, x0, J_nelder_mead);
                    const auto& end = std::chrono::high_resolution_clock::now();
                    t_nelder_mead = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
                    break;
                }
            }
        }
        
        // Publish position
        agent_.setpoint.header.stamp = RosUtils::now(node_);
        RosUtils::toMsg(x_ellipsoid, agent_.setpoint.point);
        agent_.setpoint_pub->publish(agent_.setpoint);

        // Create solver comparison message
        flychams_interfaces::msg::SolverDebug solver_debug_msg;
        solver_debug_msg.header.stamp = RosUtils::now(node_);
        // Ellipsoid Method
        solver_debug_msg.j_ellipsoid = J_ellipsoid;
        RosUtils::toMsg(x_ellipsoid, solver_debug_msg.x_ellipsoid);
        solver_debug_msg.t_ellipsoid = t_ellipsoid;
        // PSO Algorithm
        solver_debug_msg.j_pso = J_pso;
        RosUtils::toMsg(x_pso, solver_debug_msg.x_pso);
        solver_debug_msg.t_pso = t_pso;
        // ALC-PSO Algorithm
        solver_debug_msg.j_alc_pso = J_alc_pso;
        RosUtils::toMsg(x_alc_pso, solver_debug_msg.x_alc_pso);
        solver_debug_msg.t_alc_pso = t_alc_pso;
        // Nesterov Algorithm
        solver_debug_msg.j_nesterov = J_nesterov;
        RosUtils::toMsg(x_nesterov, solver_debug_msg.x_nesterov);
        solver_debug_msg.t_nesterov = t_nesterov;
        // Nelder-Mead Algorithm
        solver_debug_msg.j_nelder_mead = J_nelder_mead;
        RosUtils::toMsg(x_nelder_mead, solver_debug_msg.x_nelder_mead);
        solver_debug_msg.t_nelder_mead = t_nelder_mead;
        // Publish solver comparison results
        agent_.solver_debug_pub->publish(solver_debug_msg);
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
        if (params.mode == TrackingMode::MultiCamera)
        {
            params.f_min = tracking_params.central_head_params.f_min;
            params.f_max = tracking_params.central_head_params.f_max;
            params.f_ref = tracking_params.central_head_params.f_ref;
            params.s_min = tracking_params.central_head_params.s_min;
            params.s_max = tracking_params.central_head_params.s_max;
            params.s_ref = tracking_params.central_head_params.s_ref;
        }

        // Window parameters
        if (params.mode == TrackingMode::MultiWindow)
        {
            params.central_f = tracking_params.central_head_params.f_ref;
            params.lambda_min = tracking_params.central_window_params.lambda_min;
            params.lambda_max = tracking_params.central_window_params.lambda_max;
            params.lambda_ref = tracking_params.central_window_params.lambda_ref;
            params.s_min = tracking_params.central_window_params.s_min;
            params.s_max = tracking_params.central_window_params.s_max;
            params.s_ref = tracking_params.central_window_params.s_ref;
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