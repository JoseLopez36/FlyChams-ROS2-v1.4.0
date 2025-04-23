#include "flychams_coordination/assignment/agent_assignment.hpp"

using namespace flychams::core;

namespace flychams::coordination
{
    // ════════════════════════════════════════════════════════════════════════════
    // CONSTRUCTOR: Constructor and destructor
    // ════════════════════════════════════════════════════════════════════════════

    void AgentAssignment::onInit()
    {
        // Get parameters from parameter server
        // Get update rate
        update_rate_ = RosUtils::getParameterOr<float>(node_, "agent_assignment.update_rate", 1.0f);
        // Get assignment mode
        AssignmentSolver::SolverMode assignment_solver_mode = static_cast<AssignmentSolver::SolverMode>(RosUtils::getParameterOr<uint8_t>(node_, "agent_assignment.solver_mode", 0));
        // Get assignment parameters
        float observation_weight = RosUtils::getParameterOr<float>(node_, "agent_assignment.observation_weight", 1.0f);
        float distance_weight = RosUtils::getParameterOr<float>(node_, "agent_assignment.distance_weight", 10.0f);
        float switch_weight = RosUtils::getParameterOr<float>(node_, "agent_assignment.switch_weight", 5000.0f);
        // Get position solver parameters
        position_solver_mode_ = static_cast<PositionSolver::SolverMode>(RosUtils::getParameterOr<uint8_t>(node_, "agent_positioning.solver_mode", 0));
        eps_ = RosUtils::getParameterOr<float>(node_, "agent_positioning.eps", 1.0e-1f);
        convergence_tolerance_ = RosUtils::getParameterOr<float>(node_, "agent_positioning.convergence_tolerance", 1.0e-5f);
        max_iterations_ = RosUtils::getParameterOr<int>(node_, "agent_positioning.max_iterations", 100);

        // Initialize data
        agents_.clear();
        A_.clear();
        clusters_.clear();
        T_.clear();
        X_prev_.resize(0);

        // Create and initialize assignment solver
        // Note: Position solvers will be created when adding agents
        solver_ = std::make_shared<AssignmentSolver>();
        AssignmentSolver::Parameters solver_params;
        solver_params.observation_weight = observation_weight;
        solver_params.distance_weight = distance_weight;
        solver_params.switch_weight = switch_weight;
        solver_->init(assignment_solver_mode, solver_params);

        // Set update timer
        update_timer_ = RosUtils::createTimer(node_, update_rate_,
            std::bind(&AgentAssignment::update, this), module_cb_group_);
    }

    void AgentAssignment::onShutdown()
    {
        // Destroy assignment solver
        solver_->destroy();
        // Destroy agents and clusters
        agents_.clear();
        A_.clear();
        clusters_.clear();
        T_.clear();
        // Destroy update timer
        update_timer_.reset();
    }

    // ════════════════════════════════════════════════════════════════════════════
    // PUBLIC METHODS: Public methods for adding/removing clusters and agents
    // ════════════════════════════════════════════════════════════════════════════

    void AgentAssignment::addAgent(const ID& agent_id)
    {
        // Create and add agent
        agents_.insert({ agent_id, Agent() });
        A_.insert(agent_id); // Add agent to ordered set

        // Create and initialize position solver
        createPositionSolver(agents_[agent_id].position_solver, agent_id);

        // Add tracking units to previous assignments
        X_prev_.resize(X_prev_.size() + agents_[agent_id].position_solver->n());
        X_prev_.setConstant(-1);

        // Create agent status subscriber
        agents_[agent_id].status_sub = topic_tools_->createAgentStatusSubscriber(agent_id,
            [this, agent_id](const AgentStatusMsg::SharedPtr msg)
            {
                this->agentStatusCallback(agent_id, msg);
            }, sub_options_with_module_cb_group_);

        // Create agent position subscriber
        agents_[agent_id].position_sub = topic_tools_->createAgentPositionSubscriber(agent_id,
            [this, agent_id](const PointStampedMsg::SharedPtr msg)
            {
                this->agentPositionCallback(agent_id, msg);
            }, sub_options_with_module_cb_group_);

        // Create agent assignment publisher
        agents_[agent_id].assignment_pub = topic_tools_->createAgentAssignmentPublisher(agent_id);
    }

    void AgentAssignment::removeAgent(const ID& agent_id)
    {
        // Remove agent from map
        agents_.erase(agent_id);
        A_.erase(agent_id); // Remove agent from ordered set
    }

    void AgentAssignment::addCluster(const ID& cluster_id)
    {
        // Create and add cluster
        clusters_.insert({ cluster_id, Cluster() });
        T_.insert(cluster_id); // Add cluster to ordered set

        // Create cluster geometry subscriber
        clusters_[cluster_id].geometry_sub = topic_tools_->createClusterGeometrySubscriber(cluster_id,
            [this, cluster_id](const ClusterGeometryMsg::SharedPtr msg)
            {
                this->clusterGeometryCallback(cluster_id, msg);
            }, sub_options_with_module_cb_group_);
    }

    void AgentAssignment::removeCluster(const ID& cluster_id)
    {
        // Remove cluster from map
        clusters_.erase(cluster_id);
        T_.erase(cluster_id); // Remove cluster from ordered set
    }

    // ════════════════════════════════════════════════════════════════════════════
    // CALLBACKS: Callback functions
    // ════════════════════════════════════════════════════════════════════════════

    void AgentAssignment::clusterGeometryCallback(const ID& cluster_id, const ClusterGeometryMsg::SharedPtr msg)
    {
        // Update cluster geometry
        clusters_[cluster_id].center = msg->center;
        clusters_[cluster_id].radius = msg->radius;
        clusters_[cluster_id].has_geometry = true;
    }

    void AgentAssignment::agentStatusCallback(const ID& agent_id, const AgentStatusMsg::SharedPtr msg)
    {
        // Update agent status
        agents_[agent_id].status = static_cast<AgentStatus>(msg->status);
        agents_[agent_id].has_status = true;
    }

    void AgentAssignment::agentPositionCallback(const ID& agent_id, const PointStampedMsg::SharedPtr msg)
    {
        // Update agent position
        agents_[agent_id].position = msg->point;
        agents_[agent_id].has_position = true;
    }

    // ════════════════════════════════════════════════════════════════════════════
    // UPDATE: Update assignment
    // ════════════════════════════════════════════════════════════════════════════

    void AgentAssignment::update()
    {
        // Check if we have a valid agent status, position and cluster geometry
        for (const auto& [agent_id, agent] : agents_)
        {
            if (!agent.has_status || !agent.has_position)
            {
                RCLCPP_WARN(node_->get_logger(), "Agent assignment: Agent %s has no status or position", agent_id.c_str());
                return; // Skip assignment if we don't have a valid agent status or position
            }

            // Check if we are in the correct state to assign clusters
            if (agent.status != AgentStatus::TRACKING)
            {
                RCLCPP_WARN(node_->get_logger(), "Agent assignment: Agent %s is not in the correct state to assign clusters",
                    agent_id.c_str());
                return;
            }
        }
        for (const auto& [cluster_id, cluster] : clusters_)
        {
            if (!cluster.has_geometry)
            {
                RCLCPP_WARN(node_->get_logger(), "Agent assignment: Cluster %s has no geometry", cluster_id.c_str());
                return; // Skip assignment if we don't have a valid cluster geometry
            }
        }

        // Get vectors of agent and cluster data with ordered data
        // It is important that the data is ordered according to the ordered sets A_ and T_,
        // since the assignment solver assumes that the data follows the same order always.
        // Agents
        int n_agents = static_cast<int>(A_.size());
        Matrix3Xr tab_x(3, n_agents);
        std::vector<PositionSolver::SharedPtr> solvers(n_agents);
        int k = 0;
        for (const auto& agent_id : A_)
        {
            const auto& agent = agents_[agent_id];
            tab_x.col(k) = RosUtils::fromMsg(agent.position);
            solvers[k] = agent.position_solver;
            k++;
        }
        // Clusters
        int n_clusters = static_cast<int>(T_.size());
        Matrix3Xr tab_P(3, n_clusters);
        RowVectorXr tab_r(n_clusters);
        int i = 0;
        for (const auto& cluster_id : T_)
        {
            const auto& cluster = clusters_[cluster_id];
            tab_P.col(i) = RosUtils::fromMsg(cluster.center);
            tab_r(i) = cluster.radius;
            i++;
        }

        // Perform agent assignment
        RCLCPP_INFO(node_->get_logger(), "Agent assignment: Performing agent assignment...");
        RowVectorXi X = solver_->run(tab_x, tab_P, tab_r, X_prev_, solvers);

        // Update previous assignment
        X_prev_ = X;

        // Create and publish an assignment message for each agent
        k = 0;
        int t = 0;
        for (const auto& agent_id : A_)
        {
            // Create message
            AgentAssignmentMsg msg;
            msg.header.stamp = node_->get_clock()->now();

            // Get assignment
            int n = solvers[k]->n();
            for (int i = 0; i < n; i++)
            {
                const int& cluster_index = X(t);
                const std::string cluster_id = *std::next(T_.begin(), cluster_index);
                msg.cluster_ids.push_back(cluster_id);
                t++;
            }

            // Publish
            agents_[agent_id].assignment_pub->publish(msg);
            
            // Log assignment
            RCLCPP_INFO(node_->get_logger(), "Agent assignment: Agent %s assigned to %d clusters", agent_id.c_str(), n);
            for (int i = 0; i < n; i++)
            {
                RCLCPP_INFO(node_->get_logger(), "Agent assignment:     - Cluster %s", msg.cluster_ids[i].c_str());
            }

            k++;
        }
    }

    // ════════════════════════════════════════════════════════════════════════════
    // UTILITY METHODS
    // ════════════════════════════════════════════════════════════════════════════

    void AgentAssignment::createPositionSolver(PositionSolver::SharedPtr& solver, const ID& agent_id)
    {
        // Get positioning parameters
        const auto& config_ptr = config_tools_->getConfig();
        const auto& agent_ptr = config_tools_->getAgent(agent_id);
        const auto& tracking_params = config_tools_->getTrackingParameters(agent_id);
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

        // Create and initialize position solver
        solver = std::make_shared<PositionSolver>();
        PositionSolver::Parameters solver_params;
        solver_params.cost_params = cost_params;
        solver_params.x_min = x_min;
        solver_params.x_max = x_max;
        solver_params.eps = eps_;
        solver_params.tol = convergence_tolerance_;
        solver_params.max_iter = max_iterations_;
        solver->init(position_solver_mode_, solver_params);
    }

    CostFunctions::TrackingUnit AgentAssignment::centralUnitParameters(const TrackingParameters& tracking_params)
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

    std::vector<CostFunctions::TrackingUnit> AgentAssignment::trackingUnitParameters(const TrackingParameters& tracking_params)
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