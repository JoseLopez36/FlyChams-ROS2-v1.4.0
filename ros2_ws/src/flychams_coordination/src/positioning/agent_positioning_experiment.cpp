#include "flychams_coordination/positioning/agent_positioning_experiment.hpp"

using namespace flychams::core;

namespace flychams::coordination
{
    // ════════════════════════════════════════════════════════════════════════════
    // CONSTRUCTOR: Constructor and destructor
    // ════════════════════════════════════════════════════════════════════════════

    void AgentPositioningExperiment::onInit()
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

        // Create and initialize solvers
        for (const auto& mode : modes_)
        {
            solvers_.push_back(createSolver(agent_id_, solver_params_, mode));
        }

        // Create publisher for solver data
        solver_debug_pub_ = node_->create_publisher<flychams_interfaces::msg::SolverDebug>(
            RosUtils::replace("coordination/AGENTID/debug/solvers", "AGENTID", agent_id_), 10);

        // Log
        RCLCPP_INFO(node_->get_logger(), "Agent positioning experiment: Running with %d clusters...", K_);

        // Iterate
        float t = 0.0f;
        for (int n = 0; n < N_; n++)
        {
            // Update
            update(n, t);

            // Update time
            t += 1.0f / update_rate_;

            // Log
            RCLCPP_INFO(node_->get_logger(), "Agent positioning experiment: Completed %d iterations", n + 1);
        }
    }

    void AgentPositioningExperiment::onShutdown()
    {
        // Destroy solver
        for (auto& solver : solvers_)
        {
            solver->destroy();
        }
        solvers_.clear();
        // Destroy publisher
        solver_debug_pub_.reset();
    }

    // ════════════════════════════════════════════════════════════════════════════
    // UPDATE: Update positioning
    // ════════════════════════════════════════════════════════════════════════════

    void AgentPositioningExperiment::update(const int& n_step, const float& t_step)
    {
        // Add configured random Gaussian noise to cluster centers and radii
        const auto& [tab_P_noisy, tab_r_noisy] = addNoise(tab_P_, tab_r_, tab_P_noise_, tab_r_noise_);

        // Add central unit (mean of all selected clusters and maximum radius)
        const auto& [tab_P_k_with_central, tab_r_k_with_central] = addCentral(tab_P_noisy, tab_r_noisy);

        // Create message
        flychams_interfaces::msg::SolverDebug msg;

        // Fill global data
        msg.n_iterations = N_;
        msg.n_clusters = K_;
        RosUtils::toMsg(x0_, msg.x0);

        // Fill per-iteration data
        msg.n = n_step + 1;
        msg.t = t_step;
        msg.tab_p.resize(K_);
        msg.tab_r.resize(K_);
        for (int i = 0; i < K_; i++)
        {
            RosUtils::toMsg(tab_P_k_with_central.col(i + 1), msg.tab_p[i]);
            msg.tab_r[i] = tab_r_k_with_central(i + 1);
        }

        // Solve with each solver
        for (auto& solver : solvers_)
        {
            float J, t;
            Vector3r x;

            // Run solver
            const auto& start = std::chrono::high_resolution_clock::now();
            x = solver->run(tab_P_k_with_central, tab_r_k_with_central, x0_, J);
            const auto& end = std::chrono::high_resolution_clock::now();
            t = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() / 1.0e3f;

            // Add results to solver debug message
            switch (solver->getMode())
            {
            case PositionSolver::SolverMode::ELLIPSOID_METHOD:
            {
                msg.j_ellipsoid = J;
                RosUtils::toMsg(x, msg.x_ellipsoid);
                msg.t_ellipsoid = t;
                break;
            }

            case PositionSolver::SolverMode::PSO_ALGORITHM:
            {
                msg.j_pso = J;
                RosUtils::toMsg(x, msg.x_pso);
                msg.t_pso = t;
                break;
            }

            case PositionSolver::SolverMode::ALC_PSO_ALGORITHM:
            {
                msg.j_alc_pso = J;
                RosUtils::toMsg(x, msg.x_alc_pso);
                msg.t_alc_pso = t;
                break;
            }

            case PositionSolver::SolverMode::NESTEROV_ALGORITHM:
            {
                msg.j_nesterov = J;
                RosUtils::toMsg(x, msg.x_nesterov);
                msg.t_nesterov = t;
                break;
            }

            case PositionSolver::SolverMode::NELDER_MEAD_NLOPT:
            {
                msg.j_nelder_mead = J;
                RosUtils::toMsg(x, msg.x_nelder_mead);
                msg.t_nelder_mead = t;
                break;
            }

            case PositionSolver::SolverMode::L_BFGS_NLOPT:
            {
                msg.j_l_bfgs = J;
                RosUtils::toMsg(x, msg.x_l_bfgs);
                msg.t_l_bfgs = t;
                break;
            }
            }
        }

        // Publish results
        solver_debug_pub_->publish(msg);
    }

    // ════════════════════════════════════════════════════════════════════════════
    // POSITIONING: Positioning methods
    // ════════════════════════════════════════════════════════════════════════════

    PositionSolver::SharedPtr AgentPositioningExperiment::createSolver(const std::string& agent_id, const PositionSolver::Parameters& solver_params, const PositionSolver::SolverMode& solver_mode)
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

    std::vector<CostFunctions::TrackingUnit> AgentPositioningExperiment::createUnitParameters(const TrackingParameters& tracking_params)
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

    std::pair<core::Matrix3Xr, core::RowVectorXr> AgentPositioningExperiment::addNoise(const std::vector<core::Vector3r>& centers, const std::vector<float>& radii, const std::vector<Noise>& centers_noise, const std::vector<Noise>& radii_noise)
    {
        // Get number of tracking units
        int n = static_cast<int>(centers.size());

        // Add noise to cluster centers and radii
        core::Matrix3Xr centers_noisy(3, n);
        core::RowVectorXr radii_noisy(n);
        for (int i = 0; i < n; i++)
        {
            // Add center noise (only horizontal)
            for (int j = 0; j < 2; j++)
            {
                centers_noisy(j, i) = centers[i](j) + centers_noise[i].mean + random() * centers_noise[i].std;
            }
            centers_noisy(2, i) = centers[i](2);
            // Add radius noise
            radii_noisy(i) = radii[i] + radii_noise[i].mean + random() * radii_noise[i].std;
        }

        // Return noisy cluster centers and radii
        return std::make_pair(centers_noisy, radii_noisy);
    }

    std::pair<core::Matrix3Xr, core::RowVectorXr> AgentPositioningExperiment::addCentral(const core::Matrix3Xr& centers, const core::RowVectorXr& radii)
    {
        // Get number of tracking units
        int n = centers.cols();

        // Compute mean of all available clusters
        core::Vector3r centers_mean = core::Vector3r::Zero();
        for (int i = 0; i < n; i++)
        {
            centers_mean += centers.col(i);
        }
        centers_mean /= static_cast<float>(n);

        // Get the largest possible radius
        float radii_max = 0.0f;
        for (int i = 0; i < n; i++)
        {
            radii_max = std::max(radii_max, (centers_mean - centers.col(i)).norm() + radii(i));
        }

        // Create center matrix with central unit
        Matrix3Xr centers_with_central(3, n + 1);
        centers_with_central.col(0) = centers_mean;
        centers_with_central.block(0, 1, 3, n) = centers;

        // Create radii vector with central radius
        RowVectorXr radii_with_central(n + 1);
        radii_with_central(0) = radii_max;
        radii_with_central.tail(n) = radii;

        // Return central cluster and radius
        return std::make_pair(centers_with_central, radii_with_central);
    }

    float AgentPositioningExperiment::random()
    {
        // Generate a random number between 0 and 1
        return static_cast<float>(std::rand()) / static_cast<float>(RAND_MAX);
    }

} // namespace flychams::coordination
