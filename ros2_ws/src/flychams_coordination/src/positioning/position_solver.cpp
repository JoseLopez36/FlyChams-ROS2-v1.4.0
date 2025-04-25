#include "flychams_coordination/positioning/position_solver.hpp"

using namespace flychams::core;

namespace flychams::coordination
{
    // ════════════════════════════════════════════════════════════════════════════
    // PUBLIC METHODS: Public methods for configuration and control
    // ════════════════════════════════════════════════════════════════════════════

    void PositionSolver::init(const SolverMode& mode, const Parameters& params)
    {
        // Set mode
        mode_ = mode;

        // Store parameters
        params_ = params;

        // Initialize the solver algorithms
        switch (mode_)
        {
        case SolverMode::NELDER_MEAD_NLOPT:
        {
            // Get Nelder-Mead parameters
            NelderMeadNLopt::Parameters nelder_mead_nlopt_params;
            nelder_mead_nlopt_params.x_min = params.x_min;
            nelder_mead_nlopt_params.x_max = params.x_max;
            nelder_mead_nlopt_params.eps = params.eps;
            nelder_mead_nlopt_params.tol = params.tol;
            nelder_mead_nlopt_params.max_iter = params.max_iter;

            // Initialize the Nelder-Mead solver with the parameters
            nelder_mead_nlopt_.init(nelder_mead_nlopt_params, params.cost_params);
            break;
        }

        case SolverMode::L_BFGS_NLOPT:
        {
            // Get L-BFGS parameters
            LBFGSNLopt::Parameters l_bfgs_nlopt_params;
            l_bfgs_nlopt_params.x_min = params.x_min;
            l_bfgs_nlopt_params.x_max = params.x_max;
            l_bfgs_nlopt_params.eps = params.eps;
            l_bfgs_nlopt_params.tol = params.tol;
            l_bfgs_nlopt_params.max_iter = params.max_iter;

            // Initialize the L-BFGS solver with the parameters
            l_bfgs_nlopt_.init(l_bfgs_nlopt_params, params.cost_params);
            break;
        }

        case SolverMode::ELLIPSOID_METHOD:
        {
            // Get Ellipsoid Method parameters
            EllipsoidMethod::Parameters ellipsoid_method_params;
            ellipsoid_method_params.x_min = params.x_min;
            ellipsoid_method_params.x_max = params.x_max;
            ellipsoid_method_params.eps = params.eps;
            ellipsoid_method_params.tol = params.tol;
            ellipsoid_method_params.max_iter = params.max_iter;

            // Initialize the Ellipsoid Method solver with the parameters
            ellipsoid_method_.init(ellipsoid_method_params, params.cost_params);
            break;
        }

        case SolverMode::PSO_ALGORITHM:
        {
            // Get PSO Algorithm parameters
            PSOAlgorithm::Parameters pso_algorithm_params;
            pso_algorithm_params.x_min = params.x_min;
            pso_algorithm_params.x_max = params.x_max;
            pso_algorithm_params.tol = params.tol;
            pso_algorithm_params.max_iter = params.max_iter;
            pso_algorithm_params.num_particles = params.num_particles;
            pso_algorithm_params.w_max = params.w_max;
            pso_algorithm_params.w_min = params.w_min;
            pso_algorithm_params.c1 = params.c1;
            pso_algorithm_params.c2 = params.c2;
            pso_algorithm_params.stagnation_limit = params.stagnation_limit;

            // Initialize the PSO Algorithm solver with the parameters
            pso_algorithm_.init(pso_algorithm_params, params.cost_params);
            break;
        }

        case SolverMode::ALC_PSO_ALGORITHM:
        {
            // Get ALC-PSO Algorithm parameters
            ALCPSOAlgorithm::Parameters alc_pso_algorithm_params;
            alc_pso_algorithm_params.x_min = params.x_min;
            alc_pso_algorithm_params.x_max = params.x_max;
            alc_pso_algorithm_params.tol = params.tol;
            alc_pso_algorithm_params.max_iter = params.max_iter;
            alc_pso_algorithm_params.num_particles = params.num_particles;
            alc_pso_algorithm_params.w_max = params.w_max;
            alc_pso_algorithm_params.w_min = params.w_min;
            alc_pso_algorithm_params.c1 = params.c1;
            alc_pso_algorithm_params.c2 = params.c2;
            alc_pso_algorithm_params.stagnation_limit = params.stagnation_limit;
            alc_pso_algorithm_params.max_lifespan = params.max_lifespan;
            alc_pso_algorithm_params.num_challenger_tests = params.num_challenger_tests;

            // Initialize the ALC-PSO Algorithm solver with the parameters
            alc_pso_algorithm_.init(alc_pso_algorithm_params, params.cost_params);
            break;
        }

        case SolverMode::NESTEROV_ALGORITHM:
        {
            // Get Nesterov Algorithm parameters
            NesterovAlgorithm::Parameters nesterov_algorithm_params;
            nesterov_algorithm_params.x_min = params.x_min;
            nesterov_algorithm_params.x_max = params.x_max;
            nesterov_algorithm_params.eps = params.eps;
            nesterov_algorithm_params.tol = params.tol;
            nesterov_algorithm_params.max_iter = params.max_iter;
            nesterov_algorithm_params.lipschitz_constant = params.lipschitz_constant;

            // Initialize the Nesterov Algorithm solver with the parameters
            nesterov_algorithm_.init(nesterov_algorithm_params, params.cost_params);
            break;
        }

        default:
            throw std::invalid_argument("Invalid solver mode");
        }
    }

    void PositionSolver::destroy()
    {
        // Destroy the solver algorithms
        switch (mode_)
        {
        case SolverMode::NELDER_MEAD_NLOPT:
        {
            nelder_mead_nlopt_.destroy();
            break;
        }

        case SolverMode::L_BFGS_NLOPT:
        {
            l_bfgs_nlopt_.destroy();
            break;
        }

        case SolverMode::ELLIPSOID_METHOD:
        {
            ellipsoid_method_.destroy();
            break;
        }

        case SolverMode::PSO_ALGORITHM:
        {
            pso_algorithm_.destroy();
            break;
        }

        case SolverMode::ALC_PSO_ALGORITHM:
        {
            alc_pso_algorithm_.destroy();
            break;
        }

        case SolverMode::NESTEROV_ALGORITHM:
        {
            nesterov_algorithm_.destroy();
            break;
        }

        default:
            throw std::invalid_argument("Invalid solver mode");
        }
    }

    Vector3r PositionSolver::run(const Matrix3Xr& tab_P, const RowVectorXr& tab_r, const Vector3r& x0, float& J)
    {
        // Run the optimization based on the mode
        switch (mode_)
        {
        case SolverMode::NELDER_MEAD_NLOPT:
        {
            return nelder_mead_nlopt_.run(tab_P, tab_r, x0, J);
        }

        case SolverMode::L_BFGS_NLOPT:
        {
            return l_bfgs_nlopt_.run(tab_P, tab_r, x0, J);
        }

        case SolverMode::ELLIPSOID_METHOD:
        {
            return ellipsoid_method_.run(tab_P, tab_r, x0, J);
        }

        case SolverMode::PSO_ALGORITHM:
        {
            return pso_algorithm_.run(tab_P, tab_r, J);
        }

        case SolverMode::ALC_PSO_ALGORITHM:
        {
            return alc_pso_algorithm_.run(tab_P, tab_r, J);
        }

        case SolverMode::NESTEROV_ALGORITHM:
        {
            return nesterov_algorithm_.run(tab_P, tab_r, x0, J);
        }

        default:
            throw std::invalid_argument("Invalid solver mode");
        }
    }

} // namespace flychams::coordination