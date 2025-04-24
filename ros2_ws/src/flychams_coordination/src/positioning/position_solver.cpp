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
            case SolverMode::NELDER_MEAD:
            {
                // Get Nelder-Mead parameters
                NelderMead::Parameters nelder_mead_params;
                nelder_mead_params.x_min = params.x_min;
                nelder_mead_params.x_max = params.x_max;
                nelder_mead_params.eps = params.eps;
                nelder_mead_params.tol = params.tol;
                nelder_mead_params.max_iter = params.max_iter;

                // Initialize the Nelder-Mead solver with the parameters
                nelder_mead_.init(nelder_mead_params, params.cost_params);
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

            case SolverMode::ACL_PSO_ALGORITHM:
            {
                // Get ACL-PSO Algorithm parameters
                ACLPSOAlgorithm::Parameters acl_pso_algorithm_params;
                acl_pso_algorithm_params.x_min = params.x_min;
                acl_pso_algorithm_params.x_max = params.x_max;
                acl_pso_algorithm_params.tol = params.tol;
                acl_pso_algorithm_params.max_iter = params.max_iter;
                acl_pso_algorithm_params.num_particles = params.num_particles;
                acl_pso_algorithm_params.w_max = params.w_max;
                acl_pso_algorithm_params.w_min = params.w_min;
                acl_pso_algorithm_params.c1 = params.c1;
                acl_pso_algorithm_params.c2 = params.c2;
                acl_pso_algorithm_params.stagnation_limit = params.stagnation_limit;
                acl_pso_algorithm_params.max_lifespan = params.max_lifespan;
                acl_pso_algorithm_params.num_challenger_tests = params.num_challenger_tests;

                // Initialize the ACL-PSO Algorithm solver with the parameters
                acl_pso_algorithm_.init(acl_pso_algorithm_params, params.cost_params);
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
            case SolverMode::NELDER_MEAD:
            {
                nelder_mead_.destroy();
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

            case SolverMode::ACL_PSO_ALGORITHM:
            {
                acl_pso_algorithm_.destroy();
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
            case SolverMode::NELDER_MEAD:
            {
                return nelder_mead_.run(tab_P, tab_r, x0, J);
            }

            case SolverMode::ELLIPSOID_METHOD:
            {
                return ellipsoid_method_.run(tab_P, tab_r, x0, J);
            }

            case SolverMode::PSO_ALGORITHM:
            {
                return pso_algorithm_.run(tab_P, tab_r, J);
            }

            case SolverMode::ACL_PSO_ALGORITHM:
            {
                return acl_pso_algorithm_.run(tab_P, tab_r, J);
            }

            default:
                throw std::invalid_argument("Invalid solver mode");
        }
    }

} // namespace flychams::coordination