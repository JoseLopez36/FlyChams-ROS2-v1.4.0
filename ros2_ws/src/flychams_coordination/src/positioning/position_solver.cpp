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

            default:
                throw std::invalid_argument("Invalid solver mode");
        }
    }

    Vector3r PositionSolver::run(const Matrix3Xr& tab_P, const RowVectorXr& tab_r, const Vector3r& x0)
    {
        // Run the optimization based on the mode
        switch (mode_)
        {
            case SolverMode::NELDER_MEAD:
            {
                return nelder_mead_.run(tab_P, tab_r, x0);
            }

            case SolverMode::ELLIPSOID_METHOD:
            {
                return ellipsoid_method_.run(tab_P, tab_r, x0);
            }

            default:
                throw std::invalid_argument("Invalid solver mode");
        }
    }

} // namespace flychams::coordination