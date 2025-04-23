#include "flychams_coordination/assignment/assignment_solver.hpp"

using namespace flychams::core;

namespace flychams::coordination
{
    // ════════════════════════════════════════════════════════════════════════════
    // PUBLIC METHODS: Public methods for configuration and control
    // ════════════════════════════════════════════════════════════════════════════

    void AssignmentSolver::init(const SolverMode& mode, const Parameters& params)
    {
        // Set mode
        mode_ = mode;

        // Initialize the solver algorithms
        switch (mode_)
        {
            case SolverMode::SUBOPTIMAL_COMBINATORIAL:
            {
                // Get Suboptimal Combinatorial parameters
                SuboptimalCombinatorial::Parameters suboptimal_combinatorial_params;
                suboptimal_combinatorial_params.observation_weight = params.observation_weight;
                suboptimal_combinatorial_params.distance_weight = params.distance_weight;
                suboptimal_combinatorial_params.switch_weight = params.switch_weight;

                // Initialize the Suboptimal Combinatorial solver with the parameters
                suboptimal_combinatorial_.init(suboptimal_combinatorial_params);
                break;
            }

            default:
                throw std::invalid_argument("Invalid solver mode");
        }
    }

    void AssignmentSolver::destroy()
    {
        // Destroy the solver algorithms
        switch (mode_)
        {
            case SolverMode::SUBOPTIMAL_COMBINATORIAL:
            {
                suboptimal_combinatorial_.destroy();
                break;
            }

            default:
                throw std::invalid_argument("Invalid solver mode");
        }
    }

    core::RowVectorXi AssignmentSolver::run(const core::Matrix3Xr& tab_x, const core::Matrix3Xr& tab_P, const core::RowVectorXr& tab_r, const core::RowVectorXi& X_prev, std::vector<PositionSolver::SharedPtr>& solvers)
    {
        // Run the assignment based on the mode
        switch (mode_)
        {
            case SolverMode::SUBOPTIMAL_COMBINATORIAL:
            {
                return suboptimal_combinatorial_.run(tab_x, tab_P, tab_r, X_prev, solvers);
            }

            default:
                throw std::invalid_argument("Invalid solver mode");
        }
    }

} // namespace flychams::coordination