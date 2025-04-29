#pragma once

// Position solver
#include "flychams_coordination/positioning/position_solver.hpp"

// Solver algorithms
#include "flychams_coordination/assignment/suboptimal_combinatorial.hpp"

// Utilities
#include "flychams_core/types/core_types.hpp"

namespace flychams::coordination
{
    /**
     * ════════════════════════════════════════════════════════════════
     * @brief Solver for cluster-agent assignment problems
     * ════════════════════════════════════════════════════════════════
     * @author Jose Francisco Lopez Ruiz
     * @date 2025-01-31
     * ════════════════════════════════════════════════════════════════
     */
    class AssignmentSolver
    {
    public: // Types
        using SharedPtr = std::shared_ptr<AssignmentSolver>;
        // Modes
        enum class SolverMode
        {
            SUBOPTIMAL_COMBINATORIAL
        };
        // Parameters
        struct Parameters
        {
            // Optimization weights
            float observation_weight;
            float distance_weight;
            float switch_weight;
        };

    private: // Parameters
        SolverMode mode_;

    private: // Data
        // Solver algorithms
        SuboptimalCombinatorial suboptimal_combinatorial_;

    public: // Public methods
        // Configuration
        void init(const SolverMode& mode, const Parameters& params);
        void destroy();
        // Optimization
        core::RowVectorXi run(const core::Matrix3Xr& tab_x, const core::Matrix3Xr& tab_P, const core::RowVectorXr& tab_r, const core::RowVectorXi& X_prev, std::vector<PositionSolver::SharedPtr>& solvers);
    };

} // namespace flychams::coordination