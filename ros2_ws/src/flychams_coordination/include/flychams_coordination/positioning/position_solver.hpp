#pragma once

// Cost functions
#include "flychams_coordination/positioning/cost_functions.hpp"

// Solver algorithms
#include "flychams_coordination/positioning/nelder_mead.hpp"
#include "flychams_coordination/positioning/ellipsoid_method.hpp"

// Utilities
#include "flychams_core/types/core_types.hpp"

namespace flychams::coordination
{
    /**
     * ════════════════════════════════════════════════════════════════
     * @brief Solver for agent positioning
     *
     * @details
     * This class implements optimization algorithms for finding optimal
     * agent positions based on target positions and visibility constraints.
     * It uses non-linear optimization techniques to minimize cost functions
     * that consider camera parameters, target sizes, and height constraints.
     *
     * ════════════════════════════════════════════════════════════════
     * @author Jose Francisco Lopez Ruiz
     * @date 2025-01-31
     * ════════════════════════════════════════════════════════════════
     */
    class PositionSolver
    {
    public: // Types
        using SharedPtr = std::shared_ptr<PositionSolver>;
        // Modes
        enum class SolverMode
        {
            NELDER_MEAD,
            ELLIPSOID_METHOD
        };
        // Parameters
        struct Parameters
        {
            // Cost parameters
            CostFunctions::Parameters cost_params;

            // Space constraints
            core::Vector3r x_min;
            core::Vector3r x_max;

            // Solver parameters
            float tol = 1e-6f;
            int max_iter = 100;
            float eps = 1.0f;
        };   

    private: // Parameters
        SolverMode mode_;

    private: // Data
        // Solver algorithms
        NelderMead nelder_mead_;
        EllipsoidMethod ellipsoid_method_;

    public: // Public methods
        // Configuration
        void init(const SolverMode& mode, const Parameters& params);
        void destroy();
        // Optimization
        core::Vector3r run(const core::Matrix3Xr& tab_P, const core::RowVectorXr& tab_r, const core::Vector3r& x0);
    };

} // namespace flychams::coordination