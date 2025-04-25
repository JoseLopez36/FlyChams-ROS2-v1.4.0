#pragma once

// Cost functions
#include "flychams_coordination/positioning/cost_functions.hpp"

// Solver algorithms
#include "flychams_coordination/positioning/nelder_mead_nlopt.hpp"
#include "flychams_coordination/positioning/l_bfgs_nlopt.hpp"
#include "flychams_coordination/positioning/ellipsoid_method.hpp"
#include "flychams_coordination/positioning/pso_algorithm.hpp"
#include "flychams_coordination/positioning/alc_pso_algorithm.hpp"
#include "flychams_coordination/positioning/nesterov_algorithm.hpp"

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
            NELDER_MEAD_NLOPT,
            L_BFGS_NLOPT,
            ELLIPSOID_METHOD,
            PSO_ALGORITHM,
            ALC_PSO_ALGORITHM,
            NESTEROV_ALGORITHM
        };
        // Parameters
        struct Parameters
        {
            // Cost parameters
            CostFunctions::Parameters cost_params;

            // Space constraints
            core::Vector3r x_min;
            core::Vector3r x_max;

            // Generic solver parameters
            float tol = 1e-6f;
            int max_iter = 100;
            float eps = 1.0f;

            // PSO parameters
            int num_particles = 50;
            float w_max = 0.4f;
            float w_min = 0.1f;
            float c1 = 1.0f;
            float c2 = 1.0f;
            int stagnation_limit = 5;

            // ALC-PSO parameters
            int max_lifespan = 60;
            int num_challenger_tests = 10;

            // Nesterov parameters
            float lipschitz_constant = 0.0f;
        };

    private: // Parameters
        SolverMode mode_;
        Parameters params_;

    private: // Data
        // Solver algorithms
        NelderMeadNLopt nelder_mead_nlopt_;
        LBFGSNLopt l_bfgs_nlopt_;
        EllipsoidMethod ellipsoid_method_;
        PSOAlgorithm pso_algorithm_;
        ALCPSOAlgorithm alc_pso_algorithm_;
        NesterovAlgorithm nesterov_algorithm_;

    public: // Public methods
        // Configuration
        void init(const SolverMode& mode, const Parameters& params);
        void destroy();
        // Getters
        int n() const { return params_.cost_params.n; }
        // Optimization
        core::Vector3r run(const core::Matrix3Xr& tab_P, const core::RowVectorXr& tab_r, const core::Vector3r& x0, float& J);
    };

} // namespace flychams::coordination