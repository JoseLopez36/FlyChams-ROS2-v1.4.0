#pragma once

// Non-Linear Optimization Library: https://github.com/stevengj/nlopt
#include <nlopt.hpp>

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
            NLOPT_NELDER_MEAD
        };
        // Data
        struct Data
        {
            core::TrackingParameters params;    // Tracking parameters
            core::Matrix3Xr tab_P;              // Cluster positions matrix
            core::RowVectorXr tab_r;            // Cluster radii vector
            core::Vector3r xHat;                // Estimated optimal position

            Data(const int& num_clusters)
                : params(), tab_P(3, num_clusters), tab_r(num_clusters), xHat() {
            }
        };

    private: // Parameters
        SolverMode mode_;
        float tol_;
        int max_iter_;
        float eps_;

    private: // Data
        // NLopt optimizer instance
        nlopt_opt opt_;

    public: // Public methods
        // Configuration
        void init();
        void destroy();
        void reset();
        void setMode(const SolverMode& mode);
        void setParameters(const float& tol, const int& max_iter, const float& eps);
        // Optimization
        core::Vector3r run(const core::Matrix3Xr& tab_P, const core::RowVectorXr& tab_r,
            const core::Vector3r& x0, const float& min_h, const float& max_h,
            const core::TrackingParameters& params);

    private: // Implementation
        // Optimization stages
        core::Vector3r preOptimization(const core::Vector3r& x0, Data& data);
        core::Vector3r iterativeOptimization(const core::Vector3r& x0, Data& data);
        // Objective functions
        static double funJ1(unsigned n, const double* x, double* grad, void* data);
        static double funJ2(unsigned n, const double* x, double* grad, void* data);
        // Cost function implementations
        static float calculateCameraJ1(const core::Vector3r& z, const float& r, const core::Vector3r& x, const core::CameraParameters& camera_params, const core::ProjectionParameters& projection_params);
        static float calculateCameraJ2(const core::Vector3r& z, const float& r, const core::Vector3r& x, const core::Vector3r& xHat, const core::CameraParameters& camera_params, const core::ProjectionParameters& projection_params);
        static float calculateWindowJ1(const core::Vector3r& z, const float& r, const core::Vector3r& x, const core::WindowParameters& window_params, const core::ProjectionParameters& projection_params);
        static float calculateWindowJ2(const core::Vector3r& z, const float& r, const core::Vector3r& x, const core::Vector3r& xHat, const core::WindowParameters& window_params, const core::ProjectionParameters& projection_params);
        // Optimization utility methods
        float optimize(core::Vector3r& xOpt);
    };

} // namespace flychams::coord