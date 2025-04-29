#pragma once

// Standard includes
#include <iostream>

// Cost functions
#include "flychams_coordination/positioning/cost_functions.hpp"

// Utilities
#include "flychams_core/types/core_types.hpp"

namespace flychams::coordination
{
    /**
     * ════════════════════════════════════════════════════════════════
     * @brief Solver for agent positioning using Nesterov's algorithm
     * ════════════════════════════════════════════════════════════════
     * @author Jose Francisco Lopez Ruiz
     * @date 2025-04-24
     * ════════════════════════════════════════════════════════════════
     */
    class NesterovAlgorithm
    {
    public: // Types
        // Parameters
        struct Parameters
        {
            // Space constraints
            core::Vector3r x_min;
            core::Vector3r x_max;

            // Generic solver parameters
            float eps = 1e-1f;
            float tol = 1e-5f;
            int max_iter = 100;

            // Nesterov parameters
            float lipschitz_constant = 0.0f;
        };
        // Data
        struct Data
        {
            // Cost function data
            core::Matrix3Xr tab_P;
            core::RowVectorXr tab_r;
            core::Vector3r x_hat;

            // Cost function parameters
            CostFunctions::Parameters cost_params;
        };

    private: // Parameters
        Parameters params_;

    private: // Data
        // Data
        Data data_;

    public: // Public methods
        void init(const Parameters& params, const CostFunctions::Parameters& cost_params)
        {
            // Check parameters
            if ((params.x_min.array() > params.x_max.array()).any() || params.tol <= 0.0f || params.max_iter <= 0 || params.eps <= 0.0f)
            {
                throw std::invalid_argument("Invalid parameters");
            }

            // Store parameters
            params_ = params;
            data_.cost_params = cost_params;

            // Initialize data
            data_.tab_P = core::Matrix3Xr::Zero(3, data_.cost_params.n);
            data_.tab_r = core::RowVectorXr::Zero(data_.cost_params.n);
            data_.x_hat = core::Vector3r::Zero();
        }
        void destroy()
        {
            // Nothing to destroy
        }
        core::Vector3r run(const core::Matrix3Xr& tab_P, const core::RowVectorXr& tab_r, const core::Vector3r& x0, float& J)
        {
            // Clip position to constraints
            core::Vector3r x0_clipped = x0;
            x0_clipped(0) = std::min(std::max(x0(0), params_.x_min(0)), params_.x_max(0));
            x0_clipped(1) = std::min(std::max(x0(1), params_.x_min(1)), params_.x_max(1));
            x0_clipped(2) = std::min(std::max(x0(2), params_.x_min(2)), params_.x_max(2));

            // Update data struct
            data_.tab_P = tab_P;
            data_.tab_r = tab_r;
            data_.x_hat = core::Vector3r::Zero();

            // First optimization (solve for initial position)
            core::Vector3r x_opt_1;
            float J_1 = preOptimization(x0_clipped, x_opt_1);

            // Iterative optimization (solve for optimal position)
            core::Vector3r x_opt_2;
            float J_2 = iterativeOptimization(x_opt_1, x_opt_2);

            // Return the optimal position and the cost function value
            J = J_2;
            return x_opt_2;
        }

    private: // Optimization stages
        float preOptimization(const core::Vector3r& x0, core::Vector3r& x_opt)
        {
            float J = 0.0f;

            // Optimize for J1
            x_opt = x0;
            J = optimize(x_opt, false);

            return J;
        }

        float iterativeOptimization(const core::Vector3r& x0, core::Vector3r& x_opt)
        {
            float J = 0.0f;

            // Initialize with the previous solution
            core::Vector3r x_opt_prev = x0;
            x_opt = x_opt_prev;

            // Iteratively call the optimization algorithm that implements the convex relaxation (J2)
            float x_diff = HUGE_VALF;
            while (x_diff > params_.eps)
            {
                // Define the x_hat parameter of cost function J2
                data_.x_hat << x_opt_prev[0], x_opt_prev[1], x_opt_prev[2];

                // Optimize for J2
                J = optimize(x_opt, true);

                // Compute the norm of difference
                x_diff = (x_opt - x_opt_prev).norm();

                // Update the previous solution
                x_opt_prev = x_opt;
            }

            return J;
        }

    private: // Optimization methods
        float optimize(core::Vector3r& x_opt, bool convex_relaxation)
        {
            // Check Lipschitz constant (if not provided, it will be computed heuristically)
            float L = params_.lipschitz_constant;
            if (L <= 1e-6f)
            {
                // Compute Lipschitz constant using the number of tracking units
                L = 15.0f * (static_cast<float>(data_.cost_params.n) + 1.0f);
            }

            // Initialize variables
            float f_prev = HUGE_VALF;
            core::Vector3r x = x_opt;
            core::Vector3r x_prev = x_opt;
            float theta = 1.0f;
            float theta_prev = 1.0f;

            // Iterate until convergence or max iterations
            for (int k = 0; k < params_.max_iter; k++)
            {
                float beta = (theta_prev - 1.0f) / theta;

                // Calculate position for the k-th iteration
                core::Vector3r y = x + beta * (x - x_prev);

                // Compute the cost function and gradient with the current position
                float f;
                core::Vector3r grad_f;
                if (convex_relaxation)
                    f = CostFunctions::J2(data_.tab_P, data_.tab_r, y, data_.x_hat, data_.cost_params, grad_f);
                else
                    f = CostFunctions::J1(data_.tab_P, data_.tab_r, y, data_.cost_params, grad_f);

                // Check if the cost function is increasing
                if (f > f_prev)
                {
                    // Reset the momentum (avoids overshooting)
                    x_prev = x;
                    theta = 1.0f;
                    theta_prev = 1.0f;
                }

                // Accelerated gradient step
                core::Vector3r x_next = y - (1.0f / L) * grad_f;

                // Limit to the constraints
                if (x_next.x() < params_.x_min.x()) x_next.x() = params_.x_min.x();
                if (x_next.x() > params_.x_max.x()) x_next.x() = params_.x_max.x();
                if (x_next.y() < params_.x_min.y()) x_next.y() = params_.x_min.y();
                if (x_next.y() > params_.x_max.y()) x_next.y() = params_.x_max.y();
                if (x_next.z() < params_.x_min.z()) x_next.z() = params_.x_min.z();
                if (x_next.z() > params_.x_max.z()) x_next.z() = params_.x_max.z();

                // Check convergence
                float norm_diff = (x_next - x).norm();
                if (norm_diff < params_.tol)
                {
                    break;
                }

                // Update theta
                theta_prev = theta;
                theta = 0.5f * (1.0f + std::sqrt(1.0f + 4.0f * std::pow(theta, 2)));

                // Update xk and xk_prev
                x_prev = x;
                x = x_next;
            }

            // Evaluate the final cost (without gradient)
            float f;
            if (convex_relaxation)
                f = CostFunctions::J2(data_.tab_P, data_.tab_r, x, data_.x_hat, data_.cost_params);
            else
                f = CostFunctions::J1(data_.tab_P, data_.tab_r, x, data_.cost_params);

            // Return the cost function value and position
            x_opt = x;
            return f;
        }
    };

} // namespace flychams::coordination