#pragma once

// Cost functions
#include "flychams_coordination/positioning/cost_functions.hpp"

// Utilities
#include "flychams_core/types/core_types.hpp"

namespace flychams::coordination
{
    /**
     * ════════════════════════════════════════════════════════════════
     * @brief Solver for agent positioning using ellipsoid method
     * ════════════════════════════════════════════════════════════════
     * @author Jose Francisco Lopez Ruiz
     * @date 2025-04-17
     * ════════════════════════════════════════════════════════════════
     */
    class EllipsoidMethod
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
        };
        // Data
        struct Ellipsoid
        {
            core::Vector3r a0;     // Initial ellipsoid center
            core::Matrix3Xr A0;    // Initial ellipsoid matrix
        };
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
        Ellipsoid ellipsoid_;

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

            // Initialize ellipsoid
            ellipsoid_.a0 = (params_.x_max + params_.x_min) / 2.0f;
            ellipsoid_.A0 = core::Matrix3r::Zero();
            ellipsoid_.A0(0, 0) = 3.0f * pow((params_.x_max(0) - params_.x_min(0)) / 2.0f, 2);
            ellipsoid_.A0(1, 1) = 3.0f * pow((params_.x_max(1) - params_.x_min(1)) / 2.0f, 2);
            ellipsoid_.A0(2, 2) = 3.0f * pow((params_.x_max(2) - params_.x_min(2)) / 2.0f, 2);
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
            // Start from initial position
            core::Vector3r a = ellipsoid_.a0;
            core::Matrix3Xr A = ellipsoid_.A0;

            // Compute the gradient of the cost function
            float f;
            core::Vector3r grad_f;
            if (convex_relaxation)
                f = CostFunctions::J2(data_.tab_P, data_.tab_r, a, data_.x_hat, data_.cost_params, grad_f);
            else
                f = CostFunctions::J1(data_.tab_P, data_.tab_r, a, data_.cost_params, grad_f);

            // Compute the norm of the gradient of the cost function using the matrix A
            float factor_norm_f = std::sqrt(
                grad_f(0) * A(0, 0) * grad_f(0) +
                grad_f(1) * A(1, 1) * grad_f(1) +
                grad_f(2) * A(2, 2) * grad_f(2) +
                2.0f * (grad_f(0) * A(0, 1) * grad_f(1) +
                    grad_f(0) * A(0, 2) * grad_f(2) +
                    grad_f(1) * A(1, 2) * grad_f(2)));

                // Define the gradient of the constraints
            core::Matrix3Xr grad_g(3, 6);
            grad_g << 1.0f, 0.0f, 0.0f, -1.0f, 0.0f, 0.0f,
                0.0f, 1.0f, 0.0f, 0.0f, -1.0f, 0.0f,
                0.0f, 0.0f, 1.0f, 0.0f, 0.0f, -1.0f;

      // Define the constraint values
            core::VectorXr g(6);
            g << a(0) - params_.x_max(0),
                a(1) - params_.x_max(1),
                a(2) - params_.x_max(2),
                -a(0) + params_.x_min(0),
                -a(1) + params_.x_min(1),
                -a(2) + params_.x_min(2);

           // Find the first constraint that is violated (g[i] > 0)
            bool exists_max_g = false;
            int max_g_index = -1;
            for (int i = 0; i < 6; i++)
            {
                if (g(i) > 0.0f)
                {
                    exists_max_g = true;
                    max_g_index = i;
                    break;
                }
            }

            // Iterate while factor_norm is greater than tolerance, exists a violated constraint or max iterations is reached
            core::Vector3r best_a = a;  // Best solution found
            float best_f = f;           // Best value of the cost function
            int iter = 0;
            while ((factor_norm_f > params_.tol || exists_max_g) && iter < params_.max_iter)
            {
                core::Vector3r v;
                float d = 0.0f;

                if (!exists_max_g)
                {
                    v = grad_f / factor_norm_f;
                    d = (f - best_f) / factor_norm_f;
                }
                else
                {
                    // Calculate the norm of the gradient of the constraint using the matrix A
                    float factor_norm_g = std::sqrt(
                        grad_g(0, max_g_index) * A(0, 0) * grad_g(0, max_g_index) +
                        grad_g(1, max_g_index) * A(1, 1) * grad_g(1, max_g_index) +
                        grad_g(2, max_g_index) * A(2, 2) * grad_g(2, max_g_index) +
                        2.0f * (grad_g(0, max_g_index) * A(0, 1) * grad_g(1, max_g_index) +
                            grad_g(0, max_g_index) * A(0, 2) * grad_g(2, max_g_index) +
                            grad_g(1, max_g_index) * A(1, 2) * grad_g(2, max_g_index)));
                    v = grad_g.col(max_g_index) / factor_norm_g;
                    d = g(max_g_index) / factor_norm_g;
                }

                // Update the ellipsoid
                ellipsoidal_covering(a, v, d, A);

                // Update the norm of the gradient of the cost function
                if (convex_relaxation)
                    f = CostFunctions::J2(data_.tab_P, data_.tab_r, a, data_.x_hat, data_.cost_params, grad_f);
                else
                    f = CostFunctions::J1(data_.tab_P, data_.tab_r, a, data_.cost_params, grad_f);
                factor_norm_f = std::sqrt(
                    grad_f(0) * A(0, 0) * grad_f(0) +
                    grad_f(1) * A(1, 1) * grad_f(1) +
                    grad_f(2) * A(2, 2) * grad_f(2) +
                    2.0f * (grad_f(0) * A(0, 1) * grad_f(1) +
                        grad_f(0) * A(0, 2) * grad_f(2) +
                        grad_f(1) * A(1, 2) * grad_f(2)));

                    // Store the best solution
                if (f < best_f && !exists_max_g)
                {
                    best_f = f;
                    best_a = a;
                }

                // Find the first constraint that is violated (g[i] > 0)
                exists_max_g = false;
                for (int i = 0; i < 6; i++)
                {
                    if (g(i) > 0.0f)
                    {
                        exists_max_g = true;
                        max_g_index = i;
                        break;
                    }
                }

                // Update the constraint values
                g << a(0) - params_.x_max(0),
                    a(1) - params_.x_max(1),
                    a(2) - params_.x_max(2),
                    -a(0) + params_.x_min(0),
                    -a(1) + params_.x_min(1),
                    -a(2) + params_.x_min(2);

                iter++;
            }

            x_opt = best_a;
            return best_f;
        }

        void ellipsoidal_covering(core::Vector3r& a, const core::Vector3r& v, float d, core::Matrix3Xr& A)
        {
            // Constants for the ellipsoidal covering algorithm
            const float M = 3.0f; // Dimension of the problem
            float alpha = (1.0f + M * d) / (1.0f + d) * 2.0f / (1.0f + M);
            float r = -(1.0f + d) / 2.0f;
            float beta = std::pow((1.0f + r * alpha), 2) / (1.0f - alpha);

            // Store the result of A * v
            core::Vector3r Av = A * v;

            // Update A with: A = beta * (A - alpha * (A*v) * (A*v)')
            A = beta * (A - alpha * Av * Av.transpose());

            // Force symmetry in A
            A = (A + A.transpose()) / 2.0f;

            // Update the center of the ellipsoid: a = a + alpha * r * A * v
            a += alpha * r * Av;
        }
    };

} // namespace flychams::coordination