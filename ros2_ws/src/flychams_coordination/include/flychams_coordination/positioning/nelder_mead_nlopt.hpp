#pragma once

// Non-Linear Optimization Library: https://github.com/stevengj/nlopt
#include <nlopt.hpp>

// Cost functions
#include "flychams_coordination/positioning/cost_functions.hpp"

// Utilities
#include "flychams_core/types/core_types.hpp"

namespace flychams::coordination
{
    /**
     * ════════════════════════════════════════════════════════════════
     * @brief Solver for agent positioning using Nelder-Mead method
     * ════════════════════════════════════════════════════════════════
     * @author Jose Francisco Lopez Ruiz
     * @date 2025-04-17
     * ════════════════════════════════════════════════════════════════
     */
    class NelderMeadNLopt
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
        // NLopt optimizer instance
        nlopt_opt opt_;
        // Cost function data
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

            // Create an NLopt optimizer
            opt_ = nlopt_create(NLOPT_LN_NELDERMEAD, 3); // 3 is the dimension of the problem

            // Optimizer options
            nlopt_set_xtol_rel(opt_, static_cast<double>(params_.tol)); // Set convergence tolerance
            nlopt_set_maxeval(opt_, params_.max_iter);                  // Maximum number of function evaluations
        }
        void destroy()
        {
            if (opt_)
            {
                nlopt_destroy(opt_);
            }
        }
        core::Vector3r run(const core::Matrix3Xr& tab_P, const core::RowVectorXr& tab_r, const core::Vector3r& x0, float& J)
        {
            // Check if NLopt optimizer is initialized
            if (!opt_)
            {
                throw std::runtime_error("NLopt optimizer not initialized");
            }

            // Define the optimization bounds
            const double lb[3] = { static_cast<double>(params_.x_min(0)), static_cast<double>(params_.x_min(1)), static_cast<double>(params_.x_min(2)) };
            const double ub[3] = { static_cast<double>(params_.x_max(0)), static_cast<double>(params_.x_max(1)), static_cast<double>(params_.x_max(2)) };
            nlopt_set_lower_bounds(opt_, lb);
            nlopt_set_upper_bounds(opt_, ub);

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

            // Set the objective cost function J1 along with the data
            nlopt_set_min_objective(opt_, funJ1, &data_);

            // Optimize for J1
            x_opt = x0; // Start from initial position
            J = optimize(x_opt);

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

                // Set the objective cost function J2 along with the data
                nlopt_set_min_objective(opt_, funJ2, &data_);

                // Optimize for J2
                J = optimize(x_opt);

                // Compute the norm of difference
                x_diff = (x_opt - x_opt_prev).norm();

                // Update the previous solution
                x_opt_prev = x_opt;
            }

            return J;
        }

    private: // Optimization methods
        float optimize(core::Vector3r& x_opt)
        {
            double x_opt_nlopt[3] = { static_cast<double>(x_opt(0)), static_cast<double>(x_opt(1)), static_cast<double>(x_opt(2)) };

            // Call the optimization algorithm
            // J: optimal value of the cost function
            // x_opt: value that minimizes the cost function
            double J;
            nlopt_optimize(opt_, x_opt_nlopt, &J);

            // Update the position
            x_opt << static_cast<float>(x_opt_nlopt[0]), static_cast<float>(x_opt_nlopt[1]), static_cast<float>(x_opt_nlopt[2]);

            // Return the optimal value of the cost function
            return static_cast<float>(J);
        }

        static double funJ1(unsigned n, const double* x, double* grad, void* data)
        {
            // Extract data
            core::Vector3r x_vec(static_cast<float>(x[0]), static_cast<float>(x[1]), static_cast<float>(x[2]));
            Data* data_ptr = reinterpret_cast<Data*>(data);

            // Calculate the cost for J1
            float J1 = CostFunctions::J1(data_ptr->tab_P, data_ptr->tab_r, x_vec, data_ptr->cost_params);

            return static_cast<double>(J1);
        }

        static double funJ2(unsigned n, const double* x, double* grad, void* data)
        {
            // Extract data
            core::Vector3r x_vec(static_cast<float>(x[0]), static_cast<float>(x[1]), static_cast<float>(x[2]));
            Data* data_ptr = reinterpret_cast<Data*>(data);

            // Calculate the cost for J2
            float J2 = CostFunctions::J2(data_ptr->tab_P, data_ptr->tab_r, x_vec, data_ptr->x_hat, data_ptr->cost_params);

            return static_cast<double>(J2);
        }
    };

} // namespace flychams::coordination