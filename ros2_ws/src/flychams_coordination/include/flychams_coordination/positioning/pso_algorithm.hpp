#pragma once

// Cost functions
#include "flychams_coordination/positioning/cost_functions.hpp"

// Utilities
#include "flychams_core/types/core_types.hpp"

namespace flychams::coordination
{
    /**
     * ════════════════════════════════════════════════════════════════
     * @brief Solver for agent positioning using PSO algorithm
     * ════════════════════════════════════════════════════════════════
     * @author Jose Francisco Lopez Ruiz
     * @date 2025-04-23
     * ════════════════════════════════════════════════════════════════
     */
    class PSOAlgorithm
    {
    public: // Types
        // Parameters
        struct Parameters
        {
            // Space constraints
            core::Vector3r x_min;
            core::Vector3r x_max;

            // Generic solver parameters
            float tol = 1e-5f;
            int max_iter = 100;

            // PSO parameters
            int num_particles = 50;         // Number of particles
            float w_max = 0.4f;             // Maximum inertia weight
            float w_min = 0.1f;             // Minimum inertia weight
            float c1 = 1.0f;                // Cognitive parameter
            float c2 = 1.0f;                // Social parameter
            int stagnation_limit = 5;       // Stagnation limit
        };
        // Data
        struct Particle
        {
            core::Vector3r position;
            core::Vector3r best_position;
            core::Vector3r velocity;
            float best_score;

            Particle()
            {
                position = core::Vector3r::Zero();
                best_position = core::Vector3r::Zero();
                velocity = core::Vector3r::Zero();
                best_score = HUGE_VALF;
            }
        };
        struct Data
        {
            // Cost function data
            core::Matrix3Xr tab_P;
            core::RowVectorXr tab_r;

            // Cost function parameters
            CostFunctions::Parameters cost_params;
        };

    private: // Parameters
        Parameters params_;

    private: // Data
        // Data
        Data data_;
        std::vector<Particle> particles_;

    public: // Public methods
        void init(const Parameters& params, const CostFunctions::Parameters& cost_params)
        {
            // Check parameters
            if ((params.x_min.array() > params.x_max.array()).any() || params.tol <= 0.0f || params.max_iter <= 0
                || params.num_particles <= 0 || params.w_max <= params.w_min || params.c1 <= 0.0f || params.c2 <= 0.0f
                || params.stagnation_limit <= 0)
            {
                throw std::invalid_argument("Invalid parameters");
            }

            // Store parameters
            params_ = params;
            data_.cost_params = cost_params;

            // Initialize data
            data_.tab_P = core::Matrix3Xr::Zero(3, data_.cost_params.n);
            data_.tab_r = core::RowVectorXr::Zero(data_.cost_params.n);

            // Initialize particles
            particles_.resize(params_.num_particles);
        }
        void destroy()
        {
            // Destroy particles
            particles_.clear();
        }
        core::Vector3r run(const core::Matrix3Xr& tab_P, const core::RowVectorXr& tab_r, float& J)
        {
            // Update data struct
            data_.tab_P = tab_P;
            data_.tab_r = tab_r;

            // Compute the optimal position
            core::Vector3r x_opt;
            J = optimize(x_opt);

            // Return the optimal position and the cost function value
            return x_opt;
        }

    private: // Optimization methods
        float optimize(core::Vector3r& x_opt)
        {
            // Prepare variables
            float global_best_score = HUGE_VALF;                                // Global best score
            core::Vector3r global_best_position = core::Vector3r::Zero();       // Global best position
            core::Vector3r global_best_position_prev = core::Vector3r::Zero();  // Previous global best position
            int stagnant_generations = 0;                                       // Stagnation counter (generations without improvement)

            // Initialize particles
            for (int k = 0; k < params_.num_particles; k++)
            {
                // Generate random position
                core::Vector3r r = randomVector();

                // Initialize particle position and best score
                particles_[k].position = params_.x_min + ((params_.x_max - params_.x_min).array() * r.array()).matrix();
                particles_[k].best_position = particles_[k].position;
                particles_[k].velocity = core::Vector3r::Zero();
                particles_[k].best_score = CostFunctions::J0(data_.tab_P, data_.tab_r, particles_[k].position, data_.cost_params);
            }

            // PSO iterations
            float w;
            for (int iter = 0; iter < params_.max_iter; iter++)
            {
                // Update inertia weight
                w = (params_.w_max - params_.w_min) * (params_.max_iter - iter) / params_.max_iter + params_.w_min;

                // Iterate over all particles to compute and update their scores
                for (int k = 0; k < params_.num_particles; k++)
                {
                    float score = CostFunctions::J0(data_.tab_P, data_.tab_r, particles_[k].position, data_.cost_params);

                    // Update best score if current score is better
                    if (score < particles_[k].best_score)
                    {
                        particles_[k].best_score = score;
                        particles_[k].best_position = particles_[k].position;
                    }

                    // Update global best score if current score is better
                    if (score < global_best_score)
                    {
                        global_best_score = score;
                        global_best_position = particles_[k].position;
                    }
                }

                // Update particles positions and velocities
                for (int k = 0; k < params_.num_particles; k++)
                {
                    // Generate random coefficients
                    float r1 = random();
                    float r2 = random();

                    // Update velocity
                    particles_[k].velocity = w * particles_[k].velocity + params_.c1 * r1 * (particles_[k].best_position - particles_[k].position) + params_.c2 * r2 * (global_best_position - particles_[k].position);

                    // Update position
                    particles_[k].position += particles_[k].velocity;
                }

                // Verify convergence
                if ((global_best_position - global_best_position_prev).norm() < params_.tol)
                {
                    stagnant_generations++;    // Increase stagnation counter if no improvement
                }
                else
                {
                    stagnant_generations = 0;  // Reset if improvement
                }

                global_best_position_prev = global_best_position;  // Update the previous best position

                // If there is no improvement in the last stagnation_limit generations, exit optimization
                if (stagnant_generations >= params_.stagnation_limit)
                {
                    break;
                }
            }

            // Return the global best score and position
            x_opt = global_best_position;
            return global_best_score;
        }

        float random()
        {
            // Generate a random number between 0 and 1
            return static_cast<float>(std::rand()) / static_cast<float>(RAND_MAX);
        }

        core::Vector3r randomVector()
        {
            // Generate a random vector with values in [0, 1]
            core::Vector3r r = core::Vector3r::Random();
            r = r.array().abs();
            return r;
        }
    };

} // namespace flychams::coordination