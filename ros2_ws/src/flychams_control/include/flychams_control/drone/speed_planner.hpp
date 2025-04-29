#pragma once

// Core includes
#include "flychams_core/types/core_types.hpp"

namespace flychams::control
{
    /**
     * ════════════════════════════════════════════════════════════════
     * @brief Speed planner
     * ════════════════════════════════════════════════════════════════
     * @author Jose Francisco Lopez Ruiz
     * @date 2025-03-27
     * ════════════════════════════════════════════════════════════════
     */
    class SpeedPlanner
    {
    public: // Constructor/Destructor
        SpeedPlanner()
            : min_speed_(0.0f), max_speed_(10.0f), min_distance_(0.0f),
            max_distance_(100.0f), max_acceleration_(1.0f), curr_speed_(0.0f),
            curr_distance_(0.0f)
        {
            // Nothing to do
        }

    private: // Parameters
        float min_speed_;
        float max_speed_;
        float min_distance_;
        float max_distance_;
        float max_acceleration_;

    private: // Data
        float curr_speed_;
        float curr_distance_;

    public: // Public methods
        void setParameters(const float& min_speed, const float& max_speed, const float& min_distance, const float& max_distance, const float& max_acceleration)
        {
            min_speed_ = min_speed;
            max_speed_ = max_speed;
            min_distance_ = min_distance;
            max_distance_ = max_distance;
            max_acceleration_ = max_acceleration;
        }

        float planSpeed(const float& curr_x, const float& curr_y, const float& curr_z, const float& target_x, const float& target_y, const float& target_z, const float& dt)
        {
            // Check if time step is valid
            if (dt <= 0.0f)
            {
                return curr_speed_;
            }

            // Compute distance to goal
            curr_distance_ = std::sqrt(std::pow(target_x - curr_x, 2) + std::pow(target_y - curr_y, 2) + std::pow(target_z - curr_z, 2));

            // Compute target speed based on distance to goal and scenario
            float target_speed = 0.0f;     // Target speed to reach goal
            bool is_smooth = true;         // Whether to enforce a smooth speed profile (only when close to goal)
            if (curr_distance_ <= min_distance_)
            {
                // Scenario 1: Goal position is too close - don't move at all
                target_speed = 0.0f;
                is_smooth = true;
            }
            else if (curr_distance_ >= max_distance_)
            {
                // Scenario 2: Goal position is too far - move at max speed
                target_speed = max_speed_;
                is_smooth = false;
            }
            else
            {
                // Scenario 3: Goal position is at an intermediate distance
                // Scale speed linearly based on distance to maintain stability for camera tracking
                float distance_ratio = (curr_distance_ - min_distance_) / (max_distance_ - min_distance_);
                target_speed = min_speed_ + distance_ratio * (max_speed_ - min_speed_);
                is_smooth = true;
            }

            // Enforce trapezoidal speed profile to ensure smooth transitions
            curr_speed_ = enforceTrapezoidal(curr_speed_, target_speed, max_acceleration_, is_smooth, dt);

            return curr_speed_;
        }

    private: // Implementation
        float enforceTrapezoidal(const float& curr_speed, const float& target_speed, const float& max_acceleration, const bool& is_smooth, const float& dt)
        {
            // If smooth profile is required, enforce a lower acceleration limit
            float acceleration_limit = is_smooth ? max_acceleration_ / 4.0f : max_acceleration_;

            // Compute acceleration and enforce limits
            float acceleration = (target_speed - curr_speed) / dt;
            acceleration = std::clamp(acceleration, -acceleration_limit, acceleration_limit);

            // Apply acceleration to get new speed
            float speed_change = acceleration * dt;
            float new_speed = curr_speed + speed_change;

            // Enforce speed limits
            new_speed = std::clamp(new_speed, min_speed_, max_speed_);

            return new_speed;
        }
    };

} // namespace flychams::control