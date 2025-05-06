#pragma once

// Utilities
#include "flychams_core/types/core_types.hpp"
#include "flychams_core/utils/math_utils.hpp"

namespace flychams::coordination
{
    /**
     * ════════════════════════════════════════════════════════════════
     * @brief Solver for agent tracking. Specific for head tracking
     * ════════════════════════════════════════════════════════════════
     * @author Jose Francisco Lopez Ruiz
     * @date 2025-04-28
     * ════════════════════════════════════════════════════════════════
     */
    class HeadSolver
    {
    public: // Types
        using SharedPtr = std::shared_ptr<HeadSolver>;
        // Modes
        enum class AimingMode
        {
            INITIAL,
            CONTINUOUS
        };

    private: // Data
        float focal_prev_;
        core::Vector3r rpy_prev_;
        bool is_first_update_;

    public: // Public methods
        // Configuration
        void reset()
        {
            // Reset tracking data
            focal_prev_ = 0.0f;
            rpy_prev_.setZero();
            is_first_update_ = true;
        }

        // Runtime methods
        std::tuple<float, core::Vector3r, float> runCamera(const core::Vector3r& z, const float& r, const core::Matrix4r& T, const core::HeadParameters& head_params)
        {
            // Args:
            // z: Target position in world frame (m)
            // r: Equivalent radius of the target's area of interest (m)
            // T: Ci_ in world frame (auxiliary frame)
            // head_params: Head parameters

            // Extract camera position and orientation
            const core::Vector3r x = T.block<3, 1>(0, 3);
            const core::Matrix3r R = T.block<3, 3>(0, 0);

            // Compute focal length
            const auto [focal, s_proj_pix] = computeCameraFocal(z, r, x, head_params);

            // Update previous focal
            focal_prev_ = focal;

            // Compute camera orientation
            core::Vector3r rpy;
            if (is_first_update_)
            {
                rpy = computeCameraOrientation(z, x, R, core::Vector3r(), true);
                is_first_update_ = false;
            }
            else
            {
                rpy = computeCameraOrientation(z, x, R, rpy_prev_, false);
            }

            // Update previous orientation
            rpy_prev_ = rpy;

            // Return focal length, orientation and projected size
            return std::make_tuple(focal, rpy, s_proj_pix);
        }

    private: // Implementation
        std::pair<float, float> computeCameraFocal(const core::Vector3r& z, const float& r, const core::Vector3r& x, const core::HeadParameters& head_params)
        {
            // Args:
            // z: Target position in world frame (m)
            // r: Equivalent radius of the target's area of interest (m)
            // x: Camera position in world frame (m)
            // head_params: Head parameters

            // Extract parameters
            const auto& f_min = head_params.f_min;
            const auto& f_max = head_params.f_max;
            const auto& rho = head_params.rho;
            const auto& s_ref = head_params.s_ref;

            // Compute distance between target and camera
            float d = (x - z).norm();

            // Attempt to adjust the focal length to achieve the desired apparent size of the object
            float f = (d / r) * s_ref;

            // Clamp the focal length within the camera's focal limits
            f = std::max(std::min(f, f_max), f_min);

            // Compute actual projected size after clamping
            float s_proj_pix = (r * f) / (d * rho);

            // Return focal length and projected size
            return std::make_pair(f, s_proj_pix);
        }

        core::Vector3r computeCameraOrientation(const core::Vector3r& z, const core::Vector3r& x, const core::Matrix3r& wRc, const core::Vector3r& prev_rpy, const bool& is_first_update)
        {
            // Args:
            // z: Target position in world frame (m)
            // x: Camera position in world frame (m)
            // R: Camera rotation matrix in world frame
            // prev_rpy: Previous orientation in RPY format (rad)
            // is_first_update: Whether it is the first update

            core::Vector3r rpy = core::Vector3r::Zero(); // roll, pitch, yaw

            // Determine aiming mode
            const AimingMode mode = is_first_update ? AimingMode::INITIAL : AimingMode::CONTINUOUS;

            // Compute direction vector
            const core::Vector3r t = z - x;
            const core::Vector3r v = t.normalized();

            // Handle vertical downward case
            if (std::abs(v.z() - 1.0f) < 1e-6f)
            {
                // In CONTINUOUS mode, take the previous yaw
                rpy(2) = (mode == AimingMode::CONTINUOUS) ? prev_rpy(2) : 0.0f;
                return rpy;
            }

            // Calculate base solutions
            const auto [yaw1, pitch1] = calculateCameraBaseSolution(v);
            const auto [yaw2, pitch2] = calculateCameraInvertedSolution(v);

            // Select solution based on mode
            // If it is the first iteration, we don't have a previous reference angle to establish
            // aiming mode CONTINUOUS (continuity in movement). We opt for mode INITIAL (non-inverted image).
            // Normalize target direction vector
            switch (mode)
            {
            case AimingMode::INITIAL:
                return core::Vector3r(0.0f, pitch1, yaw1);

            case AimingMode::CONTINUOUS:
            {
                return calculateCameraContinuousSolution(yaw1, pitch1, yaw2, pitch2, prev_rpy);
            }

            default:
                throw std::invalid_argument("Invalid aiming mode");
            }
        }

        std::pair<float, float> calculateCameraBaseSolution(const core::Vector3r& v)
        {
            return {
                std::atan2(v.x(), -v.y()),    // Yaw (Z-axis rotation)
                std::acos(v.z())              // Pitch (Y-axis rotation)
            };
        }

        std::pair<float, float> calculateCameraInvertedSolution(const core::Vector3r& v)
        {
            return {
                std::atan2(-v.x(), v.y()),    // Yaw (Z-axis rotation)
                -std::acos(v.z())             // Pitch (Y-axis rotation)
            };
        }

        core::Vector3r calculateCameraContinuousSolution(const float& yaw1, const float& pitch1, const float& yaw2, const float& pitch2, const core::Vector3r& prev_rpy)
        {
            // Normalize angles to [-pi, pi]
            const float prev_yaw = core::MathUtils::normalizeAngle(prev_rpy(2));
            const float norm_yaw1 = core::MathUtils::normalizeAngle(yaw1);
            const float norm_yaw2 = core::MathUtils::normalizeAngle(yaw2);

            // Calculate angular distances
            const float dist1 = std::abs(core::MathUtils::normalizeAngle(norm_yaw1 - prev_yaw));
            const float dist2 = std::abs(core::MathUtils::normalizeAngle(norm_yaw2 - prev_yaw));

            // Choose closest yaw solution
            return (dist1 <= dist2) ? core::Vector3r(0.0f, pitch1, norm_yaw1) : core::Vector3r(0.0f, pitch2, norm_yaw2);
        }
    };

} // namespace flychams::coordination