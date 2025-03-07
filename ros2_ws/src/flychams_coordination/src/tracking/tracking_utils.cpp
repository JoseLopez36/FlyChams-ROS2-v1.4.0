#include "flychams_coordination/tracking/tracking_utils.hpp"

using namespace flychams::core;

namespace flychams::coordination
{
    // ════════════════════════════════════════════════════════════════════════════
    // PUBLIC: MultiCameraTracking methods
    // ════════════════════════════════════════════════════════════════════════════

    float TrackingUtils::computeFocal(const core::Vector3r& wPt, const float& r, const core::Vector3r& wPc, const core::CameraParameters& camera_params, const core::ProjectionParameters& projection_params)
    {
        // wPt: Target position in world frame (m)
        // r: Equivalent radius of the target's area of interest (m)
        // wPc: Camera position in world frame (m)
        // camera_params: Camera parameters
        // projection_params: Projection parameters

        // Extract parameters
        const auto& f_min = camera_params.f_min;
        const auto& f_max = camera_params.f_max;
        const auto& s_ref = projection_params.s_ref;

        // Compute distance between target and camera
        float d = (wPt - wPc).norm();

        // Attempt to adjust the focal length to achieve the desired apparent size of the object
        float f = (d / r) * s_ref;

        // Clamp the focal length within the camera's focal limits
        return std::max(std::min(f, f_max), f_min);
    }

    Vector3r TrackingUtils::computeOrientation(const Vector3r& wPt, const Vector3r& wPc, const Matrix3r& wRc, const Vector3r& prev_rpy, const bool& is_first_update)
    {
        // wPt: Target position in world frame (m)
        // wPc: Camera position in world frame (m)
        // wRc: Camera rotation in world frame 
        // prev_rpy: Previous orientation in RPY format (rad)
        // is_first_update: Whether it is the first update

        // Transform target to camera frame
        const Vector3r cPt = wRc.transpose() * (wPt - wPc);

        // Determine aiming mode
        const auto mode = is_first_update ? AimingMode::INITIAL : AimingMode::CONTINUOUS;

        // Compute tracking angles
        Vector3r rpy = Vector3r::Zero(); // roll, pitch, yaw
        // If it is the first iteration, we don't have a previous reference angle to establish
        // aiming mode CONTINUOUS (continuity in movement). We opt for mode INITIAL (non-inverted image).
        // Normalize target direction vector
        Vector3r dir = cPt.normalized();

        // Handle vertical downward case
        if (1.0f - std::abs(dir.z()) < 1e-6f)
        {
            // In CONTINUOUS mode, take the previous yaw
            rpy(2) = (mode == AimingMode::CONTINUOUS) ? prev_rpy(2) : 0.0f;
            return rpy;
        }

        // Calculate base solutions
        const auto [yaw1, pitch1] = calculateBaseSolution(dir);
        const auto [yaw2, pitch2] = calculateInvertedSolution(dir);

        // Select solution based on mode
        switch (mode)
        {
        case AimingMode::INITIAL:
            return Vector3r(0.0f, pitch1, yaw1);

        case AimingMode::CONTINUOUS:
            return calculateContinuousSolution(yaw1, pitch1, yaw2, pitch2, prev_rpy);

        default:
            throw std::invalid_argument("Invalid aiming mode");
        }
    }

    // ════════════════════════════════════════════════════════════════════════════
    // PUBLIC: MultiWindowTracking methods
    // ════════════════════════════════════════════════════════════════════════════

    Crop TrackingUtils::computeWindowCrop(const core::Vector3r& wPt, const float& r, const core::Vector3r& wPc, const core::Vector2r& p, const core::WindowParameters& window_params, const core::ProjectionParameters& projection_params)
    {
        // wPt: Target position in world frame (m)
        // r: Equivalent radius of the target's area of interest (m)
        // wPc: Camera position in world frame (m)
        // p: Target projection in the camera (pix)
        // window_params: Window parameters
        // projection_params: Projection parameters

        // Extract parameters
        const auto& f = window_params.camera_params.f_ref;
        const auto& rho = window_params.camera_params.rho;
        const auto& full_width = window_params.full_width;
        const auto& full_height = window_params.full_height;
        const auto& lambda_min = window_params.lambda_min;
        const auto& lambda_max = window_params.lambda_max;
        const auto& s_ref = projection_params.s_ref;

        // Compute distance between target and camera
        float d = (wPt - wPc).norm();

        // Attempt to adjust the resolution factor to achieve the desired apparent size of the object
        float lambda = (d * s_ref * rho) / (r * f);

        // Clamp the resolution factor within the camera's resolution limits
        lambda = std::max(std::min(lambda, lambda_max), lambda_min);

        // Compute crop parameters
        const float width = lambda * static_cast<float>(full_width);
        const float height = lambda * static_cast<float>(full_height);
        const float x = p(0) - width / 2.0f;
        const float y = p(1) - height / 2.0f;

        return {
            static_cast<int>(std::round(x)),
            static_cast<int>(std::round(y)),
            static_cast<int>(std::round(width)),
            static_cast<int>(std::round(height))
        };
    }

    // ════════════════════════════════════════════════════════════════════════════
    // IMPLEMENTATION: MultiCameraTracking
    // ════════════════════════════════════════════════════════════════════════════

    std::pair<float, float> TrackingUtils::calculateBaseSolution(const Vector3r& dir)
    {
        return {
            std::atan2(dir.y(), dir.x()),           // Yaw (Z-axis rotation)
            std::acos(dir.z())                      // Pitch (Y-axis rotation)
        };
    }

    std::pair<float, float> TrackingUtils::calculateInvertedSolution(const Vector3r& dir)
    {
        return {
            std::atan2(-dir.y(), -dir.x()),         // Inverted yaw
            -std::acos(dir.z())                     // Negative pitch
        };
    }

    Vector3r TrackingUtils::calculateContinuousSolution(const float& yaw1, const float& pitch1, const float& yaw2, const float& pitch2, const Vector3r& prev_rpy)
    {
        // Normalize angles to [-pi, pi]
        const float prev_yaw = MathUtils::normalizeAngle(prev_rpy(2));
        const float norm_yaw1 = MathUtils::normalizeAngle(yaw1);
        const float norm_yaw2 = MathUtils::normalizeAngle(yaw2);

        // Calculate angular distances
        const float dist1 = std::abs(MathUtils::normalizeAngle(norm_yaw1 - prev_yaw));
        const float dist2 = std::abs(MathUtils::normalizeAngle(norm_yaw2 - prev_yaw));

        // Choose closest yaw solution
        return (dist1 <= dist2) ? Vector3r(0.0f, pitch1, norm_yaw1) : Vector3r(0.0f, pitch2, norm_yaw2);
    }

} // namespace flychams::coordination
