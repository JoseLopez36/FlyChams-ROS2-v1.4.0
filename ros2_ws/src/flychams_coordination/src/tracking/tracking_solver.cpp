#include "flychams_coordination/tracking/tracking_solver.hpp"

using namespace flychams::core;

namespace flychams::coordination
{
    // ════════════════════════════════════════════════════════════════════════════
    // PUBLIC METHODS: Public methods for configuration and control
    // ════════════════════════════════════════════════════════════════════════════

    void TrackingSolver::reset()
    {
        // Reset tracking data
        data_ = Data();
    }

    std::tuple<float, Vector3r, float> TrackingSolver::runCamera(const Vector3r& z, const float& r, const Matrix4r& T, const HeadParameters& head_params)
    {
        // Args:
        // z: Target position in world frame (m)
        // r: Equivalent radius of the target's area of interest (m)
        // T: Camera pose in world frame
        // head_params: Head parameters

        // Extract camera position and orientation
        const Vector3r x = T.block<3, 1>(0, 3);
        const Matrix3r R = T.block<3, 3>(0, 0);

        // Compute focal length
        const auto [focal, s_proj_pix] = computeCameraFocal(z, r, x, head_params);
        data_.focal = focal;

        // Compute camera orientation
        if (data_.is_first_update)
        {
            data_.rpy = computeCameraOrientation(z, x, R, Vector3r(), true);
            data_.is_first_update = false;
        }
        else
        {
            data_.rpy = computeCameraOrientation(z, x, R, data_.rpy, false);
        }

        // Return focal length, orientation and projected size
        return std::make_tuple(focal, data_.rpy, s_proj_pix);
    }

    std::tuple<Vector2i, Vector2i, float, float> TrackingSolver::runWindow(const Vector3r& z, const float& r, const Matrix4r& T, const HeadParameters& central_params, const WindowParameters& window_params)
    {
        // Args:
        // z: Target position in world frame (m)
        // r: Equivalent radius of the target's area of interest (m)
        // T: Source camera pose in world frame
        // central_params: Central head parameters
        // window_params: Window parameters

        // Extract camera position
        const Vector3r x = T.block<3, 1>(0, 3);

        // Project target position onto source camera
        Vector2r p = MathUtils::projectPoint(z, T, central_params.K);
        p(0) = central_params.width - p(0); // Flip x-axis (TODO: Check why this is needed)

        // Compute window size
        const auto [size, lambda, s_proj_pix] = computeWindowSize(z, r, x, central_params, window_params);

        // Compute window corner
        const Vector2i corner = computeWindowCorner(p, size);

        // Return window size, corner, resolution factor and projected size
        return std::make_tuple(size, corner, lambda, s_proj_pix);
    }

    // ════════════════════════════════════════════════════════════════════════════
    // IMPLEMENTATION: MultiCamera tracking methods
    // ════════════════════════════════════════════════════════════════════════════

    std::pair<float, float> TrackingSolver::computeCameraFocal(const Vector3r& z, const float& r, const Vector3r& x, const HeadParameters& head_params)
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

    Vector3r TrackingSolver::computeCameraOrientation(const Vector3r& z, const Vector3r& x, const Matrix3r& R, const Vector3r& prev_rpy, const bool& is_first_update)
    {
        // Args:
        // z: Target position in world frame (m)
        // x: Camera position in world frame (m)
        // R: Camera rotation matrix in world frame
        // prev_rpy: Previous orientation in RPY format (rad)
        // is_first_update: Whether it is the first update

        Vector3r rpy = Vector3r::Zero(); // roll, pitch, yaw

        // Determine aiming mode
        const AimingMode mode = is_first_update ? AimingMode::INITIAL : AimingMode::CONTINUOUS;

        // Compute direction vector
        const Vector3r t = z - x;
        const Vector3r v = t.normalized();

        // Handle vertical downward case
        if (1.0f - std::abs(v.z()) < 1e-6f)
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
            return Vector3r(0.0f, pitch1, yaw1);

        case AimingMode::CONTINUOUS:
        {
            return calculateCameraContinuousSolution(yaw1, pitch1, yaw2, pitch2, prev_rpy);
        }

        default:
            throw std::invalid_argument("Invalid aiming mode");
        }
    }

    std::pair<float, float> TrackingSolver::calculateCameraBaseSolution(const Vector3r& v)
    {
        return {
            std::atan2(v.y(), v.x()),     // Yaw (Z-axis rotation)
            -std::asin(v.z())             // Pitch (Y-axis rotation)
        };
    }

    std::pair<float, float> TrackingSolver::calculateCameraInvertedSolution(const Vector3r& v)
    {
        const auto& base = calculateCameraBaseSolution(v);
        return {
            base.first + M_PIf,
            M_PIf - base.second      
        };
    }

    Vector3r TrackingSolver::calculateCameraContinuousSolution(const float& yaw1, const float& pitch1, const float& yaw2, const float& pitch2, const Vector3r& prev_rpy)
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

    // ════════════════════════════════════════════════════════════════════════════
    // IMPLEMENTATION: MultiWindow tracking methods
    // ════════════════════════════════════════════════════════════════════════════

    std::tuple<Vector2i, float, float> TrackingSolver::computeWindowSize(const Vector3r& z, const float& r, const Vector3r& x, const HeadParameters& central_params, const WindowParameters& window_params)
    {
        // Args:
        // z: Target position in world frame (m)
        // r: Equivalent radius of the target's area of interest (m)
        // x: Source camera position in world frame (m)
        // central_params: Central head parameters
        // window_params: Window parameters

        // Extract parameters
        const auto& f = central_params.f_ref;
        const auto& tracking_width = window_params.tracking_width;
        const auto& tracking_height = window_params.tracking_height;
        const auto& lambda_min = window_params.lambda_min;
        const auto& lambda_max = window_params.lambda_max;
        const auto& rho = window_params.rho;
        const auto& s_ref = window_params.s_ref;

        // Compute distance between target and camera
        float d = (x - z).norm();

        // Attempt to adjust the resolution factor to achieve the desired apparent size of the object
        float lambda = (d * s_ref) / (r * f);

        // Clamp the resolution factor within the camera's resolution limits
        lambda = std::max(std::min(lambda, lambda_max), lambda_min);

        // Compute window size using the resolution factor
        Vector2i size(0, 0);
        size(0) = static_cast<int>(std::round(static_cast<float>(tracking_width) / lambda));
        size(1) = static_cast<int>(std::round(static_cast<float>(tracking_height) / lambda));

        // Compute actual projected size after clamping
        float s_proj_pix = (lambda * r * f) / (d * rho);

        // Return window size, resolution factor and projected size
        return std::make_tuple(size, lambda, s_proj_pix);
    }

    Vector2i TrackingSolver::computeWindowCorner(const Vector2r& p, const Vector2i& size)
    {
        // Args:
        // p: Projected point on central camera (pix)
        // size: Window size (pix)

        // Compute window corner to place the crop in the center of the image
        const float x = p(0) - static_cast<float>(size(0)) / 2.0f;
        const float y = p(1) - static_cast<float>(size(1)) / 2.0f;

        return {
            static_cast<int>(std::round(x)),
            static_cast<int>(std::round(y))
        };
    }

} // namespace flychams::coordination