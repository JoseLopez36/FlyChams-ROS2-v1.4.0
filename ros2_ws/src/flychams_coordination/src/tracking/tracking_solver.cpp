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

    std::pair<float, Vector3r> TrackingSolver::runCamera(const Vector3r& z, const float& r, const Matrix4r& T, const CameraParameters& camera_params, const ProjectionParameters& projection_params, float& s_proj_pix)
    {
        // Args:
        // z: Target position in world frame (m)
        // r: Equivalent radius of the target's area of interest (m)
        // T: Camera pose in world frame
        // camera_params: Camera parameters
        // projection_params: Projection parameters

        // Extract camera position and orientation
        const Vector3r x = T.block<3, 1>(0, 3);
        const Matrix3r R = T.block<3, 3>(0, 0);

        // Compute focal length
        data_.focal = computeCameraFocal(z, r, x, camera_params, projection_params, s_proj_pix);

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

        // Return focal length and orientation
        return std::make_pair(data_.focal, data_.rpy);
    }

    std::pair<Vector2i, Vector2i> TrackingSolver::runWindow(const Vector3r& z, const float& r, const Matrix4r& T, const CameraParameters& central_params, const WindowParameters& window_params, const ProjectionParameters& projection_params, float& lambda, float& s_proj_pix)
    {
        // Args:
        // z: Target position in world frame (m)
        // r: Equivalent radius of the target's area of interest (m)
        // T: Source camera pose in world frame
        // window_params: Window parameters
        // projection_params: Projection parameters

        // Extract camera position
        const Vector3r x = T.block<3, 1>(0, 3);

        // Project target position onto source camera
        Vector2r p = MathUtils::projectPoint(z, T, central_params.K);
        p(0) = central_params.width - p(0); // Flip x-axis (TODO: Check why this is needed)

        // Compute window size
        const Vector2i size = computeWindowSize(z, r, x, central_params, window_params, projection_params, lambda, s_proj_pix);

        // Compute window corner
        const Vector2i corner = computeWindowCorner(p, size);

        // Return window size and corner
        return std::make_pair(size, corner);
    }

    // ════════════════════════════════════════════════════════════════════════════
    // IMPLEMENTATION: MultiCamera tracking methods
    // ════════════════════════════════════════════════════════════════════════════

    float TrackingSolver::computeCameraFocal(const Vector3r& z, const float& r, const Vector3r& x, const CameraParameters& camera_params, const ProjectionParameters& projection_params, float& s_proj_pix)
    {
        // Args:
        // z: Target position in world frame (m)
        // r: Equivalent radius of the target's area of interest (m)
        // x: Camera position in world frame (m)
        // camera_params: Camera parameters
        // projection_params: Projection parameters
        // s_proj_pix: Projected size (pix)

        // Extract parameters
        const auto& f_min = camera_params.f_min;
        const auto& f_max = camera_params.f_max;
        const auto& rho = camera_params.rho;
        const auto& s_ref = projection_params.s_ref;

        // Compute distance between target and camera
        float d = (x - z).norm();

        // Attempt to adjust the focal length to achieve the desired apparent size of the object
        float f = (d / r) * s_ref;

        // Clamp the focal length within the camera's focal limits
        f = std::max(std::min(f, f_max), f_min);

        // Compute actual projected size after clamping
        s_proj_pix = (r * f) / (d * rho);

        return f;
    }

    Vector3r TrackingSolver::computeCameraOrientation(const Vector3r& z, const Vector3r& x, const Matrix3r& R, const Vector3r& prev_rpy, const bool& is_first_update)
    {
        // Args:
        // z: Target position in world frame (m)
        // x: Camera position in world frame (m)
        // R: Camera rotation matrix in world frame
        // prev_rpy: Previous orientation in RPY format (rad)
        // is_first_update: Whether it is the first update

        // Transform target to camera frame
        const Vector3r cPt = R.transpose() * (z - x);

        // Determine aiming mode
        const AimingMode mode = is_first_update ? AimingMode::INITIAL : AimingMode::CONTINUOUS;

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
        const auto [yaw1, pitch1] = calculateCameraBaseSolution(dir);
        const auto [yaw2, pitch2] = calculateCameraInvertedSolution(dir);

        // Select solution based on mode
        switch (mode)
        {
        case AimingMode::INITIAL:
            return Vector3r(0.0f, pitch1, yaw1);

        case AimingMode::CONTINUOUS:
            return calculateCameraContinuousSolution(yaw1, pitch1, yaw2, pitch2, prev_rpy);

        default:
            throw std::invalid_argument("Invalid aiming mode");
        }
    }

    std::pair<float, float> TrackingSolver::calculateCameraBaseSolution(const Vector3r& dir)
    {
        return {
            std::atan2(dir.y(), dir.x()),           // Yaw (Z-axis rotation)
            std::acos(dir.z())                      // Pitch (Y-axis rotation)
        };
    }

    std::pair<float, float> TrackingSolver::calculateCameraInvertedSolution(const Vector3r& dir)
    {
        return {
            std::atan2(-dir.y(), -dir.x()),         // Inverted yaw
            -std::acos(dir.z())                     // Negative pitch
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

    Vector2i TrackingSolver::computeWindowSize(const Vector3r& z, const float& r, const Vector3r& x, const CameraParameters& central_params, const WindowParameters& window_params, const ProjectionParameters& projection_params, float& lambda, float& s_proj_pix)
    {
        // Args:
        // z: Target position in world frame (m)
        // r: Equivalent radius of the target's area of interest (m)
        // x: Source camera position in world frame (m)
        // window_params: Window parameters
        // lambda: Resolution factor (0-1)
        // s_proj_pix: Projected size (pix)
        // projection_params: Projection parameters

        // Extract parameters
        const auto& f = central_params.f_ref;
        const auto& tracking_width = window_params.tracking_width;
        const auto& tracking_height = window_params.tracking_height;
        const auto& lambda_min = window_params.lambda_min;
        const auto& lambda_max = window_params.lambda_max;
        const auto& rho = window_params.rho;
        const auto& s_ref = projection_params.s_ref;

        // Compute distance between target and camera
        float d = (x - z).norm();

        // Attempt to adjust the resolution factor to achieve the desired apparent size of the object
        lambda = (d * s_ref) / (r * f);

        // Clamp the resolution factor within the camera's resolution limits
        lambda = std::max(std::min(lambda, lambda_max), lambda_min);

        // Compute window size using the resolution factor
        const float width = static_cast<float>(tracking_width) / lambda;
        const float height = static_cast<float>(tracking_height) / lambda;

        // Compute actual projected size after clamping
        s_proj_pix = (lambda * r * f) / (d * rho);

        return {
            static_cast<int>(std::round(width)),
            static_cast<int>(std::round(height))
        };
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