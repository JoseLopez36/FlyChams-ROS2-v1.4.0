#pragma once

// Standard includes
#include <cmath>
#include <algorithm>

// Core includes
#include "flychams_core/types/core_types.hpp"
#include "flychams_core/types/ros_types.hpp"
#include "flychams_core/utils/msg_conversions.hpp"
#include "flychams_core/utils/math_utils.hpp"
#include "flychams_core/utils/camera_utils.hpp"

namespace flychams::coordination
{
    /**
     * ════════════════════════════════════════════════════════════════
     * @brief Tracking utilities implementation
     *
     * @details
     * This class implements tracking utilities, such as focal length
     * or orientation computation.
     *
     * ════════════════════════════════════════════════════════════════
     * @author Jose Francisco Lopez Ruiz
     * @date 2025-02-27
     * ════════════════════════════════════════════════════════════════
     */
    class TrackingUtils
    {
    private: // Types
        enum class AimingMode
        {
            INITIAL,
            CONTINUOUS
        };

    public: // Public methods
        // MultiCameraTracking
        static float computeFocal(const core::Vector3r& wPt, const float& r, const core::Vector3r& wPc, const core::CameraParameters& camera_params, const core::ProjectionParameters& projection_params);
        static core::Vector3r computeOrientation(const core::Vector3r& wPt, const core::Vector3r& wPc, const core::Matrix3r& wRc, const core::Vector3r& prev_rpy, const bool& is_first_update);
        // MultiWindowTracking
        static core::Vector2i computeWindowSize(const core::Vector3r& wPt, const float& r, const core::Vector3r& wPc, const core::WindowParameters& window_params, const core::ProjectionParameters& projection_params, float& lambda);
        static core::Vector2i computeWindowCorner(const core::Vector2r& p, const core::Vector2i& size);

    private: // Methods
        // MultiCameraTracking
        static std::pair<float, float> calculateBaseSolution(const core::Vector3r& dir);
        static std::pair<float, float> calculateInvertedSolution(const core::Vector3r& dir);
        static core::Vector3r calculateContinuousSolution(const float& yaw1, const float& pitch1, const float& yaw2, const float& pitch2, const core::Vector3r& prev_rpy);
    };

} // namespace flychams::coordination
