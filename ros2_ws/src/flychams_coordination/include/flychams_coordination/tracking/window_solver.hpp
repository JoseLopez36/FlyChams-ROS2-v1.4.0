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
    class WindowSolver
    {
    public: // Types
        using SharedPtr = std::shared_ptr<WindowSolver>;

    public: // Public methods
        // Configuration
        void reset()
        {
            // Nothing to reset
        }

        // Runtime methods
        std::tuple<core::Crop, float, float> runWindow(const core::Vector3r& z, const float& r, const core::Matrix4r& central_head_T, const core::HeadParameters& central_head_params, const core::WindowParameters& window_params)
        {
            // Args:
            // z: Target position in world frame (m)
            // r: Equivalent radius of the target's area of interest (m)
            // central_head_T: Central head pose in world frame
            // central_head_params: Central head parameters
            // window_params: Window parameters

            // Extract camera position
            const core::Vector3r x = central_head_T.block<3, 1>(0, 3);

            // Project target position onto source camera
            core::Vector2r p = core::MathUtils::projectPoint(z, central_head_T, central_head_params.K);

            // Compute window size
            const auto [size, lambda, s_proj_pix] = computeWindowSize(z, r, x, central_head_params, window_params);

            // Compute window corner
            const core::Vector2i corner = computeWindowCorner(p, size);

            // Check if crop is out of bounds (i.e. if the crop is completely outside the image)
            bool is_out_of_bounds =
                (corner(0) + size(0) <= 0) ||                   // Completely to the left
                (corner(1) + size(1) <= 0) ||                   // Completely above
                (corner(0) >= central_head_params.width) ||     // Completely to the right
                (corner(1) >= central_head_params.height);      // Completely below

            // Make crop struct
            core::Crop crop;
            crop.x = corner.x();
            crop.y = corner.y();
            crop.w = size.x();
            crop.h = size.y();
            crop.is_out_of_bounds = is_out_of_bounds;

            // Return window size, corner, resolution factor and projected size
            return std::make_tuple(crop, lambda, s_proj_pix);
        }

    private: // Implementation
        std::tuple<core::Vector2i, float, float> computeWindowSize(const core::Vector3r& z, const float& r, const core::Vector3r& x, const core::HeadParameters& central_params, const core::WindowParameters& window_params)
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
            core::Vector2i size(0, 0);
            size(0) = static_cast<int>(std::round(static_cast<float>(tracking_width) / lambda));
            size(1) = static_cast<int>(std::round(static_cast<float>(tracking_height) / lambda));

            // Compute actual projected size after clamping
            float s_proj_pix = (lambda * r * f) / (d * rho);

            // Return window size, resolution factor and projected size
            return std::make_tuple(size, lambda, s_proj_pix);
        }

        core::Vector2i computeWindowCorner(const core::Vector2r& p, const core::Vector2i& size)
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
    };

} // namespace flychams::coordination