#pragma once

// Core includes
#include "flychams_core/types/core_types.hpp"
#include "flychams_core/types/ros_types.hpp"

namespace flychams::core
{
    /**
     * ════════════════════════════════════════════════════════════════
     * @brief Camera utilities
     *
     * @details
     * This class contains camera utilities
     * ════════════════════════════════════════════════════════════════
     */
    class CameraUtils
    {
    public:
        static float computeFov(float focal, float sensor_width)
        {
            return 2.0f * std::atan(sensor_width / (2.0f * focal));
        }

        static Vector2r projectPoint(const PointMsg& bP, const TransformMsg& bTc, const CameraInfoMsg& camera_info)
        {
            // Extract camera parameters
            const float& fx = camera_info.k[0];  // Focal length x
            const float& fy = camera_info.k[4];  // Focal length y
            const float& cx = camera_info.k[2];  // Principal point x
            const float& cy = camera_info.k[5];  // Principal point y

            // Convert camera pose (base frame) to a tf2::Transform
            TransformTf bTc_tf;
            tf2::fromMsg(bTc, bTc_tf);
            TransformTf cTb_tf = bTc_tf.inverse();

            // Apply the inverse transform to the input point (in base frame)
            Vector3Tf bP_tf(bP.x, bP.y, bP.z);
            Vector3Tf cP_tf = cTb_tf * bP_tf;

            // Project point on image plane
            float cX = cP_tf.getX();
            float cY = cP_tf.getY();
            float cZ = cP_tf.getZ();

            float u = (fx * cX / cZ) + cx;
            float v = (fy * cY / cZ) + cy;
            return Vector2r(u, v);
        }
    };

} // namespace flychams::core