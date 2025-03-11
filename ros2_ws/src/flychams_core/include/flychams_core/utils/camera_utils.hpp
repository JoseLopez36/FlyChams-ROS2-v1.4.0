#pragma once

// OpenCV includes
#include <opencv2/opencv.hpp>

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
            return 2.0f * std::atan((sensor_width / 2.0f) / focal);
        }

        static Matrix2Xr projectPoints(const Matrix3Xr& bP, const Matrix4r& bTc, const Matrix3r& k_ref)
        {
            // Get number of points
            const int n = static_cast<int>(bP.cols());

            // Pre-compute elements
            const Matrix4r cTb = bTc.inverse();
            const cv::Mat K = (cv::Mat_<float>(3, 3) << k_ref(0, 0), 0.0f, k_ref(0, 2), 0.0f, k_ref(1, 1), k_ref(1, 2), 0.0f, 0.0f, 1.0f);
            const cv::Mat dist_coeffs = cv::Mat::zeros(5, 1, CV_64F); // Assuming no distortion

            // Iterate through all points and convert to OpenCV format
            std::vector<cv::Point3f> P(n);
            for (int i = 0; i < n; i++)
            {
                // Transform point from base frame to camera frame
                const Vector3r bPi = bP.col(i);
                const Vector4r bPi_(bPi.x(), bPi.y(), bPi.z(), 1.0f);
                const Vector4r cPi_ = cTb * bPi_;

                // Convert
                P[i] = cv::Point3f(cPi_.x(), cPi_.y(), cPi_.z());
            }

            // Project points using OpenCV
            std::vector<cv::Point2f> p(n);
            cv::projectPoints(P, cv::Mat::zeros(3, 1, CV_64F), cv::Mat::zeros(3, 1, CV_64F), K, dist_coeffs, p);

            // Return projected points as matrix
            Matrix2Xr cP(2, n);
            for (int i = 0; i < n; i++)
            {
                const auto& pi = p[i];
                cP.col(i) << pi.x, pi.y;
            }
            return cP;
        }
    };

} // namespace flychams::core