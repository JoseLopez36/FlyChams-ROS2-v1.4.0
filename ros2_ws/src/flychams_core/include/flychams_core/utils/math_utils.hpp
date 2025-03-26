#pragma once

// Standard includes
#include <cmath>
#include <vector>

// OpenCV includes
#include <opencv2/opencv.hpp>

// Core includes
#include "flychams_core/types/core_types.hpp"

namespace flychams::core
{
    class MathUtils
    {
    public:
        // ════════════════════════════════════════════════════════════════════════════
        // ANGLE: Angle utilities
        // ════════════════════════════════════════════════════════════════════════════

        static float degToRad(float degrees)
        {
            return degrees * M_PIf / 180.0f;
        }

        static float radToDeg(float radians)
        {
            return radians * 180.0f / M_PIf;
        }

        static float normalizeAngle(float angle)
        {
            // Normalize angle to [-π, π]
            return std::atan2(std::sin(angle), std::cos(angle));
        }

        // ════════════════════════════════════════════════════════════════════════════
        // VECTOR: Vector utilities
        // ════════════════════════════════════════════════════════════════════════════

        static float distance(const Vector3r& p1, const Vector3r& p2)
        {
            return (p1 - p2).norm();
        }

        static Vector3r direction(const Vector3r& from, const Vector3r& to)
        {
            Vector3r dir = to - from;
            if (dir.norm() < 1e-6) {
                return Vector3r::Zero();
            }
            return dir.normalized();
        }

        // ════════════════════════════════════════════════════════════════════════════
        // GEOMETRY: Geometry utilities
        // ════════════════════════════════════════════════════════════════════════════

        static Vector3r quaternionToEuler(const Quaternionr& ori)
        {
            return ori.toRotationMatrix().eulerAngles(0, 1, 2);
        }

        static Matrix3r quaternionToRotationMatrix(const Quaternionr& ori)
        {
            return ori.toRotationMatrix();
        }

        static Quaternionr eulerToQuaternion(const Vector3r& euler)
        {
            Eigen::AngleAxisf rollAngle(euler.x(), Vector3r::UnitX());
            Eigen::AngleAxisf pitchAngle(euler.y(), Vector3r::UnitY());
            Eigen::AngleAxisf yawAngle(euler.z(), Vector3r::UnitZ());

            return yawAngle * pitchAngle * rollAngle;
        }

        // ════════════════════════════════════════════════════════════════════════════
        // CAMERA: Camera utilities
        // ════════════════════════════════════════════════════════════════════════════

        static float computeFov(float focal, float sensor_width)
        {
            return 2.0f * std::atan((sensor_width / 2.0f) / focal);
        }

        static Vector2r projectPoint(const Vector3r& wP, const Matrix4r& wTc, const Matrix3r& K)
        {
            // Args:
            // - wP: World point
            // - wTc: World to camera transform
            // - K: Camera intrinsic matrix

            // Get transform from camera to world
            const Matrix4r cTw = wTc.inverse();

            // Get camera parameters in OpenCV format
            const cv::Mat K_cv = (cv::Mat_<float>(3, 3) << K(0, 0), K(0, 1), K(0, 2), K(1, 0), K(1, 1), K(1, 2), K(2, 0), K(2, 1), K(2, 2));
            const cv::Mat D_cv = cv::Mat::zeros(5, 1, CV_64F); // Assuming no distortion

            // Transform point to camera frame
            const Vector4r wP_(wP.x(), wP.y(), wP.z(), 1.0f);
            const Vector4r cP_ = cTw * wP_;

            // Project point using OpenCV
            std::vector<cv::Point3f> cP_cv(1);
            cP_cv[0] = cv::Point3f(cP_.x(), cP_.y(), cP_.z());
            std::vector<cv::Point2f> p(1);
            cv::projectPoints(cP_cv, cv::Mat::zeros(3, 1, CV_64F), cv::Mat::zeros(3, 1, CV_64F), K_cv, D_cv, p);

            // Return projected point
            return Vector2r(p[0].x, p[0].y);
        }

        static Matrix2Xr projectPoints(const Matrix3Xr& tab_wP, const Matrix4r& wTc, const Matrix3r& K)
        {
            // Args:
            // - tab_wP: World points
            // - wTc: World to camera transform
            // - K: Camera intrinsic matrix

            // Get transform from camera to world
            const Matrix4r cTw = wTc.inverse();

            // Get camera parameters in OpenCV format
            const cv::Mat K_cv = (cv::Mat_<float>(3, 3) << K(0, 0), K(0, 1), K(0, 2), K(1, 0), K(1, 1), K(1, 2), K(2, 0), K(2, 1), K(2, 2));
            const cv::Mat D_cv = cv::Mat::zeros(5, 1, CV_64F); // Assuming no distortion

            // Transform points to camera frame
            int n = tab_wP.cols();
            std::vector<cv::Point3f> tab_cP_cv(n);
            for (int i = 0; i < n; i++)
            {
                const Vector3r& wP = tab_wP.col(i);
                const Vector4r wP_(wP.x(), wP.y(), wP.z(), 1.0f);
                const Vector4r cP_ = cTw * wP_;

                // Add to vector
                tab_cP_cv[i] = cv::Point3f(cP_.x(), cP_.y(), cP_.z());
            }

            // Project points using OpenCV
            std::vector<cv::Point2f> tab_p_cv(n);
            cv::projectPoints(tab_cP_cv, cv::Mat::zeros(3, 1, CV_64F), cv::Mat::zeros(3, 1, CV_64F), K_cv, D_cv, tab_p_cv);

            // Return projected points
            Matrix2Xr tab_p(2, n);
            for (int i = 0; i < n; i++)
            {
                const auto& pi = tab_p_cv[i];
                tab_p.col(i) << pi.x, pi.y;
            }

            return tab_p;
        }
    };

}  // namespace flychams::core