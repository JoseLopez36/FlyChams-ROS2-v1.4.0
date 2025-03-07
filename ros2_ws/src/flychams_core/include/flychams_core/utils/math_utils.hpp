#pragma once

// Standard includes
#include <cmath>
#include <vector>

// Core includes
#include "flychams_core/types/core_types.hpp"

namespace flychams::core
{
    class MathUtils
    {
    public:
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

        static Matrix3r quaternionToRotationMatrix(const Quaternionr& ori)
        {
            return ori.toRotationMatrix();
        }

        static Vector3r quaternionToEuler(const Quaternionr& ori)
        {
            return ori.toRotationMatrix().eulerAngles(0, 1, 2);
        }

        static Quaternionr eulerToQuaternion(const Vector3r& euler)
        {
            Eigen::AngleAxisf rollAngle(euler.x(), Vector3r::UnitX());
            Eigen::AngleAxisf pitchAngle(euler.y(), Vector3r::UnitY());
            Eigen::AngleAxisf yawAngle(euler.z(), Vector3r::UnitZ());

            return yawAngle * pitchAngle * rollAngle;
        }

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
    };

}  // namespace flychams::core