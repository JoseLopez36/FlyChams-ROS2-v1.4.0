#pragma once

// Core includes
#include "flychams_core/types/core_types.hpp"
#include "flychams_core/types/ros_types.hpp"
#include "flychams_core/utils/camera_utils.hpp"

namespace flychams::core
{
    /**
     * ════════════════════════════════════════════════════════════════
     * @brief Message converters between ROS messages and core types
     *
     * @details
     * This class contains all the message converters between ROS messages and core types.
     * ════════════════════════════════════════════════════════════════
     */
    class MsgConversions
    {
    public:
        // ════════════════════════════════════════════════════════════════
        // ROS To Core Conversions
        // ════════════════════════════════════════════════════════════════

        static Vector3r fromMsg(const PointMsg& point)
        {
            return Vector3r{ point.x, point.y, point.z };
        }

        static Vector3r fromMsg(const Vector3Msg& vector)
        {
            return Vector3r{ vector.x, vector.y, vector.z };
        }

        static Quaternionr fromMsg(const QuaternionMsg& quat)
        {
            return Quaternionr{ quat.x, quat.y, quat.z, quat.w };
        }

        static Pose fromMsg(const PoseMsg& pose)
        {
            return Pose{
                fromMsg(pose.position),
                fromMsg(pose.orientation)
            };
        }

        static Twist fromMsg(const TwistMsg& twist)
        {
            return Twist{
                fromMsg(twist.linear),
                fromMsg(twist.angular)
            };
        }

        static Odometry fromMsg(const OdometryMsg& odom)
        {
            return Odometry{
                fromMsg(odom.pose.pose),
                fromMsg(odom.twist.twist)
            };
        }

        static std::pair<Matrix3Xr, RowVectorXr> fromMsg(const AgentInfoMsg& info)
        {
            // Get the number of clusters
            size_t n_clusters = info.centers.size();

            // Create matrices to store cluster centers and radii
            Matrix3Xr centers(3, n_clusters);
            RowVectorXr radii(n_clusters);

            // Fill the matrices with data from the message
            for (size_t i = 0; i < n_clusters; i++)
            {
                // Extract center coordinates
                centers(0, i) = info.centers[i].x;
                centers(1, i) = info.centers[i].y;
                centers(2, i) = info.centers[i].z;

                // Extract radius
                radii(i) = info.radii[i];
            }

            return std::make_pair(centers, radii);
        }

        // ════════════════════════════════════════════════════════════════
        // Core To ROS Conversions
        // ════════════════════════════════════════════════════════════════

        static void toMsg(const Vector3r& vector, PointMsg& point)
        {
            point.x = vector.x();
            point.y = vector.y();
            point.z = vector.z();
        }

        static void toMsg(const Vector3r& vector, Vector3Msg& vec)
        {
            vec.x = vector.x();
            vec.y = vector.y();
            vec.z = vector.z();
        }

        static void toMsg(const Quaternionr& orientation, QuaternionMsg& quat)
        {
            quat.x = orientation.x();
            quat.y = orientation.y();
            quat.z = orientation.z();
            quat.w = orientation.w();
        }

        static void toMsg(const Pose& pose, PoseMsg& ros_pose)
        {
            toMsg(pose.position, ros_pose.position);
            toMsg(pose.orientation, ros_pose.orientation);
        }

        static void toMsg(const Twist& twist, TwistMsg& ros_twist)
        {
            toMsg(twist.linear, ros_twist.linear);
            toMsg(twist.angular, ros_twist.angular);
        }

        static void toMsg(const Vector3r& position, AgentGoalMsg& ros_goal)
        {
            toMsg(position, ros_goal.position);
            ros_goal.yaw = 0.0f;
        }

        static void toMsg(const AgentMetrics& metrics, AgentMetricsMsg& ros_metrics)
        {
            // Agent data
            ros_metrics.curr_x = metrics.curr_x;
            ros_metrics.curr_y = metrics.curr_y;
            ros_metrics.curr_z = metrics.curr_z;
            ros_metrics.curr_yaw = metrics.curr_yaw;
            ros_metrics.vel_x = metrics.vel_x;
            ros_metrics.vel_y = metrics.vel_y;
            ros_metrics.vel_z = metrics.vel_z;
            ros_metrics.vel_yaw = metrics.vel_yaw;
            ros_metrics.goal_x = metrics.goal_x;
            ros_metrics.goal_y = metrics.goal_y;
            ros_metrics.goal_z = metrics.goal_z;
            ros_metrics.goal_yaw = metrics.goal_yaw;

            // Position and movement metrics
            ros_metrics.total_distance_traveled = metrics.total_distance_traveled;
            ros_metrics.current_speed = metrics.current_speed;

            // Goal-related metrics
            ros_metrics.distance_to_goal = metrics.distance_to_goal;

            // Mission metrics
            ros_metrics.time_elapsed = metrics.time_elapsed;

            // Performance metrics
            ros_metrics.average_speed = metrics.average_speed;
        }

        static void toMsg(const TargetMetrics& metrics, TargetMetricsMsg& ros_metrics)
        {
            // Target data
            ros_metrics.curr_x = metrics.curr_x;
            ros_metrics.curr_y = metrics.curr_y;
            ros_metrics.curr_z = metrics.curr_z;

            // Position and movement metrics
            ros_metrics.total_distance_traveled = metrics.total_distance_traveled;
        }

        static void toMsg(const ClusterMetrics& metrics, ClusterMetricsMsg& ros_metrics)
        {
            // Cluster data
            ros_metrics.curr_center_x = metrics.curr_center_x;
            ros_metrics.curr_center_y = metrics.curr_center_y;
            ros_metrics.curr_center_z = metrics.curr_center_z;
            ros_metrics.curr_radius = metrics.curr_radius;

            // Position and movement metrics
            ros_metrics.total_distance_traveled = metrics.total_distance_traveled;
        }

        static void toMsg(const GlobalMetrics& metrics, GlobalMetricsMsg& ros_metrics)
        {
            // Mission metrics
            ros_metrics.mission_time = metrics.mission_time;
        }
    };

} // namespace flychams::core