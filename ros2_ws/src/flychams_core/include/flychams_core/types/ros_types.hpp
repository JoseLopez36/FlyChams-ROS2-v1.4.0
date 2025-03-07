#pragma once

// ROS includes
#include <rclcpp/rclcpp.hpp>

// Standard messages
#include <std_msgs/msg/header.hpp>

// Geometry messages
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>

// Navigation messages
#include <nav_msgs/msg/odometry.hpp>

// Sensor messages
#include <sensor_msgs/msg/camera_info.hpp>

// Visualization messages
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

// TF2 includes
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/utils.h>

// Custom message types
#include "flychams_interfaces/msg/registration.hpp"
#include "flychams_interfaces/msg/agent_goal.hpp"
#include "flychams_interfaces/msg/agent_info.hpp"
#include "flychams_interfaces/msg/tracking_goal.hpp"
#include "flychams_interfaces/msg/target_info.hpp"
#include "flychams_interfaces/msg/cluster_info.hpp"
#include "flychams_interfaces/msg/agent_metrics.hpp"
#include "flychams_interfaces/msg/target_metrics.hpp"
#include "flychams_interfaces/msg/cluster_metrics.hpp"
#include "flychams_interfaces/msg/global_metrics.hpp"
#include "flychams_interfaces/msg/crop.hpp"

// Airsim messages
#include "airsim_interfaces/msg/camera_info_array.hpp"

namespace flychams::core
{
    /**
     * ════════════════════════════════════════════════════════════════
     * @brief ROS types
     *
     * @details
     * This file contains all the ROS types used throughout the project,
     * including ROS message types, TF2 types, and custom message types.
     * ════════════════════════════════════════════════════════════════
     */

    // ════════════════════════════════════════════════════════════════
    // BASIC ROS TYPES: Basic ROS types
    // ════════════════════════════════════════════════════════════════

    // Node pointer
    using NodePtr = rclcpp::Node::SharedPtr;
    // Executor
    using ExecutorPtr = rclcpp::executors::MultiThreadedExecutor::SharedPtr;
    // TF2
    using BufferPtr = std::shared_ptr<tf2_ros::Buffer>;
    using ListenerPtr = std::shared_ptr<tf2_ros::TransformListener>;
    // Time
    using Time = rclcpp::Time;
    using TimerPtr = rclcpp::TimerBase::SharedPtr;
    // Publisher
    template<typename T>
    using PublisherPtr = typename rclcpp::Publisher<T>::SharedPtr;
    // Subscriber
    template<typename T>
    using SubscriberPtr = typename rclcpp::Subscription<T>::SharedPtr;
    // Service
    template<typename T>
    using ServicePtr = typename rclcpp::Service<T>::SharedPtr;
    // Client
    template<typename T>
    using ClientPtr = typename rclcpp::Client<T>::SharedPtr;

    // ════════════════════════════════════════════════════════════════
    // MESSAGE TYPES: ROS message types
    // ════════════════════════════════════════════════════════════════

    // Header
    using HeaderMsg = std_msgs::msg::Header;
    // Pose and twist
    using PointMsg = geometry_msgs::msg::Point;
    using PointStampedMsg = geometry_msgs::msg::PointStamped;
    using QuaternionMsg = geometry_msgs::msg::Quaternion;
    using PoseMsg = geometry_msgs::msg::Pose;
    using PoseStampedMsg = geometry_msgs::msg::PoseStamped;
    using TwistMsg = geometry_msgs::msg::Twist;
    using TwistStampedMsg = geometry_msgs::msg::TwistStamped;
    // Transform
    using Vector3Msg = geometry_msgs::msg::Vector3;
    using Vector3StampedMsg = geometry_msgs::msg::Vector3Stamped;
    using TransformMsg = geometry_msgs::msg::Transform;
    using TransformStampedMsg = geometry_msgs::msg::TransformStamped;
    // Odometry
    using OdometryMsg = nav_msgs::msg::Odometry;
    // Camera info
    using CameraInfoMsg = sensor_msgs::msg::CameraInfo;
    // Marker
    using MarkerMsg = visualization_msgs::msg::Marker;
    using MarkerArrayMsg = visualization_msgs::msg::MarkerArray;

    // ════════════════════════════════════════════════════════════════
    // TF2 TYPES: Transform types
    // ════════════════════════════════════════════════════════════════

    // Transform
    using TransformTf = tf2::Transform;
    // Vector3
    using Vector3Tf = tf2::Vector3;

    // ════════════════════════════════════════════════════════════════
    // CUSTOM MESSAGE TYPES: FlyChams-specific message types
    // ════════════════════════════════════════════════════════════════

    // Registration messages
    using RegistrationMsg = flychams_interfaces::msg::Registration;
    // Agent messages
    using AgentGoalMsg = flychams_interfaces::msg::AgentGoal;
    using AgentInfoMsg = flychams_interfaces::msg::AgentInfo;
    using TrackingGoalMsg = flychams_interfaces::msg::TrackingGoal;
    // Target messages
    using TargetInfoMsg = flychams_interfaces::msg::TargetInfo;
    // Cluster messages
    using ClusterInfoMsg = flychams_interfaces::msg::ClusterInfo;
    // Metrics messages
    using AgentMetricsMsg = flychams_interfaces::msg::AgentMetrics;
    using TargetMetricsMsg = flychams_interfaces::msg::TargetMetrics;
    using ClusterMetricsMsg = flychams_interfaces::msg::ClusterMetrics;
    using GlobalMetricsMsg = flychams_interfaces::msg::GlobalMetrics;
    // Basic
    using CropMsg = flychams_interfaces::msg::Crop;

    // ════════════════════════════════════════════════════════════════
    // AIRSIM MESSAGE TYPES: Airsim-specific message types
    // ════════════════════════════════════════════════════════════════

    // Camera info array
    using CameraInfoArrayMsg = airsim_interfaces::msg::CameraInfoArray;

} // namespace flychams::core