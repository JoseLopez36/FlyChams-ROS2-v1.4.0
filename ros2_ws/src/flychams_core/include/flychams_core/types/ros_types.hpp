#pragma once

// ROS includes
#include <rclcpp/rclcpp.hpp>

// Standard messages
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/string.hpp>

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

// Visualization messages
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

// TF2 includes
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/utils.h>

// Custom message types
// Global messages
#include "flychams_interfaces/msg/registration.hpp"
#include "flychams_interfaces/msg/global_metrics.hpp"
// Agent messages
#include "flychams_interfaces/msg/agent_status.hpp"
#include "flychams_interfaces/msg/agent_assignment.hpp"
#include "flychams_interfaces/msg/agent_clusters.hpp"
#include "flychams_interfaces/msg/agent_tracking_setpoints.hpp"
#include "flychams_interfaces/msg/agent_metrics.hpp"
// Target messages
#include "flychams_interfaces/msg/target_metrics.hpp"
// Cluster messages
#include "flychams_interfaces/msg/cluster_assignment.hpp"
#include "flychams_interfaces/msg/cluster_geometry.hpp"
#include "flychams_interfaces/msg/cluster_metrics.hpp"
// GUI messages
#include "flychams_interfaces/msg/gui_setpoints.hpp"

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
    using BroadcasterPtr = std::shared_ptr<tf2_ros::TransformBroadcaster>;
    using StaticBroadcasterPtr = std::shared_ptr<tf2_ros::StaticTransformBroadcaster>;
    // Time
    using Time = rclcpp::Time;
    using TimerPtr = rclcpp::TimerBase::SharedPtr;
    // Publisher
    template<typename T>
    using PublisherPtr = typename rclcpp::Publisher<T>::SharedPtr;
    // Subscriber
    template<typename T>
    using SubscriberPtr = typename rclcpp::Subscription<T>::SharedPtr;
    // Callback group
    using CallbackGroupPtr = std::shared_ptr<rclcpp::CallbackGroup>;
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
    // Marker
    using MarkerMsg = visualization_msgs::msg::Marker;
    using MarkerArrayMsg = visualization_msgs::msg::MarkerArray;
    // Color
    using ColorMsg = std_msgs::msg::ColorRGBA;
    // String
    using StringMsg = std_msgs::msg::String;

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

    // Basic
    using CropMsg = flychams_interfaces::msg::Crop;
    using ElementMsg = flychams_interfaces::msg::Element;
    // Registration messages
    using RegistrationMsg = flychams_interfaces::msg::Registration;
    using GlobalMetricsMsg = flychams_interfaces::msg::GlobalMetrics;
    // Agent messages
    using AgentStatusMsg = flychams_interfaces::msg::AgentStatus;
    using AgentAssignmentMsg = flychams_interfaces::msg::AgentAssignment;
    using AgentClustersMsg = flychams_interfaces::msg::AgentClusters;
    using AgentTrackingSetpointsMsg = flychams_interfaces::msg::AgentTrackingSetpoints;
    using AgentMetricsMsg = flychams_interfaces::msg::AgentMetrics;
    // Target messages
    using TargetMetricsMsg = flychams_interfaces::msg::TargetMetrics;
    // Cluster messages
    using ClusterAssignmentMsg = flychams_interfaces::msg::ClusterAssignment;
    using ClusterGeometryMsg = flychams_interfaces::msg::ClusterGeometry;
    using ClusterMetricsMsg = flychams_interfaces::msg::ClusterMetrics;
    // GUI messages
    using GuiSetpointsMsg = flychams_interfaces::msg::GuiSetpoints;

} // namespace flychams::core