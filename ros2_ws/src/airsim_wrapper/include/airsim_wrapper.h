#pragma once

#include "common/common_utils/StrictMode.hpp"
STRICT_MODE_OFF //todo what does this do?
#ifndef RPCLIB_MSGPACK
#define RPCLIB_MSGPACK clmdep_msgpack
#endif // !RPCLIB_MSGPACK
#include "rpc/rpc_error.h"
STRICT_MODE_ON

// Standard includes
#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <mutex>
#include <atomic>

// ROS2 core headers
#include "rclcpp/rclcpp.hpp"

// ROS2 message headers
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/bool.hpp>

// TF2 headers
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

// AirSim headers
#include "airsim_settings_parser.h"
#include "common/AirSimSettings.hpp"
#include "common/common_utils/FileSystem.hpp"
#include "math_common.h"
#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"

// Third party headers
#include "yaml-cpp/yaml.h"

// Vehicle messages
#include <airsim_interfaces/msg/camera_info_array.hpp>

// Status message
#include <airsim_interfaces/msg/status.hpp>

// Global commands
#include <airsim_interfaces/srv/reset.hpp>
#include <airsim_interfaces/srv/run.hpp>
#include <airsim_interfaces/srv/pause.hpp>

// Vehicle commands
#include <airsim_interfaces/srv/takeoff_group.hpp>
#include <airsim_interfaces/srv/land_group.hpp>
#include <airsim_interfaces/srv/hover_group.hpp>
#include <airsim_interfaces/msg/vel_cmd.hpp>

// Camera and gimbal commands
#include <airsim_interfaces/msg/gimbal_angle_cmd.hpp>
#include <airsim_interfaces/msg/camera_fov_cmd.hpp>

// Window commands
#include <airsim_interfaces/msg/window_image_cmd_group.hpp>
#include <airsim_interfaces/msg/window_rectangle_cmd.hpp>
#include <airsim_interfaces/msg/window_string_cmd.hpp>

// Tracking commands
#include <airsim_interfaces/srv/add_target_group.hpp>
#include <airsim_interfaces/srv/add_cluster_group.hpp>
#include <airsim_interfaces/msg/update_target_cmd_group.hpp>
#include <airsim_interfaces/msg/update_cluster_cmd_group.hpp>

namespace airsim_wrapper
{
    /**
     * ════════════════════════════════════════════════════════════════
     * @brief AirSim ROS2 Wrapper Class
     *
     * @details
     * This class provides a ROS2 wrapper for the AirSim simulator.
     * It handles communication between ROS2 nodes and AirSim, managing
     * vehicle control, sensor data, and simulator state.
     *
     * ════════════════════════════════════════════════════════════════
     */
    class AirsimWrapper
    {
    // ════════════════════════════════════════════════════════════════
    // PUBLIC: Type Aliases
    public:
        using AirSimSettings = msr::airlib::AirSimSettings;
        using VehicleSetting = msr::airlib::AirSimSettings::VehicleSetting;
        using CameraSetting = msr::airlib::AirSimSettings::CameraSetting;
        using CaptureSetting = msr::airlib::AirSimSettings::CaptureSetting;

    // ════════════════════════════════════════════════════════════════
    // PUBLIC: Constructors/Destructors
    public:
        AirsimWrapper(const std::shared_ptr<rclcpp::Node> nh, const std::string& host_ip, uint16_t host_port, bool enable_world_plot);
        ~AirsimWrapper();
        void shutdown();

    // ════════════════════════════════════════════════════════════════
    // PUBLIC: Initialization methods
    public:
        void initialize_airsim();
        void initialize_ros();

    // ════════════════════════════════════════════════════════════════
    // PRIVATE: Vehicle Classes
    private:
        /**
         * Class for camera ROS interfaces
         */
        class CameraROS
        {
        public:
            virtual ~CameraROS() = default;
            std::string camera_name;

            // Camera setting
            CameraSetting camera_setting;
            // Transforms
            geometry_msgs::msg::TransformStamped body_tf_msg;
            geometry_msgs::msg::TransformStamped optical_static_tf_msg;
            // Frame IDs
            std::string body_frame_id;
            std::string optical_frame_id;
        };

        /**
         * Class for vehicle ROS interfaces
         */
        class VehicleROS
        {
        public:
            virtual ~VehicleROS() = default;
            std::string vehicle_name;

            // Vehicle setting
            VehicleSetting vehicle_setting;
            // Camera data
            std::unordered_map<std::string, std::unique_ptr<CameraROS>> camera_map;
            // Subscribers
            rclcpp::Subscription<airsim_interfaces::msg::GimbalAngleCmd>::SharedPtr gimbal_angle_cmd_sub;
            rclcpp::Subscription<airsim_interfaces::msg::CameraFovCmd>::SharedPtr camera_fov_cmd_sub;
            // Transforms
            geometry_msgs::msg::TransformStamped local_static_tf_msg;
            // Frame IDs
            std::string local_frame_id;
        };

    // ════════════════════════════════════════════════════════════════
    // PRIVATE: ROS Callbacks
    private:
        /// Timer callbacks
        void drone_state_timer_cb();
        void clock_timer_cb();
        void status_timer_cb();

        /// Subscriber callbacks
        void vel_cmd_cb(const std::string& vehicle_name, const airsim_interfaces::msg::VelCmd::SharedPtr vel_cmd_msg);
        void gimbal_angle_cmd_cb(const std::string& vehicle_name, const airsim_interfaces::msg::GimbalAngleCmd::SharedPtr gimbal_angle_cmd_msg);
        void camera_fov_cmd_cb(const std::string& vehicle_name, const airsim_interfaces::msg::CameraFovCmd::SharedPtr camera_fov_cmd_msg);
        void window_image_cmd_group_cb(const airsim_interfaces::msg::WindowImageCmdGroup::SharedPtr window_image_cmd_group_msg);
        void window_rectangle_cmd_cb(const airsim_interfaces::msg::WindowRectangleCmd::SharedPtr window_rectangle_cmd_msg);
        void window_string_cmd_cb(const airsim_interfaces::msg::WindowStringCmd::SharedPtr window_string_cmd_msg);
        void update_target_cmd_group_cb(const airsim_interfaces::msg::UpdateTargetCmdGroup::SharedPtr update_target_cmd_group_msg);
        void update_cluster_cmd_group_cb(const airsim_interfaces::msg::UpdateClusterCmdGroup::SharedPtr update_cluster_cmd_group_msg);

        /// Service callbacks
        bool reset_srv_cb(const std::shared_ptr<airsim_interfaces::srv::Reset::Request> request, const std::shared_ptr<airsim_interfaces::srv::Reset::Response> response);
        bool run_srv_cb(const std::shared_ptr<airsim_interfaces::srv::Run::Request> request, const std::shared_ptr<airsim_interfaces::srv::Run::Response> response);
        bool pause_srv_cb(const std::shared_ptr<airsim_interfaces::srv::Pause::Request> request, const std::shared_ptr<airsim_interfaces::srv::Pause::Response> response);
        bool takeoff_group_srv_cb(const std::shared_ptr<airsim_interfaces::srv::TakeoffGroup::Request> request, const std::shared_ptr<airsim_interfaces::srv::TakeoffGroup::Response> response);
        bool land_group_srv_cb(const std::shared_ptr<airsim_interfaces::srv::LandGroup::Request> request, const std::shared_ptr<airsim_interfaces::srv::LandGroup::Response> response);
        bool hover_group_srv_cb(const std::shared_ptr<airsim_interfaces::srv::HoverGroup::Request> request, const std::shared_ptr<airsim_interfaces::srv::HoverGroup::Response> response);
        bool add_target_group_cb(const std::shared_ptr<airsim_interfaces::srv::AddTargetGroup::Request> request, const std::shared_ptr<airsim_interfaces::srv::AddTargetGroup::Response> response);
        bool add_cluster_group_cb(const std::shared_ptr<airsim_interfaces::srv::AddClusterGroup::Request> request, const std::shared_ptr<airsim_interfaces::srv::AddClusterGroup::Response> response);

    // ════════════════════════════════════════════════════════════════
    // PRIVATE: State Update
    private:
        void update_vehicle_odom(VehicleROS* vehicle_ros, const msr::airlib::Kinematics::State& kinematics_estimated);
        void update_camera_info(CameraROS* camera_ros, const float& hfov);

    // ════════════════════════════════════════════════════════════════
    // PRIVATE: State Publish
    private:
        void publish_vehicle_state();

    // ════════════════════════════════════════════════════════════════
    // PRIVATE: Settings and Configuration
    private:
        void create_ros_comms_from_settings_json();
        void convert_tf_msg_to_ros(geometry_msgs::msg::TransformStamped& tf_msg);
        void convert_pose_msg_to_ros(geometry_msgs::msg::Pose& pose_msg);
        void initialize_vehicle_odom(VehicleROS* vehicle_ros, const VehicleSetting& vehicle_setting);
        void initialize_vehicle_tf(VehicleROS* vehicle_ros, const VehicleSetting& vehicle_setting);
        void initialize_camera_info(CameraROS* camera_ros, const CameraSetting& camera_setting, const CaptureSetting& capture_setting);
        void initialize_camera_tf(VehicleROS* vehicle_ros, CameraROS* camera_ros, const CameraSetting& camera_setting);
        void set_nans_to_zeros_in_pose(VehicleSetting& vehicle_setting) const;
        void set_nans_to_zeros_in_pose(const VehicleSetting& vehicle_setting, CameraSetting& camera_setting) const;
        geometry_msgs::msg::Transform get_camera_optical_tf() const;

    // ════════════════════════════════════════════════════════════════
    // PRIVATE: AirSim Client Methods
    private:
        // State methods
        rclcpp::Time client_get_timestamp();
        bool client_get_paused();
        msr::airlib::MultirotorState client_get_multirotor_state(const std::string& vehicle_name);
        msr::airlib::CameraInfo client_get_camera_info(const std::string& vehicle_name, const std::string& camera_name);
        msr::airlib::Vector2r client_get_camera_fov(const std::string& vehicle_name, const std::string& camera_name);

        // Global methods
        void client_reset();
        void client_pause(const bool& is_paused);
        void client_enable_control();

        // Vehicle methods
        void client_takeoff(const float& timeout, const std::string& vehicle_name, const bool& wait = false);
        void client_land(const float& timeout, const std::string& vehicle_name, const bool& wait = false);
        void client_hover(const std::string& vehicle_name);
        void client_move_by_velocity(const float& vx, const float& vy, const float& vz, const float& dt, const std::string& vehicle_name);
        void client_set_camera_pose(const std::string& camera_name, const msr::airlib::Pose& pose, const std::string& vehicle_name);
        void client_set_camera_fov(const std::string& camera_name, const float& fov, const std::string& vehicle_name);

        // Window methods
        void client_set_window_images(const std::vector<int>& window_indices, const std::vector<std::string>& vehicle_names, const std::vector<std::string>& camera_names, const std::vector<msr::airlib::Vector2r>& corners, const std::vector<msr::airlib::Vector2r>& sizes);
        void client_set_window_rectangle(const int& window_index, const std::vector<msr::airlib::Vector2r>& corners, const std::vector<msr::airlib::Vector2r>& sizes, const std::vector<float>& color, const float& thickness);
        void client_set_window_strings(const int& window_index, const std::vector<std::string>& strings, const std::vector<msr::airlib::Vector2r>& positions, const std::vector<float>& color, const float& scale);

        // Tracking methods
        void client_add_targets(const std::vector<std::string>& target_names, const std::vector<std::string>& target_types, const std::vector<msr::airlib::Vector3r>& positions, const bool& highlight, const std::vector<std::vector<float>>& highlight_color_rgba);
        void client_add_clusters(const std::vector<std::string>& cluster_names, const std::vector<msr::airlib::Vector3r>& centers, const std::vector<float>& radii, const bool& highlight, const std::vector<std::vector<float>>& highlight_color_rgba);
        void client_remove_targets(const std::vector<std::string>& target_names);
        void client_remove_clusters(const std::vector<std::string>& cluster_names);
        void client_remove_all_targets();
        void client_remove_all_clusters();
        void client_update_targets(const std::vector<std::string>& target_names, const std::vector<msr::airlib::Vector3r>& positions);
        void client_update_clusters(const std::vector<std::string>& cluster_names, const std::vector<msr::airlib::Vector3r>& centers, const std::vector<float>& radii);

    // ════════════════════════════════════════════════════════════════
    // PRIVATE: Utility Methods
    private:
        rclcpp::Time get_airsim_timestamp();
        rclcpp::Time get_sim_clock_time();
        tf2::Quaternion get_tf2_quat(const msr::airlib::Quaternionr& airlib_quat) const;
        msr::airlib::Quaternionr get_airlib_quat(const geometry_msgs::msg::Quaternion& geometry_msgs_quat) const;
        msr::airlib::Quaternionr get_airlib_quat(const tf2::Quaternion& tf2_quat) const;
        msr::airlib::Pose get_airlib_pose(const float& x, const float& y, const float& z, const msr::airlib::Quaternionr& airlib_quat) const;
        msr::airlib::Pose get_airlib_pose(const geometry_msgs::msg::Pose& geometry_msgs_pose) const;
        msr::airlib::Vector3r get_airlib_point(const geometry_msgs::msg::Point& geometry_msgs_point) const;
        msr::airlib::Vector2r get_airlib_point_2d(const geometry_msgs::msg::Point& geometry_msgs_point) const;
        std::vector<msr::airlib::Vector3r> get_airlib_points(const std::vector<geometry_msgs::msg::Point>& geometry_msgs_points) const;
        std::vector<msr::airlib::Vector2r> get_airlib_points_2d(const std::vector<geometry_msgs::msg::Point>& geometry_msgs_points) const;
        std::vector<float> get_airlib_color(const std_msgs::msg::ColorRGBA& std_msgs_color) const;
        std::vector<std::vector<float>> get_airlib_colors(const std::vector<std_msgs::msg::ColorRGBA>& std_msgs_colors) const;
        geometry_msgs::msg::Transform get_transform_msg_from_airsim(const msr::airlib::Vector3r& position, const msr::airlib::AirSimSettings::Rotation& rotation);
        geometry_msgs::msg::Transform get_transform_msg_from_airsim(const msr::airlib::Vector3r& position, const msr::airlib::Quaternionr& quaternion);

    // ════════════════════════════════════════════════════════════════
    // PRIVATE: ROS Components
    private:
        // Status publisher
        rclcpp::Publisher<airsim_interfaces::msg::Status>::SharedPtr airsim_status_pub_;

        // Clock publisher
        rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub_;

        // Global services
        rclcpp::Service<airsim_interfaces::srv::Reset>::SharedPtr reset_srvr_;
        rclcpp::Service<airsim_interfaces::srv::Run>::SharedPtr run_srvr_;
        rclcpp::Service<airsim_interfaces::srv::Pause>::SharedPtr pause_srvr_;

        // Vehicle group services
        rclcpp::Service<airsim_interfaces::srv::TakeoffGroup>::SharedPtr takeoff_group_srvr_;
        rclcpp::Service<airsim_interfaces::srv::LandGroup>::SharedPtr land_group_srvr_;
        rclcpp::Service<airsim_interfaces::srv::HoverGroup>::SharedPtr hover_group_srvr_;

        // Window group subscribers
        rclcpp::Subscription<airsim_interfaces::msg::WindowImageCmdGroup>::SharedPtr window_image_cmd_group_sub_;
        rclcpp::Subscription<airsim_interfaces::msg::WindowRectangleCmd>::SharedPtr window_rectangle_cmd_sub_;
        rclcpp::Subscription<airsim_interfaces::msg::WindowStringCmd>::SharedPtr window_string_cmd_sub_;

        // Tracking group services
        rclcpp::Service<airsim_interfaces::srv::AddTargetGroup>::SharedPtr add_target_group_srvr_;
        rclcpp::Service<airsim_interfaces::srv::AddClusterGroup>::SharedPtr add_cluster_group_srvr_;

        // Tracking group subscribers
        rclcpp::Subscription<airsim_interfaces::msg::UpdateTargetCmdGroup>::SharedPtr update_target_cmd_group_sub_;
        rclcpp::Subscription<airsim_interfaces::msg::UpdateClusterCmdGroup>::SharedPtr update_cluster_cmd_group_sub_;

    // ════════════════════════════════════════════════════════════════
    // PRIVATE: Member Variables
    private:
        // Settings and configuration
        AirSimSettingsParser airsim_settings_parser_;
        std::string host_ip_;
        uint16_t host_port_;
        bool enable_world_plot_;
        double update_airsim_state_every_n_sec_;
        double update_sim_clock_every_n_sec_;
        double update_airsim_status_every_n_sec_;

        // Vehicle data
        std::unordered_map<std::string, std::unique_ptr<VehicleROS>> vehicle_map_;

        // AirSim clients
        std::unique_ptr<msr::airlib::MultirotorRpcLibClient> airsim_client_state_;
        std::unique_ptr<msr::airlib::MultirotorRpcLibClient> airsim_client_control_;
        std::unique_ptr<msr::airlib::RpcLibClientBase> airsim_client_window_;
        std::unique_ptr<msr::airlib::RpcLibClientBase> airsim_client_tracking_;

        // Node and callback groups
        std::shared_ptr<rclcpp::Node> nh_;
        std::shared_ptr<rclcpp::CallbackGroup> cb_state_;
        std::shared_ptr<rclcpp::CallbackGroup> cb_control_;
        std::shared_ptr<rclcpp::CallbackGroup> cb_window_;
        std::shared_ptr<rclcpp::CallbackGroup> cb_tracking_;

        // TF components
        std::string global_frame_id_ = "world";
        std::string local_frame_id_ = "local";
        std::string odom_frame_id_ = "odom";
        std::string camera_body_frame_id_ = "body";
        std::string camera_optical_frame_id_ = "optical";
        std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
        std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_pub_;

        // Timers
        rclcpp::TimerBase::SharedPtr airsim_state_update_timer_;
        rclcpp::TimerBase::SharedPtr sim_clock_update_timer_;
        rclcpp::TimerBase::SharedPtr airsim_status_update_timer_;
        rclcpp::TimerBase::SharedPtr world_plot_update_timer_;

        // Message storage
        rosgraph_msgs::msg::Clock ros_clock_;
        bool publish_clock_;
        std::atomic<bool> is_connected_;
        std::atomic<bool> is_running_;
    };

} // namespace airsim_wrapper