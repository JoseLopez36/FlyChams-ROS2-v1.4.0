#pragma once

// External tools include
#include "flychams_core/framework/framework_tools.hpp"

// AirSim interfaces includes
// Global commands
#include <airsim_interfaces/srv/reset.hpp>
#include <airsim_interfaces/srv/run.hpp>
#include <airsim_interfaces/srv/pause.hpp>
// Vehicle commands
#include <airsim_interfaces/srv/enable_control.hpp>
#include <airsim_interfaces/srv/arm_disarm.hpp>
#include <airsim_interfaces/srv/takeoff.hpp>
#include <airsim_interfaces/srv/land.hpp>
#include <airsim_interfaces/srv/hover.hpp>
#include <airsim_interfaces/msg/vel_cmd.hpp>
#include <airsim_interfaces/msg/pos_cmd.hpp>
#include <airsim_interfaces/msg/gimbal_angle_cmd.hpp>
#include <airsim_interfaces/msg/camera_fov_cmd.hpp>
// Window commands
#include <airsim_interfaces/msg/window_image_cmd_group.hpp>
#include <airsim_interfaces/msg/window_rectangle_cmd.hpp>
#include <airsim_interfaces/msg/window_string_cmd.hpp>
// Tracking commands
#include <airsim_interfaces/srv/add_target_group.hpp>
#include <airsim_interfaces/srv/add_cluster_group.hpp>
#include <airsim_interfaces/srv/remove_all_targets.hpp>
#include <airsim_interfaces/srv/remove_all_clusters.hpp>
#include <airsim_interfaces/msg/update_target_cmd_group.hpp>
#include <airsim_interfaces/msg/update_cluster_cmd_group.hpp>

// Core includes
#include "flychams_core/types/core_types.hpp"
#include "flychams_core/types/ros_types.hpp"
#include "flychams_core/utils/ros_utils.hpp"

namespace flychams::core
{
    /**
     * ════════════════════════════════════════════════════════════════
     * @brief AirSim-specific implementation of the framework tools utility
     *
     * ════════════════════════════════════════════════════════════════
     * @author Jose Francisco Lopez Ruiz
     * @date 2025-02-14
     * ════════════════════════════════════════════════════════════════
     */
    class AirsimTools : public FrameworkTools
    {
    public: // Constructors/Destructors
        AirsimTools(NodePtr node, const ConfigTools::SharedPtr& config_tools);
        ~AirsimTools() override;
        void shutdown() override;

    public: // Types
        using ResetSrv = airsim_interfaces::srv::Reset;
        using RunSrv = airsim_interfaces::srv::Run;
        using PauseSrv = airsim_interfaces::srv::Pause;
        using EnableControlSrv = airsim_interfaces::srv::EnableControl;
        using ArmDisarmSrv = airsim_interfaces::srv::ArmDisarm;
        using TakeoffSrv = airsim_interfaces::srv::Takeoff;
        using LandSrv = airsim_interfaces::srv::Land;
        using HoverSrv = airsim_interfaces::srv::Hover;
        using VelCmdMsg = airsim_interfaces::msg::VelCmd;
        using PosCmdMsg = airsim_interfaces::msg::PosCmd;
        using GimbalAngleCmdMsg = airsim_interfaces::msg::GimbalAngleCmd;
        using CameraFovCmdMsg = airsim_interfaces::msg::CameraFovCmd;
        using WindowImageCmdGroupMsg = airsim_interfaces::msg::WindowImageCmdGroup;
        using WindowRectangleCmdMsg = airsim_interfaces::msg::WindowRectangleCmd;
        using WindowStringCmdMsg = airsim_interfaces::msg::WindowStringCmd;
        using AddTargetGroupSrv = airsim_interfaces::srv::AddTargetGroup;
        using AddClusterGroupSrv = airsim_interfaces::srv::AddClusterGroup;
        using RemoveAllTargetsSrv = airsim_interfaces::srv::RemoveAllTargets;
        using RemoveAllClustersSrv = airsim_interfaces::srv::RemoveAllClusters;
        using UpdateTargetCmdGroupMsg = airsim_interfaces::msg::UpdateTargetCmdGroup;
        using UpdateClusterCmdGroupMsg = airsim_interfaces::msg::UpdateClusterCmdGroup;

    public: // Vehicle adders
        void addVehicle(const ID& vehicle_id);
        void removeVehicle(const ID& vehicle_id);

    public: // Vehicle state methods (override)
        SubscriberPtr<OdometryMsg> createOdometrySubscriber(const ID& vehicle_id, const std::function<void(const OdometryMsg::SharedPtr)>& callback, const rclcpp::SubscriptionOptions& options = rclcpp::SubscriptionOptions()) override;

    public: // Global control methods
        bool resetSimulation() override;
        bool runSimulation() override;
        bool pauseSimulation() override;

    public: // Vehicle control methods
        bool enableControl(const ID& vehicle_id, const bool& enable) override;
        bool armDisarm(const ID& vehicle_id, const bool& arm) override;
        bool takeoff(const ID& vehicle_id) override;
        bool land(const ID& vehicle_id) override;
        bool hover(const ID& vehicle_id) override;
        void setVelocity(const ID& vehicle_id, const float& vel_cmd_x, const float& vel_cmd_y, const float& vel_cmd_z, const float& vel_cmd_dt) override;
        void setPosition(const ID& vehicle_id, const float& pos_cmd_x, const float& pos_cmd_y, const float& pos_cmd_z, const float& pos_cmd_vel, const float& pos_cmd_timeout) override;
        void setGimbalOrientations(const ID& vehicle_id, const IDs& camera_ids, const std::vector<QuaternionMsg>& target_quats) override;
        void setCameraFovs(const ID& vehicle_id, const IDs& camera_ids, const std::vector<float>& target_fovs) override;

    public: // Window control methods
        void setWindows(const std::vector<WindowCmd>& window_cmds) override;
        void drawWindow(const DrawCmd& draw_cmd) override;

    public: // Tracking control methods
        bool addTargetGroup(const IDs& target_ids, const std::vector<TargetType>& target_types, const std::vector<PointMsg>& positions, const bool& highlight, const std::vector<ColorMsg>& highlight_colors) override;
        bool addClusterGroup(const IDs& cluster_ids, const std::vector<PointMsg>& centers, const std::vector<float>& radii, const bool& highlight, const std::vector<ColorMsg>& highlight_colors) override;
        bool removeAllTargets() override;
        bool removeAllClusters() override;
        void updateTargetGroup(const IDs& target_ids, const std::vector<PointMsg>& positions) override;
        void updateClusterGroup(const IDs& cluster_ids, const std::vector<PointMsg>& centers, const std::vector<float>& radii) override;

    private: // Utility methods
        int getWindowIndex(const ID& window_id) const
        {
            // Get system config
            const auto& system_config = config_tools_->getSystem();

            if (window_id == system_config.scenario_view_id)
                return 0;
            else if (window_id == system_config.agent_view_id)
                return 1;
            else if (window_id == system_config.map_view_id)
                return 2;
            else if (window_id == system_config.payload_view_id)
                return 3;
            else if (window_id == system_config.tracking_view_ids.at(0))
                return 4;
            else if (window_id == system_config.tracking_view_ids.at(1))
                return 5;
            else if (window_id == system_config.tracking_view_ids.at(2))
                return 6;
            else if (window_id == system_config.tracking_view_ids.at(3))
                return 7;
            else if (window_id == system_config.tracking_view_ids.at(4))
                return 8;
            else
            {
                RCLCPP_ERROR(node_->get_logger(), "Invalid window ID: %s", window_id.c_str());
                return 0;
            }
        }

    private: // ROS components
        // Global commands
        ClientPtr<ResetSrv> reset_client_;
        ClientPtr<RunSrv> run_client_;
        ClientPtr<PauseSrv> pause_client_;

        // Vehicle commands
        ClientPtr<EnableControlSrv> enable_control_client_;
        ClientPtr<ArmDisarmSrv> arm_disarm_client_;
        ClientPtr<TakeoffSrv> takeoff_client_;
        ClientPtr<LandSrv> land_client_;
        ClientPtr<HoverSrv> hover_client_;
        std::unordered_map<ID, PublisherPtr<VelCmdMsg>> vel_cmd_pub_map_;
        std::unordered_map<ID, PublisherPtr<PosCmdMsg>> pos_cmd_pub_map_;
        std::unordered_map<ID, PublisherPtr<GimbalAngleCmdMsg>> gimbal_angle_cmd_pub_map_;
        std::unordered_map<ID, PublisherPtr<CameraFovCmdMsg>> camera_fov_cmd_pub_map_;

        // Window commands
        PublisherPtr<WindowImageCmdGroupMsg> window_image_cmd_group_pub_;
        PublisherPtr<WindowRectangleCmdMsg> window_rectangle_cmd_pub_;
        PublisherPtr<WindowStringCmdMsg> window_string_cmd_pub_;

        // Tracking commands
        ClientPtr<AddTargetGroupSrv> add_target_group_client_;
        ClientPtr<AddClusterGroupSrv> add_cluster_group_client_;
        ClientPtr<RemoveAllTargetsSrv> remove_all_targets_client_;
        ClientPtr<RemoveAllClustersSrv> remove_all_clusters_client_;
        PublisherPtr<UpdateTargetCmdGroupMsg> update_target_cmd_group_pub_;
        PublisherPtr<UpdateClusterCmdGroupMsg> update_cluster_cmd_group_pub_;
    };

} // namespace flychams::core