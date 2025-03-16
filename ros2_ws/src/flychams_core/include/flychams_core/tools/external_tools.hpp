#pragma once

// Core includes
#include "flychams_core/types/core_types.hpp"
#include "flychams_core/types/ros_types.hpp"
#include "flychams_core/utils/ros_utils.hpp"

namespace flychams::core
{
    /**
     * ════════════════════════════════════════════════════════════════
     * @brief External Communication Manager for handling communication
     * with the external framework
     *
     * @details
     * This class provides utilities for managing the communication
     * with the external framework.
     * ════════════════════════════════════════════════════════════════
     * @author Jose Francisco Lopez Ruiz
     * @date 2025-02-28
     * ════════════════════════════════════════════════════════════════
     */
    class ExternalTools
    {
    public: // Constructors/Destructors
        virtual ~ExternalTools() = default;
        virtual void shutdown() = 0;

    public: // Types
        using SharedPtr = std::shared_ptr<ExternalTools>;

    public: // Vehicle adders (override)
        virtual void addVehicle(const ID& vehicle_id) = 0;
        virtual void removeVehicle(const ID& vehicle_id) = 0;

    public: // Global control methods (override)
        virtual bool resetSimulation() = 0;
        virtual bool runSimulation() = 0;
        virtual bool pauseSimulation() = 0;

    public: // Vehicle control methods (override)
        virtual bool enableControl(const ID& vehicle_id, const bool& enable) = 0;
        virtual bool armDisarm(const ID& vehicle_id, const bool& arm) = 0;
        virtual bool takeoff(const ID& vehicle_id) = 0;
        virtual bool land(const ID& vehicle_id) = 0;
        virtual bool hover(const ID& vehicle_id) = 0;
        virtual void setVelocity(const ID& vehicle_id, const float& vel_cmd_x, const float& vel_cmd_y, const float& vel_cmd_z, const float& vel_cmd_dt) = 0;
        virtual void setPosition(const ID& vehicle_id, const float& pos_cmd_x, const float& pos_cmd_y, const float& pos_cmd_z, const float& pos_cmd_vel, const float& pos_cmd_timeout) = 0;
        virtual void setGimbalOrientations(const ID& vehicle_id, const IDs& camera_ids, const std::vector<QuaternionMsg>& target_quats) = 0;
        virtual void setCameraFovs(const ID& vehicle_id, const IDs& camera_ids, const std::vector<float>& target_fovs) = 0;

    public: // Window control methods (override)
        virtual void setWindowImageGroup(const IDs& window_ids, const IDs& vehicle_ids, const IDs& camera_ids, const std::vector<int>& crop_x, const std::vector<int>& crop_y, const std::vector<int>& crop_w, const std::vector<int>& crop_h) = 0;
        virtual void setWindowRectangles(const ID& window_id, const std::vector<PointMsg>& corners, const std::vector<PointMsg>& sizes, const ColorMsg& color, const float& thickness) = 0;
        virtual void setWindowStrings(const ID& window_id, const std::vector<std::string>& strings, const std::vector<PointMsg>& positions, const ColorMsg& color, const float& scale) = 0;

    public: // Tracking control methods (override)
        virtual bool addTargetGroup(const IDs& target_ids, const std::vector<TargetType>& target_types, const std::vector<PointMsg>& positions, const bool& highlight, const std::vector<ColorMsg>& highlight_colors) = 0;
        virtual bool addClusterGroup(const IDs& cluster_ids, const std::vector<PointMsg>& centers, const std::vector<float>& radii, const bool& highlight, const std::vector<ColorMsg>& highlight_colors) = 0;
        virtual bool removeAllTargets() = 0;
        virtual bool removeAllClusters() = 0;
        virtual void updateTargetGroup(const IDs& target_ids, const std::vector<PointMsg>& positions) = 0;
        virtual void updateClusterGroup(const IDs& cluster_ids, const std::vector<PointMsg>& centers, const std::vector<float>& radii) = 0;
    };

    ExternalTools::SharedPtr externalToolsFactory(NodePtr node, const Framework& framework);

} // namespace flychams::core