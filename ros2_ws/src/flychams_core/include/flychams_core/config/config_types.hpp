#pragma once

// Standard includes
#include <string>
#include <vector>
#include <cstdint>
#include <unordered_map>
#include <memory>

// Core includes
#include "flychams_core/types/core_types.hpp"

namespace flychams::core
{
    /**
     * ════════════════════════════════════════════════════════════════
     * @brief Types for mission configuration
     *
     * @details
     * This file contains all the configuration data structures used
     * to define mission settings, including hardware types, mission
     * types, and other types.
     * ════════════════════════════════════════════════════════════════
     */

    // ════════════════════════════════════════════════════════════════
    // HARDWARE TYPES: Hardware-related configuration types
    // ════════════════════════════════════════════════════════════════

    struct CameraConfig
    {
        // Identifiers
        ID camera_model_id;
        Name camera_name;
        CameraType camera_type;

        // Internal config
        Vector2i resolution;
        float default_focal;
        float sensor_width;
        float sensor_height;
        float weight;
        float idle_power;
        float active_power;
    };
    using CameraConfigPtr = std::shared_ptr<CameraConfig>;

    struct GimbalLinkConfig
    {
        // Identifiers
        ID gimbal_link_id;
        ID link_configuration_id;
        uint8_t link_index;

        // Internal config
        AxisType axis_type;
        Vector3r link_offset;
        float link_length;
        Vector2r joint_range;
        float max_angular_speed;
        float motor_rise_time;
        float motor_damping_factor;
    };
    using GimbalLinkConfigPtr = std::shared_ptr<GimbalLinkConfig>;
    using GimbalLinkConfigMap = std::unordered_map<ID, GimbalLinkConfigPtr>;

    struct GimbalConfig
    {
        // Identifiers
        ID gimbal_model_id;
        Name gimbal_name;
        ID link_configuration_id;

        // Internal config
        Vector3r optical_center_offset;
        float weight;
        float idle_power;
        float active_power;

        // External config
        GimbalLinkConfigMap links;
    };
    using GimbalConfigPtr = std::shared_ptr<GimbalConfig>;

    struct DroneConfig
    {
        // Identifiers
        ID drone_model_id;
        Name drone_name;
        DroneType drone_type;

        // Internal config
        float cruise_speed;
        float max_speed;
        float base_weight;
        float max_payload_weight;
        float hover_power;
        float cruise_power;
        float load_factor;
    };
    using DroneConfigPtr = std::shared_ptr<DroneConfig>;

    // ════════════════════════════════════════════════════════════════
    // AGENT TYPES: Agent-related configuration types
    // ════════════════════════════════════════════════════════════════

    struct HeadConfig
    {
        // Identifiers
        ID head_id;
        ID head_payload_id;
        Name head_name;
        ID gimbal_model_id;
        ID camera_model_id;

        // Internal config
        HeadRole head_role;
        Vector3r mount_position;
        Vector3r mount_orientation;
        Vector3r initial_orientation;
        float initial_focal;
        float min_focal;
        float max_focal;

        // External config
        GimbalConfigPtr gimbal;
        CameraConfigPtr camera;
    };
    using HeadConfigPtr = std::shared_ptr<HeadConfig>;
    using HeadConfigMap = std::unordered_map<ID, HeadConfigPtr>;

    struct AgentConfig
    {
        // Identifiers
        ID agent_id;
        ID agent_team_id;
        Name agent_name;
        ID head_payload_id;
        ID drone_model_id;

        // Internal config
        TrackingMode tracking_mode;
        Vector3r initial_position;
        Vector3r initial_orientation;
        float safety_radius;
        float max_altitude;
        float battery_capacity;

        // External config
        HeadConfigMap heads;
        DroneConfigPtr drone;

        // Other parameters
        float min_admissible_height;
        float max_admissible_height;
        ID central_head_id;
        IDs tracking_head_ids;
        int32_t max_assignments;
    };
    using AgentConfigPtr = std::shared_ptr<AgentConfig>;
    using AgentConfigMap = std::unordered_map<ID, AgentConfigPtr>;

    // ════════════════════════════════════════════════════════════════
    // GENERAL TYPES: Mission-related configuration types
    // ════════════════════════════════════════════════════════════════

    struct GroupConfig
    {
        // Identifiers
        ID group_id;
        ID group_bundle_id;
        Name group_name;

        // Internal config
        TargetType target_type;
        uint32_t target_count;
        Priority group_priority;
        std::string trajectory_folder;
    };
    using GroupConfigPtr = std::shared_ptr<GroupConfig>;
    using GroupConfigMap = std::unordered_map<ID, GroupConfigPtr>;

    struct MapConfig
    {
        ID map_id;
        Name map_name;
        RegionType region_type;
        Coordinates origin_geopoint;
        DateTime start_time;
        Vector3r wind_velocity;
    };
    using MapConfigPtr = std::shared_ptr<MapConfig>;

    struct SimulationConfig
    {
        // Identifiers
        ID simulation_id;
        Name simulation_name;

        // Internal config
        Autopilot autopilot;
        float clock_speed;
        bool record_metrics;
        bool record_markers;
    };
    using SimulationConfigPtr = std::shared_ptr<SimulationConfig>;

    struct MissionConfig
    {
        // Identifiers
        ID mission_id;
        Name mission_name;
        ID simulation_id;
        ID map_id;
        ID group_bundle_id;
        ID parameter_set_id;
        ID agent_team_id;
        Vector2r altitude_constraint;
        Vector2i tracking_scene_resolution;
    };
    using MissionConfigPtr = std::shared_ptr<MissionConfig>;

    /**
     * @brief Configuration structure for the mission
     */
    struct Config
    {
        // Mission config
        MissionConfigPtr mission;

        // Simulation config
        SimulationConfigPtr simulation;

        // Map config
        MapConfigPtr map;

        // Group config
        GroupConfigMap groups;

        // Agent config
        AgentConfigMap agents;

        // Other parameters
        uint32_t group_count;
        uint32_t agent_count;
    };
    using ConfigPtr = std::shared_ptr<Config>;

} // namespace flychams::core