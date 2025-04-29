#pragma once

// Config includes
#include "flychams_core/config/spreadsheet_parser.hpp"
#include "flychams_core/config/airsim_settings_creator.hpp"

// Core includes
#include "flychams_core/utils/math_utils.hpp"
#include "flychams_core/utils/ros_utils.hpp"

namespace flychams::core
{
    /**
     * ════════════════════════════════════════════════════════════════
     * @brief Config Manager for handling configuration parsing and
     * utilities.
     *
     * @details
     * This class provides utilities for managing the configuration of the
     * FlyChams system.
     * ════════════════════════════════════════════════════════════════
     * @author Jose Francisco Lopez Ruiz
     * @date 2025-02-28
     * ════════════════════════════════════════════════════════════════
     */
    class ConfigTools
    {
    public: // Constructor/Destructor
        ConfigTools(NodePtr node)
            : node_(node)
        {
            // Parse the configuration spreadsheet
            const std::string& path = RosUtils::getParameter<std::string>(node_, "path.config_spreadsheet_path");
            try
            {
                RCLCPP_INFO(node_->get_logger(), "Parsing config spreadsheet: %s", path.c_str());
                config_ptr_ = SpreadsheetParser::parseSpreadsheet(path);
                RCLCPP_INFO(node_->get_logger(), "Config spreadsheet parsed successfully");
            }
            catch (const std::exception& e)
            {
                RCLCPP_ERROR(node_->get_logger(), "Error parsing config spreadsheet: %s", e.what());
                rclcpp::shutdown();
            }

            // Get parameters from node
            parseParameters(config_ptr_);
        }

        ~ConfigTools()
        {
            shutdown();
        }

        void shutdown()
        {
            // Destroy config
            config_ptr_.reset();
            // Destroy node
            node_.reset();
        }

    public: // Types
        using SharedPtr = std::shared_ptr<ConfigTools>;

    private: // Data
        // Configuration
        MissionConfigPtr config_ptr_;

        // ROS components
        NodePtr node_;

    public: // AirSim utilities
        void createAirsimSettings() const
        {
            const std::string& path = RosUtils::getParameter<std::string>(node_, "path.airsim_settings_path");
            if (!AirsimSettingsCreator::createAirsimSettings(config_ptr_, path))
            {
                RCLCPP_ERROR(node_->get_logger(), "Failed to create AirSim settings.json at %s", path.c_str());
                rclcpp::shutdown();
            }
            else
            {
                RCLCPP_INFO(node_->get_logger(), "AirSim settings.json created successfully at %s", path.c_str());
            }
        }

    public: // Raw getter methods

        const MissionConfigPtr getConfig() const
        {
            return config_ptr_;
        }

        const EnvironmentConfig getEnvironment() const
        {
            return config_ptr_->environment;
        }

        const TargetGroupConfig getTargetGroup() const
        {
            return config_ptr_->target_group;
        }

        const TargetConfigPtr getTarget(const ID& target_id) const
        {
            return config_ptr_->target_group.at(target_id);
        }

        const AgentTeamConfig getAgentTeam() const
        {
            return config_ptr_->agent_team;
        }

        const AgentConfigPtr getAgent(const ID& agent_id) const
        {
            return config_ptr_->agent_team.at(agent_id);
        }

        const TrackingConfig getTracking(const ID& agent_id) const
        {
            return config_ptr_->agent_team.at(agent_id)->tracking;
        }

        const HeadSetConfig getHeadSet(const ID& agent_id) const
        {
            return config_ptr_->agent_team.at(agent_id)->head_set;
        }

        const HeadConfigPtr getHead(const ID& agent_id, const ID& head_id) const
        {
            return config_ptr_->agent_team.at(agent_id)->head_set.at(head_id);
        }

        const WindowSetConfig getWindowSet(const ID& agent_id) const
        {
            return config_ptr_->agent_team.at(agent_id)->window_set;
        }

        const WindowConfigPtr getWindow(const ID& agent_id, const ID& window_id) const
        {
            return config_ptr_->agent_team.at(agent_id)->window_set.at(window_id);
        }

        const DroneConfig getDrone(const ID& agent_id) const
        {
            return config_ptr_->agent_team.at(agent_id)->drone;
        }

        const GimbalConfig getGimbal(const ID& agent_id, const ID& head_id) const
        {
            return config_ptr_->agent_team.at(agent_id)->head_set.at(head_id)->gimbal;
        }

        const CameraConfig getCamera(const ID& agent_id, const ID& head_id) const
        {
            return config_ptr_->agent_team.at(agent_id)->head_set.at(head_id)->camera;
        }

        const SystemParameters getSystem() const
        {
            return config_ptr_->system;
        }

        const TopicParameters getTopics() const
        {
            return config_ptr_->topics;
        }

        const FrameParameters getFrames() const
        {
            return config_ptr_->frames;
        }

    public: // Processing getter methods
        const std::pair<std::vector<HeadConfigPtr>, int> getHeads(const std::string& agent_id) const
        {
            // Get head set
            const auto& head_set = getHeadSet(agent_id);
            int n = static_cast<int>(head_set.size());

            // Create vector of heads
            std::vector<HeadConfigPtr> heads(n);
            int tracking = 1;
            for (const auto& [head_id, head_ptr] : head_set)
            {
                if (head_ptr->role == TrackingRole::Central)
                {
                    heads[0] = head_ptr;
                }
                else if (head_ptr->role == TrackingRole::Tracking)
                {
                    heads[tracking] = head_ptr;
                    tracking++;
                }
            }

            return std::make_pair(heads, n);
        }

        const std::pair<std::vector<WindowConfigPtr>, int> getWindows(const std::string& agent_id) const
        {
            // Get window set
            const auto& window_set = getWindowSet(agent_id);
            int n = static_cast<int>(window_set.size());

            // Create vector of windows
            std::vector<WindowConfigPtr> windows(n);
            int tracking = 1;
            for (const auto& [window_id, window_ptr] : window_set)
            {
                if (window_ptr->role == TrackingRole::Central)
                {
                    windows[0] = window_ptr;
                }
                else if (window_ptr->role == TrackingRole::Tracking)
                {
                    windows[tracking] = window_ptr;
                    tracking++;
                }
            }

            return std::make_pair(windows, n);
        }

        const HeadParameters getHeadParameters(const std::string& agent_id, const std::string& head_id) const
        {
            HeadParameters params;

            // Extract head config
            const auto& head_ptr = getHead(agent_id, head_id);
            const auto& camera = getCamera(agent_id, head_id);

            // Extract tracking config
            const auto& tracking = getTracking(agent_id);

            // Head ID
            params.id = head_id;

            // Head role
            params.role = head_ptr->role;

            // Camera focal length limits (m)
            params.f_min = head_ptr->min_focal;
            params.f_max = head_ptr->max_focal;
            params.f_ref = head_ptr->ref_focal;

            // Camera resolution (pix)
            params.width = camera.resolution(0);
            params.height = camera.resolution(1);

            // Camera sensor dimensions (m)
            params.sensor_width = camera.sensor_size(0);
            params.sensor_height = camera.sensor_size(1);

            // Regularized pixel size (m/pix)
            float rho_x = params.sensor_width / static_cast<float>(params.width);       // [m/pix]
            float rho_y = params.sensor_height / static_cast<float>(params.height);     // [m/pix]
            params.rho = std::sqrt(rho_x * rho_y);                                      // [m/pix]

            // Camera reference intrinsic matrix K
            params.K = Matrix3r::Identity();
            params.K(0, 0) = params.f_ref / rho_x;
            params.K(1, 1) = params.f_ref / rho_y;
            params.K(0, 2) = params.width / 2.0f;
            params.K(1, 2) = params.height / 2.0f;

            // Calculate ROI parameters
            const auto& min_apparent_size = tracking.min_target_size;
            const auto& max_apparent_size = tracking.max_target_size;
            const auto& ref_apparent_size = tracking.ref_target_size;
            float sensor_half_size = static_cast<float>(std::min(params.width, params.height)) / 2.0f;

            // Minimum admissible apparent size of the object in the image (in pixels)
            float s_min_pix = sensor_half_size * min_apparent_size;
            params.s_min_pix = s_min_pix; // [pix]

            // Maximum admissible apparent size of the object in the image (in pixels)
            float s_max_pix = sensor_half_size * max_apparent_size;
            params.s_max_pix = s_max_pix; // [pix]

            // Reference apparent size of the object in the image (in pixels)
            float s_ref_pix = sensor_half_size * ref_apparent_size; // [pix]
            params.s_ref_pix = s_ref_pix; // [pix]

            // Conversion to metric distances on the sensor surface
            params.s_max = s_max_pix * params.rho; // [m]
            params.s_min = s_min_pix * params.rho; // [m]
            params.s_ref = s_ref_pix * params.rho; // [m]  

            // Print camera parameters for debugging
            RCLCPP_INFO(node_->get_logger(), "Head parameters for agent %s, head %s:", agent_id.c_str(), head_id.c_str());
            RCLCPP_INFO(node_->get_logger(), "  Focal lengths: min=%.3f, max=%.3f, ref=%.3f [m]", params.f_min, params.f_max, params.f_ref);
            RCLCPP_INFO(node_->get_logger(), "  Resolution: %d x %d [pix]", params.width, params.height);
            RCLCPP_INFO(node_->get_logger(), "  Sensor dimensions: %.6f x %.6f [m]", params.sensor_width, params.sensor_height);
            RCLCPP_INFO(node_->get_logger(), "  Regularized pixel size: %.6f [m/pix]", params.rho);
            RCLCPP_INFO(node_->get_logger(), "  Intrinsic matrix K: fx=%f fy=%f cx=%f cy=%f", params.K(0, 0), params.K(1, 1), params.K(0, 2), params.K(1, 2));
            RCLCPP_INFO(node_->get_logger(), "  s_min_pix: %.2f [pix]", params.s_min_pix);
            RCLCPP_INFO(node_->get_logger(), "  s_max_pix: %.2f [pix]", params.s_max_pix);
            RCLCPP_INFO(node_->get_logger(), "  s_ref_pix: %.2f [pix]", params.s_ref_pix);
            RCLCPP_INFO(node_->get_logger(), "  s_min: %.6f [m]", params.s_min);
            RCLCPP_INFO(node_->get_logger(), "  s_max: %.6f [m]", params.s_max);
            RCLCPP_INFO(node_->get_logger(), "  s_ref: %.6f [m]", params.s_ref);

            return params;
        }

        const WindowParameters getWindowParameters(const std::string& agent_id, const std::string& window_id, const HeadParameters& central_head_params) const
        {
            WindowParameters params;

            // Extract window config
            const auto& window_ptr = getWindow(agent_id, window_id);

            // Extract tracking config
            const auto& tracking = getTracking(agent_id);

            // Window ID
            params.id = window_id;

            // Head role
            params.role = window_ptr->role;

            // Source camera ID
            params.source_id = central_head_params.id;

            // Window resolution factors limits
            params.lambda_min = window_ptr->min_lambda;
            params.lambda_max = window_ptr->max_lambda;
            params.lambda_ref = window_ptr->ref_lambda;

            // Full resolution (pix)
            params.full_width = central_head_params.width;
            params.full_height = central_head_params.height;

            // Tracking resolution (pix)
            params.tracking_width = window_ptr->resolution(0);
            params.tracking_height = window_ptr->resolution(1);

            // Get rho
            params.rho = central_head_params.rho;

            // Calculate ROI parameters
            const auto& min_apparent_size = tracking.min_target_size;
            const auto& max_apparent_size = tracking.max_target_size;
            const auto& ref_apparent_size = tracking.ref_target_size;
            float sensor_half_size = static_cast<float>(std::min(params.tracking_width, params.tracking_height)) / 2.0f;

            // Minimum admissible apparent size of the object in the image (in pixels)
            float s_min_pix = sensor_half_size * min_apparent_size;
            params.s_min_pix = s_min_pix; // [pix]

            // Maximum admissible apparent size of the object in the image (in pixels)
            float s_max_pix = sensor_half_size * max_apparent_size;
            params.s_max_pix = s_max_pix; // [pix]

            // Reference apparent size of the object in the image (in pixels)
            float s_ref_pix = sensor_half_size * ref_apparent_size; // [pix]
            params.s_ref_pix = s_ref_pix; // [pix]

            // Conversion to metric distances on the sensor surface
            params.s_max = s_max_pix * params.rho; // [m]
            params.s_min = s_min_pix * params.rho; // [m]
            params.s_ref = s_ref_pix * params.rho; // [m]  

            // Print window parameters for debugging
            RCLCPP_INFO(node_->get_logger(), "Window parameters for agent %s, window %s:", agent_id.c_str(), window_id.c_str());
            RCLCPP_INFO(node_->get_logger(), "  Resolution factors: min=%.3f, max=%.3f, ref=%.3f", params.lambda_min, params.lambda_max, params.lambda_ref);
            RCLCPP_INFO(node_->get_logger(), "  Full resolution: %d x %d", params.full_width, params.full_height);
            RCLCPP_INFO(node_->get_logger(), "  Tracking resolution: %d x %d", params.tracking_width, params.tracking_height);
            RCLCPP_INFO(node_->get_logger(), "  Rho: %.6f [m/pix]", params.rho);
            RCLCPP_INFO(node_->get_logger(), "  s_min_pix: %.2f [pix]", params.s_min_pix);
            RCLCPP_INFO(node_->get_logger(), "  s_max_pix: %.2f [pix]", params.s_max_pix);
            RCLCPP_INFO(node_->get_logger(), "  s_ref_pix: %.2f [pix]", params.s_ref_pix);
            RCLCPP_INFO(node_->get_logger(), "  s_min: %.6f [m]", params.s_min);
            RCLCPP_INFO(node_->get_logger(), "  s_max: %.6f [m]", params.s_max);
            RCLCPP_INFO(node_->get_logger(), "  s_ref: %.6f [m]", params.s_ref);

            return params;
        }

        const TrackingParameters getTrackingParameters(const std::string& agent_id) const
        {
            TrackingParameters params;

            // Extract tracking config
            const auto& tracking = getTracking(agent_id);

            // Get tracking mode
            params.mode = tracking.mode;

            // Get heads and windows
            const auto& [heads, n_heads] = getHeads(agent_id);
            const auto& [windows, n_windows] = getWindows(agent_id);

            // Set number of heads and windows
            params.n_heads = n_heads;
            params.n_windows = n_windows;

            // Get tracking parameters based on tracking mode
            switch (params.mode)
            {
            case TrackingMode::MultiCamera:
            {
                // Set parameters for each unit
                params.head_params.resize(params.n_heads);
                for (int i = 0; i < params.n_heads; i++)
                {
                    params.head_params[i] = getHeadParameters(agent_id, heads[i]->id);
                }
                break;
            }

            case TrackingMode::MultiWindow:
            {
                // Set central head parameters
                params.head_params.resize(1);
                params.head_params[0] = getHeadParameters(agent_id, heads[0]->id);

                // Set parameters for each unit
                params.window_params.resize(params.n_windows);
                for (int i = 0; i < params.n_windows; i++)
                {
                    params.window_params[i] = getWindowParameters(agent_id, windows[i]->id, params.head_params[0]);
                }
                break;
            }
            }

            return params;
        }

    private: // Parameter parsing
        void parseParameters(MissionConfigPtr& config_ptr)
        {
            // Parse system, topics and frames parameters
            parseSystemParameters(config_ptr);
            parseTopicParameters(config_ptr);
            parseFrameParameters(config_ptr);
        }

        void parseSystemParameters(MissionConfigPtr& config_ptr)
        {
            // Simulation settings
            const std::string& framework_str = RosUtils::getParameter<std::string>(node_, "simulation.framework");
            config_ptr->system.framework = frameworkFromString(framework_str);
            config_ptr->system.clock_speed = RosUtils::getParameter<float>(node_, "simulation.clock_speed");

            // Path settings
            config_ptr->system.config_source_file = RosUtils::getParameter<std::string>(node_, "path.config_spreadsheet_path");
            config_ptr->system.airsim_settings_destination_file = RosUtils::getParameter<std::string>(node_, "path.airsim_settings_path");
            config_ptr->system.trajectory_root = RosUtils::getParameter<std::string>(node_, "path.trajectory_root");

            // GUI settings
            // Scenario view settings
            config_ptr->system.scenario_view_id = RosUtils::getParameter<ID>(node_, "gui.scenario_view_id");
            config_ptr->system.scenario_camera_id = RosUtils::getParameter<ID>(node_, "gui.scenario_camera_id");
            const std::vector<double> scenario_camera_position_vec = RosUtils::getParameter<std::vector<double>>(node_, "gui.scenario_camera_position");
            if (scenario_camera_position_vec.size() >= 3)
            {
                config_ptr->system.scenario_camera_position.x() = scenario_camera_position_vec[0];
                config_ptr->system.scenario_camera_position.y() = scenario_camera_position_vec[1];
                config_ptr->system.scenario_camera_position.z() = scenario_camera_position_vec[2];
            }
            const std::vector<double> scenario_camera_orientation_vec = RosUtils::getParameter<std::vector<double>>(node_, "gui.scenario_camera_orientation");
            if (scenario_camera_orientation_vec.size() >= 3)
            {
                config_ptr->system.scenario_camera_orientation.x() = MathUtils::degToRad(scenario_camera_orientation_vec[0]);
                config_ptr->system.scenario_camera_orientation.y() = MathUtils::degToRad(scenario_camera_orientation_vec[1]);
                config_ptr->system.scenario_camera_orientation.z() = MathUtils::degToRad(scenario_camera_orientation_vec[2]);
            }
            // Agent view settings
            config_ptr->system.agent_view_id = RosUtils::getParameter<ID>(node_, "gui.agent_view_id");
            config_ptr->system.agent_camera_id = RosUtils::getParameter<ID>(node_, "gui.agent_camera_id");
            const std::vector<double> agent_camera_position_vec = RosUtils::getParameter<std::vector<double>>(node_, "gui.agent_camera_position");
            if (agent_camera_position_vec.size() >= 3)
            {
                config_ptr->system.agent_camera_position.x() = agent_camera_position_vec[0];
                config_ptr->system.agent_camera_position.y() = agent_camera_position_vec[1];
                config_ptr->system.agent_camera_position.z() = agent_camera_position_vec[2];
            }
            const std::vector<double> agent_camera_orientation_vec = RosUtils::getParameter<std::vector<double>>(node_, "gui.agent_camera_orientation");
            if (agent_camera_orientation_vec.size() >= 3)
            {
                config_ptr->system.agent_camera_orientation.x() = MathUtils::degToRad(agent_camera_orientation_vec[0]);
                config_ptr->system.agent_camera_orientation.y() = MathUtils::degToRad(agent_camera_orientation_vec[1]);
                config_ptr->system.agent_camera_orientation.z() = MathUtils::degToRad(agent_camera_orientation_vec[2]);
            }
            // Payload view settings
            config_ptr->system.payload_view_id = RosUtils::getParameter<ID>(node_, "gui.payload_view_id");
            config_ptr->system.payload_camera_id = RosUtils::getParameter<ID>(node_, "gui.payload_camera_id");
            const std::vector<double> payload_camera_position_vec = RosUtils::getParameter<std::vector<double>>(node_, "gui.payload_camera_position");
            if (payload_camera_position_vec.size() >= 3)
            {
                config_ptr->system.payload_camera_position.x() = payload_camera_position_vec[0];
                config_ptr->system.payload_camera_position.y() = payload_camera_position_vec[1];
                config_ptr->system.payload_camera_position.z() = payload_camera_position_vec[2];
            }
            const std::vector<double> payload_camera_orientation_vec = RosUtils::getParameter<std::vector<double>>(node_, "gui.payload_camera_orientation");
            if (payload_camera_orientation_vec.size() >= 3)
            {
                config_ptr->system.payload_camera_orientation.x() = MathUtils::degToRad(payload_camera_orientation_vec[0]);
                config_ptr->system.payload_camera_orientation.y() = MathUtils::degToRad(payload_camera_orientation_vec[1]);
                config_ptr->system.payload_camera_orientation.z() = MathUtils::degToRad(payload_camera_orientation_vec[2]);
            }
            // Map view settings
            config_ptr->system.map_view_id = RosUtils::getParameter<ID>(node_, "gui.map_view_id");
            config_ptr->system.map_camera_id = RosUtils::getParameter<ID>(node_, "gui.map_camera_id");
            const std::vector<double> map_camera_position_vec = RosUtils::getParameter<std::vector<double>>(node_, "gui.map_camera_position");
            if (map_camera_position_vec.size() >= 3)
            {
                config_ptr->system.map_camera_position.x() = map_camera_position_vec[0];
                config_ptr->system.map_camera_position.y() = map_camera_position_vec[1];
                config_ptr->system.map_camera_position.z() = map_camera_position_vec[2];
            }
            const std::vector<double> map_camera_orientation_vec = RosUtils::getParameter<std::vector<double>>(node_, "gui.map_camera_orientation");
            if (map_camera_orientation_vec.size() >= 3)
            {
                config_ptr->system.map_camera_orientation.x() = MathUtils::degToRad(map_camera_orientation_vec[0]);
                config_ptr->system.map_camera_orientation.y() = MathUtils::degToRad(map_camera_orientation_vec[1]);
                config_ptr->system.map_camera_orientation.z() = MathUtils::degToRad(map_camera_orientation_vec[2]);
            }
            // Tracking views settings
            config_ptr->system.tracking_view_ids = RosUtils::getParameter<std::vector<ID>>(node_, "gui.tracking_view_ids");
        }

        void parseTopicParameters(MissionConfigPtr& config_ptr)
        {
            // Global topics
            config_ptr->topics.registration = RosUtils::getParameter<std::string>(node_, "global_topics.registration");
            config_ptr->topics.global_metrics = RosUtils::getParameter<std::string>(node_, "global_topics.metrics");

            // Agent topics
            config_ptr->topics.agent_status = RosUtils::getParameter<std::string>(node_, "agent_topics.status");
            config_ptr->topics.agent_position = RosUtils::getParameter<std::string>(node_, "agent_topics.position");
            config_ptr->topics.agent_assignment = RosUtils::getParameter<std::string>(node_, "agent_topics.assignment");
            config_ptr->topics.agent_clusters = RosUtils::getParameter<std::string>(node_, "agent_topics.clusters");
            config_ptr->topics.agent_position_setpoint = RosUtils::getParameter<std::string>(node_, "agent_topics.position_setpoint");
            config_ptr->topics.agent_tracking_setpoints = RosUtils::getParameter<std::string>(node_, "agent_topics.tracking_setpoints");
            config_ptr->topics.agent_metrics = RosUtils::getParameter<std::string>(node_, "agent_topics.metrics");
            config_ptr->topics.agent_markers = RosUtils::getParameter<std::string>(node_, "agent_topics.markers");

            // Target topics
            config_ptr->topics.target_true_position = RosUtils::getParameter<std::string>(node_, "target_topics.true_position");
            config_ptr->topics.target_est_position = RosUtils::getParameter<std::string>(node_, "target_topics.est_position");
            config_ptr->topics.target_metrics = RosUtils::getParameter<std::string>(node_, "target_topics.metrics");
            config_ptr->topics.target_markers = RosUtils::getParameter<std::string>(node_, "target_topics.markers");

            // Cluster topics
            config_ptr->topics.cluster_assignment = RosUtils::getParameter<std::string>(node_, "cluster_topics.assignment");
            config_ptr->topics.cluster_geometry = RosUtils::getParameter<std::string>(node_, "cluster_topics.geometry");
            config_ptr->topics.cluster_metrics = RosUtils::getParameter<std::string>(node_, "cluster_topics.metrics");
            config_ptr->topics.cluster_markers = RosUtils::getParameter<std::string>(node_, "cluster_topics.markers");

            // GUI topics
            config_ptr->topics.gui_setpoints = RosUtils::getParameter<std::string>(node_, "gui_topics.setpoints");
        }

        void parseFrameParameters(MissionConfigPtr& config_ptr)
        {
            // Global frames
            config_ptr->frames.world = RosUtils::getParameter<std::string>(node_, "global_frames.world");

            // Agent frames
            config_ptr->frames.agent_local = RosUtils::getParameter<std::string>(node_, "agent_frames.agent_local");
            config_ptr->frames.agent_body = RosUtils::getParameter<std::string>(node_, "agent_frames.agent_body");
            config_ptr->frames.camera_body = RosUtils::getParameter<std::string>(node_, "agent_frames.camera_body");
            config_ptr->frames.camera_optical = RosUtils::getParameter<std::string>(node_, "agent_frames.camera_optical");

        }
    };

} // namespace flychams::core