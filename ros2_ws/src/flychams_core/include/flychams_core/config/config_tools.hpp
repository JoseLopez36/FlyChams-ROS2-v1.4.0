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

        const HeadPayloadConfig getHeadPayload(const ID& agent_id) const
        {
            return config_ptr_->agent_team.at(agent_id)->head_payload;
        }

        const HeadConfigPtr getHead(const ID& agent_id, const ID& head_id) const
        {
            return config_ptr_->agent_team.at(agent_id)->head_payload.at(head_id);
        }

        const DroneConfig getDrone(const ID& agent_id) const
        {
            return config_ptr_->agent_team.at(agent_id)->drone;
        }

        const GimbalConfig getGimbal(const ID& agent_id, const ID& head_id) const
        {
            return config_ptr_->agent_team.at(agent_id)->head_payload.at(head_id)->gimbal;
        }

        const CameraConfig getCamera(const ID& agent_id, const ID& head_id) const
        {
            return config_ptr_->agent_team.at(agent_id)->head_payload.at(head_id)->camera;
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
        const HeadConfigPtr getCentralHead(const std::string& agent_id) const
        {
            for (const auto& [head_id, head_ptr] : getHeadPayload(agent_id))
            {
                if (head_ptr->role == HeadRole::Central)
                {
                    return head_ptr;
                }
            }

            return nullptr;
        }

        const std::vector<HeadConfigPtr> getTrackingHeads(const std::string& agent_id) const
        {
            std::vector<HeadConfigPtr> tracking_heads;

            for (const auto& [head_id, head_ptr] : getHeadPayload(agent_id))
            {
                if (head_ptr->role == HeadRole::Tracking)
                {
                    tracking_heads.push_back(head_ptr);
                }
            }

            return tracking_heads;
        }

        const CameraParameters getCameraParameters(const std::string& agent_id, const std::string& head_id) const
        {
            CameraParameters params;

            // Extract head config
            const auto& head_ptr = getHead(agent_id, head_id);
            const auto& camera = getCamera(agent_id, head_id);

            // Camera ID
            params.id = head_id;

            // Camera focal length limits (m)
            params.f_min = head_ptr->min_focal;
            params.f_max = head_ptr->max_focal;
            params.f_ref = head_ptr->ref_focal;

            // Camera resolution (pix)
            params.width = camera.resolution(0);
            params.height = camera.resolution(1);

            // Camera sensor dimensions (m)
            params.sensor_width = camera.sensor_width;
            params.sensor_height = camera.sensor_height;

            // Regularized pixel size (m/pix)
            float rho_x = params.sensor_width / static_cast<float>(params.width);       // [m/pix]
            float rho_y = params.sensor_height / static_cast<float>(params.height);     // [m/pix]
            params.rho = std::sqrt(rho_x * rho_y);                                      // [m/pix]

            // Camera reference intrinsic matrix K
            params.k_ref = Matrix3r::Identity();
            params.k_ref(0, 0) = params.f_ref / rho_x;
            params.k_ref(1, 1) = params.f_ref / rho_y;
            params.k_ref(0, 2) = params.width / 2.0f;
            params.k_ref(1, 2) = params.height / 2.0f;

            // Print camera parameters for debugging
            RCLCPP_INFO(node_->get_logger(), "Camera parameters for agent %s, head %s:", agent_id.c_str(), head_id.c_str());
            RCLCPP_INFO(node_->get_logger(), "  ID: %s", params.id.c_str());
            RCLCPP_INFO(node_->get_logger(), "  Focal lengths: min=%.3f, max=%.3f, ref=%.3f [m]", params.f_min, params.f_max, params.f_ref);
            RCLCPP_INFO(node_->get_logger(), "  Resolution: %d x %d [pix]", params.width, params.height);
            RCLCPP_INFO(node_->get_logger(), "  Sensor dimensions: %.6f x %.6f [m]", params.sensor_width, params.sensor_height);
            RCLCPP_INFO(node_->get_logger(), "  Regularized pixel size: %.6f [m/pix]", params.rho);
            RCLCPP_INFO(node_->get_logger(), "  Intrinsic matrix K: fx=%f fy=%f cx=%f cy=%f", params.k_ref(0, 0), params.k_ref(1, 1), params.k_ref(0, 2), params.k_ref(1, 2));

            return params;
        }

        const WindowParameters getWindowParameters(const std::string& agent_id, const std::string& source_head_id) const
        {
            WindowParameters params;

            // Get tracking config
            const auto& tracking = getTracking(agent_id);

            // Extract source head parameters
            params.camera_params = getCameraParameters(agent_id, source_head_id);

            // Full resolution (pix)
            params.full_width = params.camera_params.width;
            params.full_height = params.camera_params.height;

            // Scene resolution (pix)
            params.scene_width = tracking.scene_resolution(0);
            params.scene_height = tracking.scene_resolution(1);

            // View resolution (pix)
            params.tracking_width = tracking.tracking_resolution(0);
            params.tracking_height = tracking.tracking_resolution(1);

            // Calculate window parameters
            params.lambda_min = static_cast<float>(params.scene_width) / static_cast<float>(params.full_width);
            params.lambda_max = 1.0f;
            params.lambda_ref = (params.lambda_max + params.lambda_min) / 2.0f; // Middle of the range

            // Print window parameters for debugging
            RCLCPP_INFO(node_->get_logger(), "Window parameters for agent %s, source head %s:", agent_id.c_str(), source_head_id.c_str());
            RCLCPP_INFO(node_->get_logger(), "  Full resolution: %d x %d", params.full_width, params.full_height);
            RCLCPP_INFO(node_->get_logger(), "  Scene resolution: %d x %d", params.scene_width, params.scene_height);
            RCLCPP_INFO(node_->get_logger(), "  Tracking resolution: %d x %d", params.tracking_width, params.tracking_height);
            RCLCPP_INFO(node_->get_logger(), "  Lambda values: min=%.3f, max=%.3f, ref=%.3f", params.lambda_min, params.lambda_max, params.lambda_ref);

            return params;
        }

        const ProjectionParameters getProjectionParameters(const std::string& agent_id, const CameraParameters& camera_params) const
        {
            ProjectionParameters params;

            // Get tracking config
            const auto& tracking = getTracking(agent_id);

            // Extract ROI parameters
            const auto& min_apparent_size = tracking.min_target_size;
            const auto& max_apparent_size = tracking.max_target_size;
            const auto& ref_apparent_size = tracking.ref_target_size;
            float sensor_half_size = static_cast<float>(std::min(camera_params.width, camera_params.height)) / 2.0f;

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
            params.s_max = s_max_pix * camera_params.rho; // [m]
            params.s_min = s_min_pix * camera_params.rho; // [m]
            params.s_ref = s_ref_pix * camera_params.rho; // [m]

            // Print parameters for debugging
            RCLCPP_INFO(node_->get_logger(), "Projection parameters for agent %s, camera %s:", agent_id.c_str(), camera_params.id.c_str());
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

            // Get tracking config
            const auto& tracking = getTracking(agent_id);

            // Proceed based on tracking mode
            params.mode = tracking.mode;
            switch (params.mode)
            {
            case TrackingMode::MultiCameraTracking:
            {
                // Get tracking heads
                const auto& tracking_heads = getTrackingHeads(agent_id);

                // Get number of tracking heads
                params.n = static_cast<int>(tracking_heads.size());

                // Set camera and projection parameters for each tracking head
                params.camera_params.resize(params.n);
                params.projection_params.resize(params.n);
                for (int i = 0; i < params.n; i++)
                {
                    params.camera_params[i] = getCameraParameters(agent_id, tracking_heads[i]->id);
                    params.projection_params[i] = getProjectionParameters(agent_id, params.camera_params[i]);
                }
                break;
            }

            case TrackingMode::MultiWindowTracking:
            {
                // Get central head
                const auto& central_head = getCentralHead(agent_id);

                // Get number of tracking windows
                params.n = tracking.num_windows;

                // Set window and projection parameters for each tracking window
                params.window_params.resize(params.n);
                params.projection_params.resize(params.n);
                for (int i = 0; i < params.n; i++)
                {
                    params.window_params[i] = getWindowParameters(agent_id, central_head->id);
                    params.projection_params[i] = getProjectionParameters(agent_id, params.window_params[i].camera_params);
                }
                break;
            }
            }

            return params;
        }

    private: // Parameter parsing
        void parseParameters(MissionConfigPtr& config_ptr)
        {
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
            config_ptr->system.draw_rviz = RosUtils::getParameter<bool>(node_, "simulation.draw_rviz");
            config_ptr->system.record_metrics = RosUtils::getParameter<bool>(node_, "simulation.record_metrics");

            // Path settings
            config_ptr->system.config_source_file = RosUtils::getParameter<std::string>(node_, "path.config_spreadsheet_path");
            config_ptr->system.airsim_settings_destination_file = RosUtils::getParameter<std::string>(node_, "path.airsim_settings_path");

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

            // Central view settings
            config_ptr->system.central_view_id = RosUtils::getParameter<ID>(node_, "gui.central_view_id");

            // Tracking view settings
            config_ptr->system.tracking_view_ids = RosUtils::getParameter<std::vector<ID>>(node_, "gui.tracking_view_ids");
        }

        void parseTopicParameters(MissionConfigPtr& config_ptr)
        {
            // Global topics
            config_ptr->topics.registration = RosUtils::getParameter<std::string>(node_, "global_topics.registration");
            config_ptr->topics.global_metrics = RosUtils::getParameter<std::string>(node_, "global_topics.metrics");

            // Agent topics
            config_ptr->topics.agent_global_odom = RosUtils::getParameter<std::string>(node_, "agent_topics.global_odom");
            config_ptr->topics.agent_position_goal = RosUtils::getParameter<std::string>(node_, "agent_topics.position_goal");
            config_ptr->topics.agent_tracking_info = RosUtils::getParameter<std::string>(node_, "agent_topics.tracking_info");
            config_ptr->topics.agent_tracking_goal = RosUtils::getParameter<std::string>(node_, "agent_topics.tracking_goal");
            config_ptr->topics.agent_metrics = RosUtils::getParameter<std::string>(node_, "agent_topics.metrics");
            config_ptr->topics.agent_markers = RosUtils::getParameter<std::string>(node_, "agent_topics.markers");

            // Target topics
            config_ptr->topics.target_info = RosUtils::getParameter<std::string>(node_, "target_topics.info");
            config_ptr->topics.target_metrics = RosUtils::getParameter<std::string>(node_, "target_topics.metrics");
            config_ptr->topics.target_markers = RosUtils::getParameter<std::string>(node_, "target_topics.markers");

            // Cluster topics
            config_ptr->topics.cluster_info = RosUtils::getParameter<std::string>(node_, "cluster_topics.info");
            config_ptr->topics.cluster_metrics = RosUtils::getParameter<std::string>(node_, "cluster_topics.metrics");
            config_ptr->topics.cluster_markers = RosUtils::getParameter<std::string>(node_, "cluster_topics.markers");
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