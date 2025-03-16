#pragma once

// Config includes
#include "flychams_core/config/config_parser.hpp"
#include "flychams_core/config/config_types.hpp"

// Core includes
#include "flychams_core/utils/ros_utils.hpp"

namespace flychams::core
{
    /**
     * ════════════════════════════════════════════════════════════════
     * @brief Config Manager for handling configuration parsing and storing
     *
     * @details
     * This class provides utilities for managing the configuration of the
     * FlyChams system. It also provides utilities for parsing the configuration
     * into AirSim settings.json.
     * ════════════════════════════════════════════════════════════════
     * @author Jose Francisco Lopez Ruiz
     * @date 2025-02-28
     * ════════════════════════════════════════════════════════════════
     */
    class ConfigTools
    {
    public: // Types
        using SharedPtr = std::shared_ptr<ConfigTools>;

    private: // Data
        // Configuration
        ConfigPtr config_ptr_;

        // ROS components
        NodePtr node_;

    public: // Constructor/Destructor
        ConfigTools(NodePtr node)
            : node_(node)
        {
            // Parse the configuration file
            const std::string& config_path = RosUtils::getParameter<std::string>(node_, "config_source_file");
            try
            {
                config_ptr_ = ConfigParser::parseExcelFile(config_path);
            }
            catch (const std::exception& e)
            {
                RCLCPP_ERROR(node_->get_logger(), "Error parsing config file: %s", e.what());
                rclcpp::shutdown();
            }
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

    public: // AirSim utilities
        void createAirsimSettings() const
        {
            const std::string& airsim_path = RosUtils::getParameter<std::string>(node_, "airsim_settings_destination_file");
            if (!ConfigParser::parseAirsimSettings(config_ptr_, airsim_path))
            {
                RCLCPP_ERROR(node_->get_logger(), "Failed to create AirSim settings.json");
                rclcpp::shutdown();
            }
            else
            {
                RCLCPP_INFO(node_->get_logger(), "AirSim settings.json created successfully");
            }
        }

    public: // Raw getter methods

        const ConfigPtr getConfig() const
        {
            return config_ptr_;
        }

        const MissionConfigPtr getMission() const
        {
            return config_ptr_->mission;
        }

        const SimulationConfigPtr getSimulation() const
        {
            return config_ptr_->simulation;
        }

        const MapConfigPtr getMap() const
        {
            return config_ptr_->map;
        }

        const GroupConfigMap getGroups() const
        {
            return config_ptr_->groups;
        }

        const GroupConfigPtr getGroup(const std::string& group_id) const
        {
            return config_ptr_->groups.at(group_id);
        }

        const AgentConfigMap getAgents() const
        {
            return config_ptr_->agents;
        }

        const AgentConfigPtr getAgent(const std::string& agent_id) const
        {
            return config_ptr_->agents.at(agent_id);
        }

        const HeadConfigPtr getHead(const std::string& agent_id, const std::string& head_id) const
        {
            return config_ptr_->agents.at(agent_id)->heads.at(head_id);
        }

        const DroneConfigPtr getDrone(const std::string& agent_id) const
        {
            return config_ptr_->agents.at(agent_id)->drone;
        }

        const GimbalConfigPtr getGimbal(const std::string& agent_id, const std::string& head_id) const
        {
            return config_ptr_->agents.at(agent_id)->heads.at(head_id)->gimbal;
        }

        const GimbalLinkConfigPtr getGimbalLink(const std::string& agent_id, const std::string& head_id, const std::string& link_id) const
        {
            return config_ptr_->agents.at(agent_id)->heads.at(head_id)->gimbal->links.at(link_id);
        }

        const CameraConfigPtr getCamera(const std::string& agent_id, const std::string& head_id) const
        {
            return config_ptr_->agents.at(agent_id)->heads.at(head_id)->camera;
        }

    public: // Config utilities
        CameraParameters getCameraParameters(const std::string& agent_id, const std::string& head_id) const
        {
            CameraParameters params;

            // Extract head config
            const auto& head_ptr = getHead(agent_id, head_id);
            const auto& camera_ptr = getCamera(agent_id, head_id);

            // Camera ID
            params.id = head_id;

            // Camera focal length limits (m)
            params.f_min = head_ptr->min_focal;
            params.f_max = head_ptr->max_focal;
            params.f_ref = camera_ptr->default_focal;

            // Camera resolution (pix)
            params.width = camera_ptr->resolution(0);
            params.height = camera_ptr->resolution(1);

            // Camera sensor dimensions (m)
            params.sensor_width = camera_ptr->sensor_width;
            params.sensor_height = camera_ptr->sensor_height;

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
            // RCLCPP_INFO(node_->get_logger(), "Camera parameters for agent %s, head %s:", agent_id.c_str(), head_id.c_str());
            // RCLCPP_INFO(node_->get_logger(), "  ID: %s", params.id.c_str());
            // RCLCPP_INFO(node_->get_logger(), "  Focal lengths: min=%.3f, max=%.3f, ref=%.3f [m]", params.f_min, params.f_max, params.f_ref);
            // RCLCPP_INFO(node_->get_logger(), "  Resolution: %d x %d [pix]", params.width, params.height);
            // RCLCPP_INFO(node_->get_logger(), "  Sensor dimensions: %.6f x %.6f [m]", params.sensor_width, params.sensor_height);
            // RCLCPP_INFO(node_->get_logger(), "  Regularized pixel size: %.6f [m/pix]", params.rho);
            // RCLCPP_INFO(node_->get_logger(), "  Intrinsic matrix K: fx=%f fy=%f cx=%f cy=%f", params.k_ref(0, 0), params.k_ref(1, 1), params.k_ref(0, 2), params.k_ref(1, 2));

            return params;
        }

        WindowParameters getWindowParameters(const std::string& agent_id, const std::string& source_head_id) const
        {
            WindowParameters params;

            // Extract source head and mission config
            params.camera_params = getCameraParameters(agent_id, source_head_id);
            const auto& mission_ptr = getMission();

            // Full resolution (pix)
            params.full_width = params.camera_params.width;
            params.full_height = params.camera_params.height;

            // Scene resolution (pix)
            params.scene_width = mission_ptr->tracking_scene_resolution(0);
            params.scene_height = mission_ptr->tracking_scene_resolution(1);

            // View resolution (pix)
            params.view_width = mission_ptr->tracking_view_resolution(0);
            params.view_height = mission_ptr->tracking_view_resolution(1);

            // Calculate window parameters
            params.lambda_min = static_cast<float>(params.scene_width) / static_cast<float>(params.full_width);
            params.lambda_max = 1.0f;
            params.lambda_ref = (params.lambda_max + params.lambda_min) / 2.0f; // Middle of the range

            // Print window parameters for debugging
            // RCLCPP_INFO(node_->get_logger(), "Window parameters for agent %s, source head %s:", agent_id.c_str(), source_head_id.c_str());
            // RCLCPP_INFO(node_->get_logger(), "  Full resolution: %d x %d", params.full_width, params.full_height);
            // RCLCPP_INFO(node_->get_logger(), "  Scene resolution: %d x %d", params.scene_width, params.scene_height);
            // RCLCPP_INFO(node_->get_logger(), "  View resolution: %d x %d", params.view_width, params.view_height);
            // RCLCPP_INFO(node_->get_logger(), "  Lambda values: min=%.3f, max=%.3f, ref=%.3f", params.lambda_min, params.lambda_max, params.lambda_ref);

            return params;
        }

        ProjectionParameters getProjectionParameters(int width, int height, float rho) const
        {
            ProjectionParameters params;

            // Extract ROI parameters
            const auto& s_min_pix = RosUtils::getParameterOr<float>(node_, "tracking.s_min_pix", 50.0f); // [pix]
            const auto& kappa_s = RosUtils::getParameterOr<float>(node_, "tracking.kappa_s", 0.8f);

            // Maximum admissible apparent size of the object in the image (in pixels)
            // Assuming 90% of the half-height (or width if smaller)
            float s_max_pix = 0.5f * static_cast<float>(std::min(width, height)) * 0.9f;
            params.s_max_pix = s_max_pix; // [pix]

            // Minimum admissible apparent size (in pixels)
            // Provided externally
            params.s_min_pix = s_min_pix; // [pix]

            // Reference apparent size (in pixels)
            // Calculated using kappaS (half-width fraction) parameter
            float s_ref_pix = s_max_pix * kappa_s + s_min_pix * (1.0f - kappa_s); // [pix]
            params.s_ref_pix = s_ref_pix; // [pix]

            // Conversion to metric distances on the sensor surface
            params.s_max = s_max_pix * rho; // [m]
            params.s_min = s_min_pix * rho; // [m]
            params.s_ref = s_ref_pix * rho; // [m]

            // Print parameters for debugging
            // RCLCPP_INFO(node_->get_logger(), "Projection parameters:");
            // RCLCPP_INFO(node_->get_logger(), "  s_min_pix: %.2f [pix]", params.s_min_pix);
            // RCLCPP_INFO(node_->get_logger(), "  s_max_pix: %.2f [pix]", params.s_max_pix);
            // RCLCPP_INFO(node_->get_logger(), "  s_ref_pix: %.2f [pix]", params.s_ref_pix);
            // RCLCPP_INFO(node_->get_logger(), "  s_min: %.6f [m]", params.s_min);
            // RCLCPP_INFO(node_->get_logger(), "  s_max: %.6f [m]", params.s_max);
            // RCLCPP_INFO(node_->get_logger(), "  s_ref: %.6f [m]", params.s_ref);

            return params;
        }

        TrackingParameters getTrackingParameters(const std::string& agent_id)
        {
            TrackingParameters params;

            // Get agent config
            const auto& agent_ptr = getAgent(agent_id);
            params.mode = agent_ptr->tracking_mode;

            // Proceed based on tracking mode
            switch (params.mode)
            {
            case TrackingMode::MultiCameraTracking:
                // Get number of tracking heads
                params.n = static_cast<int>(agent_ptr->tracking_head_ids.size());

                // Set camera and projection parameters
                params.camera_params.resize(params.n);
                params.projection_params.resize(params.n);
                for (int i = 0; i < params.n; i++)
                {
                    params.camera_params[i] = getCameraParameters(agent_id, agent_ptr->tracking_head_ids[i]);
                    params.projection_params[i] = getProjectionParameters(
                        params.camera_params[i].width, params.camera_params[i].height, params.camera_params[i].rho);
                }
                break;

            case TrackingMode::MultiWindowTracking:
                // Get number of tracking windows
                params.n = RosUtils::getParameterOr<int>(node_, "tracking.num_tracking_windows", 4);

                // Set window and projection parameters
                params.window_params.resize(params.n);
                params.projection_params.resize(params.n);
                for (int i = 0; i < params.n; i++)
                {
                    params.window_params[i] = getWindowParameters(agent_id, agent_ptr->central_head_id);
                    // Calculate projection parameters for view image
                    params.projection_params[i] = getProjectionParameters(
                        params.window_params[i].view_width, params.window_params[i].view_height, params.window_params[i].camera_params.rho);
                }
                break;
            }

            return params;
        }
    };

} // namespace flychams::core