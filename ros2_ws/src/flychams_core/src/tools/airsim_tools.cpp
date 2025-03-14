#include "flychams_core/tools/airsim_tools.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;
using namespace airsim_interfaces::msg;
using namespace airsim_interfaces::srv;

namespace flychams::core
{
    // ════════════════════════════════════════════════════════════════════════════
    // CONSTRUCTOR: Constructor and destructor
    // ════════════════════════════════════════════════════════════════════════════

    AirsimTools::AirsimTools(NodePtr node)
        : node_(node)
    {
        // Initialize ROS components
        // Global commands
        reset_client_ = node_->create_client<Reset>("/airsim/reset");
        run_client_ = node_->create_client<Run>("/airsim/run");
        pause_client_ = node_->create_client<Pause>("/airsim/pause");
        // Window commands
        window_image_cmd_group_pub_ = node_->create_publisher<WindowImageCmdGroup>("/airsim/group_of_windows/image_cmd", 10);
        window_rectangle_cmd_pub_ = node_->create_publisher<WindowRectangleCmd>("/airsim/group_of_windows/rectangle_cmd", 10);
        window_string_cmd_pub_ = node_->create_publisher<WindowStringCmd>("/airsim/group_of_windows/string_cmd", 10);
        // Tracking commands
        add_target_group_client_ = node_->create_client<AddTargetGroup>("/airsim/group_of_targets/add");
        add_cluster_group_client_ = node_->create_client<AddClusterGroup>("/airsim/group_of_clusters/add");
        update_target_cmd_group_pub_ = node_->create_publisher<UpdateTargetCmdGroupMsg>("/airsim/group_of_targets/update_cmd", 10);
        update_cluster_cmd_group_pub_ = node_->create_publisher<UpdateClusterCmdGroupMsg>("/airsim/group_of_clusters/update_cmd", 10);
    }

    AirsimTools::~AirsimTools()
    {
        shutdown();
    }

    void AirsimTools::shutdown()
    {
        // Destroy clients
        reset_client_.reset();
        run_client_.reset();
        pause_client_.reset();
        add_target_group_client_.reset();
        add_cluster_group_client_.reset();
        // Destroy publisher
        gimbal_angle_cmd_pub_map_.clear();
        camera_fov_cmd_pub_map_.clear();
        window_image_cmd_group_pub_.reset();
        window_rectangle_cmd_pub_.reset();
        window_string_cmd_pub_.reset();
        update_target_cmd_group_pub_.reset();
        update_cluster_cmd_group_pub_.reset();
        // Destroy node
        node_.reset();
    }

    // ════════════════════════════════════════════════════════════════════════════
    // VEHICLE ADDERS: Vehicle adders
    // ════════════════════════════════════════════════════════════════════════════

    void AirsimTools::addVehicle(const ID& vehicle_id)
    {
        // Create publishers
        gimbal_angle_cmd_pub_map_[vehicle_id] = node_->create_publisher<GimbalAngleCmd>("/airsim/" + vehicle_id + "/gimbal_angle_cmd", 10);
        camera_fov_cmd_pub_map_[vehicle_id] = node_->create_publisher<CameraFovCmd>("/airsim/" + vehicle_id + "/camera_fov_cmd", 10);
    }

    void AirsimTools::removeVehicle(const ID& vehicle_id)
    {
        // Destroy publishers
        gimbal_angle_cmd_pub_map_.erase(vehicle_id);
        camera_fov_cmd_pub_map_.erase(vehicle_id);
    }

    // ════════════════════════════════════════════════════════════════════════════
    // GLOBAL CONTROL
    // ════════════════════════════════════════════════════════════════════════════

    bool AirsimTools::resetSimulation()
    {
        // Create request
        auto request = std::make_shared<Reset::Request>();

        // Send request and wait for response
        return RosUtils::sendRequestSync<Reset>(node_, reset_client_, request, 1000);
    }

    bool AirsimTools::runSimulation()
    {
        // Create request
        auto request = std::make_shared<Run::Request>();

        // Send request and wait for response
        return RosUtils::sendRequestSync<Run>(node_, run_client_, request, 1000);
    }

    bool AirsimTools::pauseSimulation()
    {
        // Create request
        auto request = std::make_shared<Pause::Request>();

        // Send request and wait for response
        return RosUtils::sendRequestSync<Pause>(node_, pause_client_, request, 1000);
    }

    // ════════════════════════════════════════════════════════════════════════════
    // VEHICLE CONTROL
    // ════════════════════════════════════════════════════════════════════════════

    void AirsimTools::setGimbalAngles(const ID& vehicle_id, const IDs& camera_ids, const std::vector<QuaternionMsg>& target_quats, const std::string& frame_id)
    {
        // Create message
        GimbalAngleCmd msg;
        msg.header = RosUtils::createHeader(node_, frame_id);
        msg.camera_names = camera_ids;
        msg.orientations = target_quats;

        // Publish message
        gimbal_angle_cmd_pub_map_[vehicle_id]->publish(msg);
    }

    void AirsimTools::setCameraFovs(const ID& vehicle_id, const IDs& camera_ids, const std::vector<float>& target_fovs, const std::string& frame_id)
    {
        // Create message
        CameraFovCmd msg;
        msg.header = RosUtils::createHeader(node_, frame_id);
        msg.camera_names = camera_ids;
        msg.fovs = target_fovs;

        // Publish message
        camera_fov_cmd_pub_map_[vehicle_id]->publish(msg);
    }

    // ════════════════════════════════════════════════════════════════════════════
    // WINDOW CONTROL
    // ════════════════════════════════════════════════════════════════════════════

    void AirsimTools::setWindowImageGroup(const IDs& window_ids, const IDs& vehicle_ids, const IDs& camera_ids, const std::vector<int>& crop_x, const std::vector<int>& crop_y, const std::vector<int>& crop_w, const std::vector<int>& crop_h)
    {
        // Create message
        WindowImageCmdGroup msg;
        msg.window_indices = getWindowIndices(window_ids);
        msg.vehicle_names = vehicle_ids;
        msg.camera_names = camera_ids;
        for (size_t i = 0; i < window_ids.size(); i++)
        {
            PointMsg corner;
            corner.x = crop_x[i];
            corner.y = crop_y[i];
            msg.corners.push_back(corner);
            PointMsg size;
            size.x = crop_w[i];
            size.y = crop_h[i];
            msg.sizes.push_back(size);
        }

        // Publish message
        window_image_cmd_group_pub_->publish(msg);
    }

    void AirsimTools::setWindowRectangles(const ID& window_id, const std::vector<PointMsg>& corners, const std::vector<PointMsg>& sizes, const ColorMsg& color, const float& thickness)
    {
        // Create message
        WindowRectangleCmd msg;
        msg.window_index = getWindowIndex(window_id);
        msg.corners = corners;
        msg.sizes = sizes;
        msg.color = color;
        msg.thickness = thickness;

        // Publish message
        window_rectangle_cmd_pub_->publish(msg);
    }

    void AirsimTools::setWindowStrings(const ID& window_id, const std::vector<std::string>& strings, const std::vector<PointMsg>& positions, const ColorMsg& color, const float& scale)
    {
        // Create message
        WindowStringCmd msg;
        msg.window_index = getWindowIndex(window_id);
        msg.strings = strings;
        msg.positions = positions;
        msg.color = color;
        msg.scale = scale;

        // Publish message
        window_string_cmd_pub_->publish(msg);
    }

    // ════════════════════════════════════════════════════════════════════════════
    // TRACKING CONTROL
    // ════════════════════════════════════════════════════════════════════════════

    bool AirsimTools::addTargetGroup(const IDs& target_ids, const std::vector<TargetType>& target_types, const std::vector<PointMsg>& positions, const bool& highlight, const std::vector<ColorMsg>& highlight_colors)
    {
        // Create request
        auto request = std::make_shared<AddTargetGroup::Request>();
        request->target_names = target_ids;
        request->positions = positions;
        request->highlight = highlight;
        request->highlight_color_rgba = highlight_colors;

        for (size_t i = 0; i < target_ids.size(); ++i)
        {
            // Set target type based on target type
            switch (target_types[i])
            {
            case TargetType::Cube:
                request->target_types.push_back("Cube");
                break;
            case TargetType::Human:
                request->target_types.push_back("Human");
                break;
            default:
                RCLCPP_ERROR(node_->get_logger(), "Unknown target type: %d", static_cast<int>(target_types[i]));
                request->target_types.push_back("Cube");
                break;
            }
        }

        // Send request and wait for response
        return RosUtils::sendRequestSync<AddTargetGroup>(node_, add_target_group_client_, request, 100000);
    }

    bool AirsimTools::addClusterGroup(const IDs& cluster_ids, const std::vector<PointMsg>& centers, const std::vector<float>& radii, const bool& highlight, const std::vector<ColorMsg>& highlight_colors)
    {
        // Create request
        auto request = std::make_shared<AddClusterGroup::Request>();
        request->cluster_names = cluster_ids;
        request->centers = centers;
        request->radii = radii;
        request->highlight = highlight;
        request->highlight_color_rgba = highlight_colors;

        // Send request and wait for response
        return RosUtils::sendRequestSync<AddClusterGroup>(node_, add_cluster_group_client_, request, 100000);
    }

    // ════════════════════════════════════════════════════════════════════════════
    // OBJECT CONTROL
    // ════════════════════════════════════════════════════════════════════════════

    void AirsimTools::updateTargetGroup(const IDs& target_ids, const std::vector<PointMsg>& positions)
    {
        // Create message
        UpdateTargetCmdGroup msg;
        msg.target_names = target_ids;
        msg.positions = positions;

        // Publish message
        update_target_cmd_group_pub_->publish(msg);
    }

    void AirsimTools::updateClusterGroup(const IDs& cluster_ids, const std::vector<PointMsg>& centers, const std::vector<float>& radii)
    {
        // Create message
        UpdateClusterCmdGroup msg;
        msg.cluster_names = cluster_ids;
        msg.centers = centers;
        msg.radii = radii;

        // Publish message
        update_cluster_cmd_group_pub_->publish(msg);
    }

} // namespace flychams::core