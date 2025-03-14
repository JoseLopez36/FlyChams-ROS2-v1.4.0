#include <airsim_wrapper.h>

using namespace std::placeholders;

namespace airsim_wrapper
{
    // ════════════════════════════════════════════════════════════════════════════
    // CONSTRUCTOR/DESTRUCTOR
    // ════════════════════════════════════════════════════════════════════════════

    AirsimWrapper::AirsimWrapper(const std::shared_ptr<rclcpp::Node> nh, const std::string& host_ip, uint16_t host_port, bool enable_world_plot)
        : airsim_settings_parser_(host_ip, host_port)
        , host_ip_(host_ip)
        , host_port_(host_port)
        , enable_world_plot_(enable_world_plot)
        , airsim_client_state_(nullptr)
        , airsim_client_control_(nullptr)
        , airsim_client_window_(nullptr)
        , airsim_client_tracking_(nullptr)

        , nh_(nh)
    {
        ros_clock_.clock = rclcpp::Time(0);

        if (AirSimSettings::singleton().simmode_name != AirSimSettings::kSimModeTypeMultirotor) {
            RCLCPP_ERROR(nh_->get_logger(), "Unsupported simulation mode: %s", AirSimSettings::singleton().simmode_name.c_str());
            rclcpp::shutdown();
            return;
        }

        // Create TF broadcaster and static TF broadcaster
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(nh_);
        static_tf_pub_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(nh_);

        // Initialize AirSim clients
        RCLCPP_INFO(nh_->get_logger(), "Initializing AirSim clients...");
        try
        {
            initialize_airsim();
        }
        catch (rpc::rpc_error& e) {
            std::string msg = e.get_error().as<std::string>();
            RCLCPP_ERROR(nh_->get_logger(), "Exception raised by the API:\n%s", msg.c_str());
            rclcpp::shutdown();
            return;
        }

        // Initialize ROS data and communications
        initialize_ros();

        // Initialize simulation state (UAVs armed in the air, windows emptied, no targets or clusters)
        try
        {
            // Enable control of all vehicles
            RCLCPP_INFO(nh_->get_logger(), "Enabling control of all vehicles...");
            client_enable_control();

            // Continue simulation for takeoff
            client_pause(false);
            for (const auto& [vehicle_name, _] : vehicle_map_)
            {
                RCLCPP_INFO(nh_->get_logger(), "Taking off vehicle %s...", vehicle_name.c_str());
                client_takeoff(3.0f, vehicle_name, false);
            }

            // Set empty cameras for all windows
            std::vector<int> window_indices = { 0, 1, 2, 3, 4, 5, 6, 7, 8 };
            std::vector<std::string> vehicle_names(8, "");
            std::vector<std::string> camera_names(8, "");
            std::vector<msr::airlib::Vector2r> corners(8, msr::airlib::Vector2r(0.0f, 0.0f));
            std::vector<msr::airlib::Vector2r> sizes(8, msr::airlib::Vector2r(0.0f, 0.0f));
            client_set_window_images(window_indices, vehicle_names, camera_names, corners, sizes);

            // Remove all targets and clusters from previous runs
            client_remove_all_targets();
            client_remove_all_clusters();
        }
        catch (rpc::rpc_error& e) {
            std::string msg = e.get_error().as<std::string>();
            RCLCPP_ERROR(nh_->get_logger(), "Exception raised by the API:\n%s", msg.c_str());
            rclcpp::shutdown();
            return;
        }

        // Publish AirSim status
        RCLCPP_WARN(nh_->get_logger(), "AirsimWrapper successfully initialized!");

        // Create timers
        nh_->get_parameter("update_airsim_state_every_n_sec", update_airsim_state_every_n_sec_);
        airsim_state_update_timer_ = nh_->create_wall_timer(std::chrono::duration<double>(update_airsim_state_every_n_sec_), std::bind(&AirsimWrapper::drone_state_timer_cb, this), cb_state_);
        nh_->get_parameter("update_sim_clock_every_n_sec", update_sim_clock_every_n_sec_);
        sim_clock_update_timer_ = nh_->create_wall_timer(std::chrono::duration<double>(update_sim_clock_every_n_sec_), std::bind(&AirsimWrapper::clock_timer_cb, this), cb_state_);
    }

    AirsimWrapper::~AirsimWrapper()
    {
        // Shutdown AirSim
        shutdown();
    }

    void AirsimWrapper::shutdown()
    {
        // Destroy timers
        airsim_state_update_timer_.reset();
        sim_clock_update_timer_.reset();
    }

    // ════════════════════════════════════════════════════════════════════════════
    // INITIALIZATION
    // ════════════════════════════════════════════════════════════════════════════

    void AirsimWrapper::initialize_airsim()
    {
        // Create airsim clients
        airsim_client_state_ = std::unique_ptr<msr::airlib::MultirotorRpcLibClient>(new msr::airlib::MultirotorRpcLibClient(host_ip_, host_port_));
        airsim_client_control_ = std::unique_ptr<msr::airlib::MultirotorRpcLibClient>(new msr::airlib::MultirotorRpcLibClient(host_ip_, host_port_));
        airsim_client_window_ = std::unique_ptr<msr::airlib::RpcLibClientBase>(new msr::airlib::RpcLibClientBase(host_ip_, host_port_));
        airsim_client_tracking_ = std::unique_ptr<msr::airlib::RpcLibClientBase>(new msr::airlib::RpcLibClientBase(host_ip_, host_port_));

        // Connect to AirSim in parallel
        std::vector<std::future<void>> futures;
        futures.push_back(std::async(std::launch::async, [this]() {
            airsim_client_state_->confirmConnection();
            }));

        futures.push_back(std::async(std::launch::async, [this]() {
            airsim_client_control_->confirmConnection();
            }));

        futures.push_back(std::async(std::launch::async, [this]() {
            airsim_client_window_->confirmConnection();
            }));

        futures.push_back(std::async(std::launch::async, [this]() {
            airsim_client_tracking_->confirmConnection();
            }));

        // Wait for all clients to connect
        for (auto& future : futures)
        {
            future.get();
        }

        // Initialize simulation clock
        ros_clock_.clock = client_get_timestamp();

        // Pause and reset simulation as a starting state
        client_pause(true);
        client_reset();
    }

    void AirsimWrapper::initialize_ros()
    {
        // ROS params
        nh_->get_parameter_or("global_frame_id", global_frame_id_, global_frame_id_);
        nh_->get_parameter_or("local_frame_id", local_frame_id_, local_frame_id_);
        nh_->get_parameter_or("odom_frame_id", odom_frame_id_, odom_frame_id_);
        nh_->get_parameter_or("camera_body_frame_id", camera_body_frame_id_, camera_body_frame_id_);
        nh_->get_parameter_or("camera_optical_frame_id", camera_optical_frame_id_, camera_optical_frame_id_);

        // Initialize callback groups
        cb_state_ = nh_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        cb_control_ = nh_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        cb_window_ = nh_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        cb_tracking_ = nh_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        // Create ROS communications
        create_ros_comms_from_settings_json();
    }

    void AirsimWrapper::create_ros_comms_from_settings_json()
    {
        RCLCPP_INFO(nh_->get_logger(), "Creating ROS communications...");

        // Get control subscriber options
        auto control_sub_options = rclcpp::SubscriptionOptions();
        control_sub_options.callback_group = cb_control_;

        // Get window subscriber options
        auto window_sub_options = rclcpp::SubscriptionOptions();
        window_sub_options.callback_group = cb_window_;

        // Get tracking subscriber options
        auto tracking_sub_options = rclcpp::SubscriptionOptions();
        tracking_sub_options.callback_group = cb_tracking_;

        // Initialize vehicle map from settings
        RCLCPP_INFO(nh_->get_logger(), "Initializing vehicle map from settings...");
        // Iterate over vehicles
        vehicle_map_.clear();
        for (const auto& [vehicle_name, vehicle_setting] : AirSimSettings::singleton().vehicles)
        {
            RCLCPP_INFO(nh_->get_logger(), "Initializing vehicle %s from settings...", vehicle_name.c_str());
            set_nans_to_zeros_in_pose(*vehicle_setting);

            // Initialize vehicle ROS instance
            std::unique_ptr<VehicleROS> vehicle_ros = std::unique_ptr<VehicleROS>(new VehicleROS());
            vehicle_ros->vehicle_name = vehicle_name;
            vehicle_ros->vehicle_setting = *vehicle_setting;
            vehicle_ros->local_frame_id = vehicle_name + "/" + local_frame_id_;
            vehicle_ros->odom_frame_id = vehicle_name + "/" + odom_frame_id_;
            initialize_vehicle_tf(vehicle_ros.get(), *vehicle_setting);

            // Initialize vehicle subscribers
            vehicle_ros->gimbal_angle_cmd_sub = nh_->create_subscription<airsim_interfaces::msg::GimbalAngleCmd>(
                vehicle_name + "/gimbal_angle_cmd", 10,
                [this, vehicle_name](const airsim_interfaces::msg::GimbalAngleCmd::SharedPtr msg) {
                    this->gimbal_angle_cmd_cb(vehicle_name, msg);
                },
                control_sub_options
            );
            vehicle_ros->camera_fov_cmd_sub = nh_->create_subscription<airsim_interfaces::msg::CameraFovCmd>(
                vehicle_name + "/camera_fov_cmd", 10,
                [this, vehicle_name](const airsim_interfaces::msg::CameraFovCmd::SharedPtr msg) {
                    this->camera_fov_cmd_cb(vehicle_name, msg);
                },
                control_sub_options
            );

            // Iterate over camera map
            vehicle_ros->camera_map.clear();
            for (auto& [camera_name, camera_setting] : vehicle_setting->cameras)
            {
                RCLCPP_INFO(nh_->get_logger(), "Initializing camera %s from settings...", camera_name.c_str());
                set_nans_to_zeros_in_pose(*vehicle_setting, camera_setting);

                // Initialize camera ROS instance
                auto camera_ros = std::unique_ptr<CameraROS>(new CameraROS());
                camera_ros->camera_name = camera_name;
                camera_ros->camera_setting = camera_setting;
                camera_ros->body_frame_id = vehicle_name + "/" + camera_name + "/" + camera_body_frame_id_;
                camera_ros->optical_frame_id = vehicle_name + "/" + camera_name + "/" + camera_optical_frame_id_;
                initialize_camera_tf(vehicle_ros.get(), camera_ros.get(), camera_setting);

                // Add camera to vehicle's camera map
                vehicle_ros->camera_map.emplace(camera_name, std::move(camera_ros));
            }

            // Add vehicle to map
            vehicle_map_.emplace(vehicle_name, std::move(vehicle_ros));
        }

        // Create clock publisher
        clock_pub_ = nh_->create_publisher<rosgraph_msgs::msg::Clock>("/clock", 1);
        // Create global control services
        reset_srvr_ = nh_->create_service<airsim_interfaces::srv::Reset>("reset", std::bind(&AirsimWrapper::reset_srv_cb, this, _1, _2));
        run_srvr_ = nh_->create_service<airsim_interfaces::srv::Run>("run", std::bind(&AirsimWrapper::run_srv_cb, this, _1, _2));
        pause_srvr_ = nh_->create_service<airsim_interfaces::srv::Pause>("pause", std::bind(&AirsimWrapper::pause_srv_cb, this, _1, _2));
        // Create group of windows subscribers
        window_image_cmd_group_sub_ = nh_->create_subscription<airsim_interfaces::msg::WindowImageCmdGroup>(
            "group_of_windows/image_cmd", 10, std::bind(&AirsimWrapper::window_image_cmd_group_cb, this, _1), window_sub_options);
        window_rectangle_cmd_sub_ = nh_->create_subscription<airsim_interfaces::msg::WindowRectangleCmd>(
            "group_of_windows/rectangle_cmd", 10, std::bind(&AirsimWrapper::window_rectangle_cmd_cb, this, _1), window_sub_options);
        window_string_cmd_sub_ = nh_->create_subscription<airsim_interfaces::msg::WindowStringCmd>(
            "group_of_windows/string_cmd", 10, std::bind(&AirsimWrapper::window_string_cmd_cb, this, _1), window_sub_options);
        // Create tracking services
        add_target_group_srvr_ = nh_->create_service<airsim_interfaces::srv::AddTargetGroup>("group_of_targets/add", std::bind(&AirsimWrapper::add_target_group_cb, this, _1, _2));
        add_cluster_group_srvr_ = nh_->create_service<airsim_interfaces::srv::AddClusterGroup>("group_of_clusters/add", std::bind(&AirsimWrapper::add_cluster_group_cb, this, _1, _2));
        // Create tracking subscribers
        update_target_cmd_group_sub_ = nh_->create_subscription<airsim_interfaces::msg::UpdateTargetCmdGroup>(
            "group_of_targets/update_cmd", 10, std::bind(&AirsimWrapper::update_target_cmd_group_cb, this, _1), tracking_sub_options);
        update_cluster_cmd_group_sub_ = nh_->create_subscription<airsim_interfaces::msg::UpdateClusterCmdGroup>(
            "group_of_clusters/update_cmd", 10, std::bind(&AirsimWrapper::update_cluster_cmd_group_cb, this, _1), tracking_sub_options);
    }

    void AirsimWrapper::initialize_vehicle_tf(VehicleROS* vehicle_ros, const VehicleSetting& vehicle_setting)
    {
        auto& local_tf = vehicle_ros->local_static_tf_msg;

        // World to Local transform
        local_tf.header.frame_id = global_frame_id_;
        local_tf.child_frame_id = vehicle_ros->local_frame_id;
        local_tf.header.stamp = get_sim_clock_time();
        local_tf.transform = get_transform_msg_from_airsim(vehicle_setting.position, vehicle_setting.rotation);
        static_tf_pub_->sendTransform(local_tf);
    }

    void AirsimWrapper::initialize_camera_tf(VehicleROS* vehicle_ros, CameraROS* camera_ros, const CameraSetting& camera_setting)
    {
        auto& body_tf = camera_ros->body_tf_msg;
        auto& optical_tf = camera_ros->optical_static_tf_msg;

        // World to Body transform
        if (camera_setting.external)
        {
            body_tf.header.frame_id = global_frame_id_;
        }
        else
        {
            body_tf.header.frame_id = vehicle_ros->local_frame_id;
        }
        body_tf.child_frame_id = camera_ros->body_frame_id;
        body_tf.header.stamp = get_sim_clock_time();
        auto camera_info_data = airsim_client_control_->simGetCameraInfo(camera_ros->camera_name, vehicle_ros->vehicle_name);
        body_tf.transform = get_transform_msg_from_airsim(camera_info_data.pose.position, camera_info_data.pose.orientation);
        tf_broadcaster_->sendTransform(body_tf);

        // Body to Optical transform
        optical_tf.header.frame_id = camera_ros->body_frame_id;
        optical_tf.child_frame_id = camera_ros->optical_frame_id;
        optical_tf.header.stamp = get_sim_clock_time();
        optical_tf.transform = get_camera_optical_tf();
        static_tf_pub_->sendTransform(optical_tf);
    }

    // ════════════════════════════════════════════════════════════════════════════
    // COMMAND CALLBACKS
    // ════════════════════════════════════════════════════════════════════════════

    void AirsimWrapper::gimbal_angle_cmd_cb(const std::string& vehicle_name, const airsim_interfaces::msg::GimbalAngleCmd::SharedPtr gimbal_angle_cmd_msg)
    {
        // Extract message data
        const auto& camera_names = gimbal_angle_cmd_msg->camera_names;
        const auto& orientations = gimbal_angle_cmd_msg->orientations;
        try
        {
            for (size_t i = 0; i < camera_names.size(); i++)
            {
                const auto& camera_name = camera_names[i];
                const auto& orientation = orientations[i];

                // Convert pose to airlib
                geometry_msgs::msg::Pose pose_msg;
                pose_msg.position.x = 0.0;
                pose_msg.position.y = 0.0;
                pose_msg.position.z = 0.0;
                pose_msg.orientation = orientation;
                msr::airlib::Pose pose = get_airlib_pose(pose_msg);

                // Send command to server
                client_set_camera_pose(camera_name, pose, vehicle_name);
            }
        }
        catch (rpc::rpc_error& e) {
            std::string msg = e.get_error().as<std::string>();
            RCLCPP_ERROR(nh_->get_logger(), "Exception raised by the API:\n%s", msg.c_str());
            return; // Stop execution of this callback
        }
    }

    void AirsimWrapper::camera_fov_cmd_cb(const std::string& vehicle_name, const airsim_interfaces::msg::CameraFovCmd::SharedPtr camera_fov_cmd_msg)
    {
        // Extract message data
        const auto& camera_names = camera_fov_cmd_msg->camera_names;
        const auto& fov_cmds = camera_fov_cmd_msg->fovs;
        try
        {
            for (size_t i = 0; i < camera_names.size(); i++)
            {
                const auto& camera_name = camera_names[i];
                const auto& fov_cmd = fov_cmds[i];

                // Send command to server
                client_set_camera_fov(camera_name, fov_cmd, vehicle_name);
            }
        }
        catch (rpc::rpc_error& e) {
            std::string msg = e.get_error().as<std::string>();
            RCLCPP_ERROR(nh_->get_logger(), "Exception raised by the API:\n%s", msg.c_str());
            return; // Stop execution of this callback
        }
    }

    void AirsimWrapper::window_image_cmd_group_cb(const airsim_interfaces::msg::WindowImageCmdGroup::SharedPtr window_image_cmd_group_msg)
    {
        // Extract request data
        const auto& window_indices = window_image_cmd_group_msg->window_indices;
        const auto& vehicle_names = window_image_cmd_group_msg->vehicle_names;
        const auto& camera_names = window_image_cmd_group_msg->camera_names;
        const auto& corners = window_image_cmd_group_msg->corners;
        const auto& sizes = window_image_cmd_group_msg->sizes;
        try
        {
            // Send command to server
            client_set_window_images(window_indices, vehicle_names, camera_names, get_airlib_points_2d(corners), get_airlib_points_2d(sizes));
        }
        catch (rpc::rpc_error& e) {
            std::string msg = e.get_error().as<std::string>();
            RCLCPP_ERROR(nh_->get_logger(), "Exception raised by the API:\n%s", msg.c_str());
            return; // Stop execution of this callback
        }
    }

    void AirsimWrapper::window_rectangle_cmd_cb(const airsim_interfaces::msg::WindowRectangleCmd::SharedPtr window_rectangle_cmd_msg)
    {
        // Extract request data
        const auto& window_index = window_rectangle_cmd_msg->window_index;
        const auto& corners = window_rectangle_cmd_msg->corners;
        const auto& sizes = window_rectangle_cmd_msg->sizes;
        const auto& color = window_rectangle_cmd_msg->color;
        const auto& thickness = window_rectangle_cmd_msg->thickness;
        try
        {
            // Send command to server
            client_set_window_rectangle(window_index, get_airlib_points_2d(corners), get_airlib_points_2d(sizes), get_airlib_color(color), thickness);
        }
        catch (rpc::rpc_error& e) {
            std::string msg = e.get_error().as<std::string>();
            RCLCPP_ERROR(nh_->get_logger(), "Exception raised by the API:\n%s", msg.c_str());
            return; // Stop execution of this callback
        }
    }

    void AirsimWrapper::window_string_cmd_cb(const airsim_interfaces::msg::WindowStringCmd::SharedPtr window_string_cmd_msg)
    {
        // Extract request data
        const auto& window_index = window_string_cmd_msg->window_index;
        const auto& strings = window_string_cmd_msg->strings;
        const auto& positions = window_string_cmd_msg->positions;
        const auto& scale = window_string_cmd_msg->scale;
        const auto& color = window_string_cmd_msg->color;
        try
        {
            // Send command to server
            client_set_window_strings(window_index, strings, get_airlib_points_2d(positions), get_airlib_color(color), scale);
        }
        catch (rpc::rpc_error& e) {
            std::string msg = e.get_error().as<std::string>();
            RCLCPP_ERROR(nh_->get_logger(), "Exception raised by the API:\n%s", msg.c_str());
            return; // Stop execution of this callback
        }
    }

    void AirsimWrapper::update_target_cmd_group_cb(const airsim_interfaces::msg::UpdateTargetCmdGroup::SharedPtr update_target_cmd_group_msg)
    {
        // Extract message data
        const auto& target_names = update_target_cmd_group_msg->target_names;
        const auto& positions = update_target_cmd_group_msg->positions;
        try
        {
            // Send command to server
            client_update_targets(target_names, get_airlib_points(positions));
        }
        catch (rpc::rpc_error& e) {
            std::string msg = e.get_error().as<std::string>();
            RCLCPP_ERROR(nh_->get_logger(), "Exception raised by the API:\n%s", msg.c_str());
            return; // Stop execution of this callback
        }
    }

    void AirsimWrapper::update_cluster_cmd_group_cb(const airsim_interfaces::msg::UpdateClusterCmdGroup::SharedPtr update_cluster_cmd_group_msg)
    {
        // Extract message data
        const auto& cluster_names = update_cluster_cmd_group_msg->cluster_names;
        const auto& centers = update_cluster_cmd_group_msg->centers;
        const auto& radii = update_cluster_cmd_group_msg->radii;
        try
        {
            // Send command to server
            client_update_clusters(cluster_names, get_airlib_points(centers), radii);
        }
        catch (rpc::rpc_error& e) {
            std::string msg = e.get_error().as<std::string>();
            RCLCPP_ERROR(nh_->get_logger(), "Exception raised by the API:\n%s", msg.c_str());
            return; // Stop execution of this callback
        }
    }

    // ════════════════════════════════════════════════════════════════════════════
    // COMMAND SERVICES
    // ════════════════════════════════════════════════════════════════════════════

    bool AirsimWrapper::reset_srv_cb(std::shared_ptr<airsim_interfaces::srv::Reset::Request> request, std::shared_ptr<airsim_interfaces::srv::Reset::Response> response)
    {
        RCLCPP_WARN(nh_->get_logger(), "Resetting AirSim");
        unused(request);

        // Pause and reset AirSim
        try
        {
            // Send command to server
            client_pause(true);
            client_reset();
            // We need to re-arm the vehicles after resetting
            client_enable_control();
        }
        catch (rpc::rpc_error& e) {
            std::string msg = e.get_error().as<std::string>();
            RCLCPP_ERROR(nh_->get_logger(), "Exception raised by the API:\n%s", msg.c_str());
            response->success = false;
            return false;
        }

        response->success = true;
        return true;
    }

    bool AirsimWrapper::run_srv_cb(std::shared_ptr<airsim_interfaces::srv::Run::Request> request, std::shared_ptr<airsim_interfaces::srv::Run::Response> response)
    {
        RCLCPP_WARN(nh_->get_logger(), "Running AirSim");
        unused(request);

        try
        {
            // Send command to server
            client_pause(false);
        }
        catch (rpc::rpc_error& e) {
            std::string msg = e.get_error().as<std::string>();
            RCLCPP_ERROR(nh_->get_logger(), "Exception raised by the API:\n%s", msg.c_str());
            response->success = false;
            return false;
        }

        response->success = true;
        return true;
    }

    bool AirsimWrapper::pause_srv_cb(std::shared_ptr<airsim_interfaces::srv::Pause::Request> request, std::shared_ptr<airsim_interfaces::srv::Pause::Response> response)
    {
        RCLCPP_WARN(nh_->get_logger(), "Pausing AirSim");
        unused(request);

        try
        {
            // Send command to server
            client_pause(true);
        }
        catch (rpc::rpc_error& e) {
            std::string msg = e.get_error().as<std::string>();
            RCLCPP_ERROR(nh_->get_logger(), "Exception raised by the API:\n%s", msg.c_str());
            response->success = false;
            return false;
        }

        response->success = true;
        return true;
    }

    bool AirsimWrapper::add_target_group_cb(const std::shared_ptr<airsim_interfaces::srv::AddTargetGroup::Request> request, const std::shared_ptr<airsim_interfaces::srv::AddTargetGroup::Response> response)
    {
        RCLCPP_INFO(nh_->get_logger(), "Adding group of targets");

        // Extract request data
        const auto& target_names = request->target_names;
        const auto& target_types = request->target_types;
        const auto& positions = request->positions;
        const auto& highlight = request->highlight;
        const auto& highlight_color_rgba = request->highlight_color_rgba;
        try
        {
            // Send command to server
            client_add_targets(target_names, target_types, get_airlib_points(positions), highlight, get_airlib_colors(highlight_color_rgba));
        }
        catch (rpc::rpc_error& e) {
            std::string msg = e.get_error().as<std::string>();
            RCLCPP_ERROR(nh_->get_logger(), "Exception raised by the API:\n%s", msg.c_str());
            response->success = false;
            return false;
        }

        response->success = true;
        return true;
    }

    bool AirsimWrapper::add_cluster_group_cb(const std::shared_ptr<airsim_interfaces::srv::AddClusterGroup::Request> request, const std::shared_ptr<airsim_interfaces::srv::AddClusterGroup::Response> response)
    {
        RCLCPP_INFO(nh_->get_logger(), "Adding group of clusters");

        // Extract request data
        const auto& cluster_names = request->cluster_names;
        const auto& centers = request->centers;
        const auto& radii = request->radii;
        const auto& highlight = request->highlight;
        const auto& highlight_color_rgba = request->highlight_color_rgba;
        try
        {
            // Send command to server
            client_add_clusters(cluster_names, get_airlib_points(centers), radii, highlight, get_airlib_colors(highlight_color_rgba));
        }
        catch (rpc::rpc_error& e) {
            std::string msg = e.get_error().as<std::string>();
            RCLCPP_ERROR(nh_->get_logger(), "Exception raised by the API:\n%s", msg.c_str());
            response->success = false;
            return false;
        }

        response->success = true;
        return true;
    }

    // ════════════════════════════════════════════════════════════════════════════
    // TIMER CALLBACKS
    // ════════════════════════════════════════════════════════════════════════════

    void AirsimWrapper::drone_state_timer_cb()
    {
        try
        {
            // Retrieve simulation state
            bool is_paused;
            rclcpp::Time curr_time;
            is_paused = client_get_paused();
            curr_time = get_sim_clock_time();

            // Update simulation state if not paused
            if (!is_paused)
            {
                // Iterate over drones
                for (auto& [vehicle_name, vehicle_ros] : vehicle_map_)
                {
                    // Iterate over cameras
                    for (auto& [camera_name, camera_ros] : vehicle_ros->camera_map)
                    {
                        // Retrieve camera info from server
                        const auto& camera_info_data = client_get_camera_info(vehicle_name, camera_name);

                        // Update camera info and tf
                        camera_ros->body_tf_msg.header.stamp = curr_time;
                        camera_ros->body_tf_msg.transform = get_transform_msg_from_airsim(camera_info_data.pose.position, camera_info_data.pose.orientation);
                    }
                }
            }

            // Publish simulation state
            publish_vehicle_state();
        }
        catch (rpc::rpc_error& e) {
            std::string msg = e.get_error().as<std::string>();
            RCLCPP_ERROR(nh_->get_logger(), "Exception raised by the API:\n%s", msg.c_str());
        }
    }

    void AirsimWrapper::clock_timer_cb()
    {
        try
        {
            // Retrieve timestamp from server
            ros_clock_.clock = client_get_timestamp();

            // Publish clock
            clock_pub_->publish(ros_clock_);
        }
        catch (rpc::rpc_error& e) {
            std::string msg = e.get_error().as<std::string>();
            RCLCPP_ERROR(nh_->get_logger(), "Exception raised by the API:\n%s", msg.c_str());
        }
    }

    // ════════════════════════════════════════════════════════════════════════════
    // UPDATE FUNCTIONS
    // ════════════════════════════════════════════════════════════════════════════

    void AirsimWrapper::update_camera_info(CameraROS* camera_ros, const float& fov)
    {
        // Calculate focal length based on FoV (rad)
        float f_x = (camera_ros->camera_info.width / 2.0f) / tan(fov / 2.0f);
        float f_y = f_x; // Assuming square pixels

        // Update camera intrinsic matrix K
        auto& k = camera_ros->camera_info.k;
        k[0] = f_x; // fx
        k[4] = f_y; // fy
    }

    // ════════════════════════════════════════════════════════════════════════════
    // PUBLISH FUNCTIONS
    // ════════════════════════════════════════════════════════════════════════════

    void AirsimWrapper::publish_vehicle_state()
    {
        for (auto& [vehicle_name, vehicle_ros] : vehicle_map_)
        {
            for (auto& [camera_name, camera_ros] : vehicle_ros->camera_map)
            {
                // Publish camera body tf
                tf_broadcaster_->sendTransform(camera_ros->body_tf_msg);
            }
        }
    }

    // ════════════════════════════════════════════════════════════════════════════
    // AIRSIM CLIENT FUNCTIONS
    // ════════════════════════════════════════════════════════════════════════════

    rclcpp::Time AirsimWrapper::client_get_timestamp()
    {
        return rclcpp::Time(airsim_client_state_->getTimestamp());
    }

    bool AirsimWrapper::client_get_paused()
    {
        return airsim_client_state_->simIsPaused();
    }

    msr::airlib::MultirotorState AirsimWrapper::client_get_multirotor_state(const std::string& vehicle_name)
    {
        return airsim_client_state_->getMultirotorState(vehicle_name);
    }

    msr::airlib::CameraInfo AirsimWrapper::client_get_camera_info(const std::string& vehicle_name, const std::string& camera_name)
    {
        return airsim_client_state_->simGetCameraInfo(camera_name, vehicle_name);
    }

    msr::airlib::Vector2r AirsimWrapper::client_get_camera_fov(const std::string& vehicle_name, const std::string& camera_name)
    {
        const std::string& fov_string = airsim_client_state_->simGetCurrentFieldOfView(camera_name, vehicle_name);

        // Extract horizontal and vertical FOV values from the string
        float horizontal_fov = 0.0f;
        float vertical_fov = 0.0f;

        // Parse the FOV string
        size_t h_pos = fov_string.find("Horizontal Field Of View: ");
        size_t v_pos = fov_string.find("Vertical Field Of View: ");

        if (h_pos != std::string::npos) {
            size_t h_start = h_pos + 26; // Length of "Horizontal Field Of View: "
            size_t h_end = fov_string.find(";", h_start);
            if (h_end != std::string::npos) {
                horizontal_fov = std::stof(fov_string.substr(h_start, h_end - h_start));
            }
        }

        if (v_pos != std::string::npos) {
            size_t v_start = v_pos + 24; // Length of "Vertical Field Of View: "
            size_t v_end = fov_string.length();
            vertical_fov = std::stof(fov_string.substr(v_start, v_end - v_start));
        }

        return msr::airlib::Vector2r(horizontal_fov, vertical_fov);
    }

    void AirsimWrapper::client_reset()
    {
        airsim_client_control_->reset();
    }

    void AirsimWrapper::client_pause(const bool& is_paused)
    {
        airsim_client_control_->simPause(is_paused);
    }

    void AirsimWrapper::client_enable_control()
    {
        for (const auto& [vehicle_name, _] : vehicle_map_)
        {
            airsim_client_control_->enableApiControl(true, vehicle_name);
            airsim_client_control_->armDisarm(true, vehicle_name);
        }
    }

    void AirsimWrapper::client_takeoff(const float& timeout, const std::string& vehicle_name, const bool& wait)
    {
        if (wait)
        {
            airsim_client_control_->takeoffAsync(timeout, vehicle_name)->waitOnLastTask();
        }
        else
        {
            airsim_client_control_->takeoffAsync(timeout, vehicle_name);
        }
    }

    void AirsimWrapper::client_land(const float& timeout, const std::string& vehicle_name, const bool& wait)
    {
        if (wait)
        {
            airsim_client_control_->landAsync(timeout, vehicle_name)->waitOnLastTask();
        }
        else
        {
            airsim_client_control_->landAsync(timeout, vehicle_name);
        }
    }
    void AirsimWrapper::client_hover(const std::string& vehicle_name)
    {
        airsim_client_control_->hoverAsync(vehicle_name);
    }

    void AirsimWrapper::client_move_by_velocity(const float& vx, const float& vy, const float& vz, const float& dt, const std::string& vehicle_name)
    {
        airsim_client_control_->moveByVelocityAsync(vx, -vy, -vz, dt,
            msr::airlib::DrivetrainType::MaxDegreeOfFreedom, msr::airlib::YawMode(), vehicle_name);
    }

    void AirsimWrapper::client_set_camera_pose(const std::string& camera_name, const msr::airlib::Pose& pose, const std::string& vehicle_name)
    {
        airsim_client_control_->simSetCameraPose(camera_name, pose, vehicle_name);
    }

    void AirsimWrapper::client_set_camera_fov(const std::string& camera_name, const float& fov, const std::string& vehicle_name)
    {
        airsim_client_control_->simSetCameraFov(camera_name, math_common::rad2deg(fov), vehicle_name);
    }

    void AirsimWrapper::client_set_window_images(const std::vector<int>& window_indices, const std::vector<std::string>& vehicle_names, const std::vector<std::string>& camera_names, const std::vector<msr::airlib::Vector2r>& corners, const std::vector<msr::airlib::Vector2r>& sizes)
    {
        airsim_client_window_->simSetWindowImages(window_indices, vehicle_names, camera_names, corners, sizes);
    }

    void AirsimWrapper::client_set_window_rectangle(const int& window_index, const std::vector<msr::airlib::Vector2r>& corners, const std::vector<msr::airlib::Vector2r>& sizes, const std::vector<float>& color, const float& thickness)
    {
        airsim_client_window_->simDrawRectangles(window_index, corners, sizes, color, thickness);
    }

    void AirsimWrapper::client_set_window_strings(const int& window_index, const std::vector<std::string>& strings, const std::vector<msr::airlib::Vector2r>& positions, const std::vector<float>& color, const float& scale)
    {
        airsim_client_window_->simDrawStrings(window_index, strings, positions, color, scale);
    }

    void AirsimWrapper::client_add_targets(const std::vector<std::string>& target_names, const std::vector<std::string>& target_types, const std::vector<msr::airlib::Vector3r>& positions, const bool& highlight, const std::vector<std::vector<float>>& highlight_color_rgba)
    {
        airsim_client_tracking_->simAddTargets(target_names, target_types, positions, highlight, highlight_color_rgba);
    }

    void AirsimWrapper::client_add_clusters(const std::vector<std::string>& cluster_names, const std::vector<msr::airlib::Vector3r>& centers, const std::vector<float>& radii, const bool& highlight, const std::vector<std::vector<float>>& highlight_color_rgba)
    {
        airsim_client_tracking_->simAddClusters(cluster_names, centers, radii, highlight, highlight_color_rgba);
    }

    void AirsimWrapper::client_remove_targets(const std::vector<std::string>& target_names)
    {
        airsim_client_tracking_->simRemoveTargets(target_names);
    }

    void AirsimWrapper::client_remove_clusters(const std::vector<std::string>& cluster_names)
    {
        airsim_client_tracking_->simRemoveClusters(cluster_names);
    }

    void AirsimWrapper::client_remove_all_targets()
    {
        airsim_client_tracking_->simRemoveAllTargets();
    }

    void AirsimWrapper::client_remove_all_clusters()
    {
        airsim_client_tracking_->simRemoveAllClusters();
    }

    void AirsimWrapper::client_update_targets(const std::vector<std::string>& target_names, const std::vector<msr::airlib::Vector3r>& positions)
    {
        airsim_client_tracking_->simUpdateTargets(target_names, positions);
    }

    void AirsimWrapper::client_update_clusters(const std::vector<std::string>& cluster_names, const std::vector<msr::airlib::Vector3r>& centers, const std::vector<float>& radii)
    {
        airsim_client_tracking_->simUpdateClusters(cluster_names, centers, radii);
    }

    // ════════════════════════════════════════════════════════════════════════════
    // UTILITY FUNCTIONS
    // ════════════════════════════════════════════════════════════════════════════

    rclcpp::Time AirsimWrapper::get_sim_clock_time()
    {
        return ros_clock_.clock;
    }

    tf2::Quaternion AirsimWrapper::get_tf2_quat(const msr::airlib::Quaternionr& airlib_quat) const
    {
        return tf2::Quaternion(airlib_quat.x(), -airlib_quat.y(), -airlib_quat.z(), airlib_quat.w());
    }

    msr::airlib::Quaternionr AirsimWrapper::get_airlib_quat(const geometry_msgs::msg::Quaternion& geometry_msgs_quat) const
    {
        return msr::airlib::Quaternionr(geometry_msgs_quat.w, geometry_msgs_quat.x, -geometry_msgs_quat.y, -geometry_msgs_quat.z);
    }

    msr::airlib::Quaternionr AirsimWrapper::get_airlib_quat(const tf2::Quaternion& tf2_quat) const
    {
        return msr::airlib::Quaternionr(tf2_quat.w(), tf2_quat.x(), -tf2_quat.y(), -tf2_quat.z());
    }

    msr::airlib::Pose AirsimWrapper::get_airlib_pose(const float& x, const float& y, const float& z, const msr::airlib::Quaternionr& airlib_quat) const
    {
        return msr::airlib::Pose(msr::airlib::Vector3r(x, -y, -z), airlib_quat);
    }

    msr::airlib::Pose AirsimWrapper::get_airlib_pose(const geometry_msgs::msg::Pose& geometry_msgs_pose) const
    {
        return get_airlib_pose(geometry_msgs_pose.position.x, -geometry_msgs_pose.position.y, -geometry_msgs_pose.position.z, get_airlib_quat(geometry_msgs_pose.orientation));
    }

    msr::airlib::Vector3r AirsimWrapper::get_airlib_point(const geometry_msgs::msg::Point& geometry_msgs_point) const
    {
        return msr::airlib::Vector3r(geometry_msgs_point.x, -geometry_msgs_point.y, -geometry_msgs_point.z);
    }

    msr::airlib::Vector2r AirsimWrapper::get_airlib_point_2d(const geometry_msgs::msg::Point& geometry_msgs_point) const
    {
        return msr::airlib::Vector2r(geometry_msgs_point.x, geometry_msgs_point.y);
    }

    std::vector<msr::airlib::Vector3r> AirsimWrapper::get_airlib_points(const std::vector<geometry_msgs::msg::Point>& geometry_msgs_points) const
    {
        std::vector<msr::airlib::Vector3r> airlib_points(geometry_msgs_points.size());
        for (size_t i = 0; i < geometry_msgs_points.size(); i++)
        {
            airlib_points[i] = get_airlib_point(geometry_msgs_points[i]);
        }
        return airlib_points;
    }

    std::vector<msr::airlib::Vector2r> AirsimWrapper::get_airlib_points_2d(const std::vector<geometry_msgs::msg::Point>& geometry_msgs_points) const
    {
        std::vector<msr::airlib::Vector2r> airlib_points(geometry_msgs_points.size());
        for (size_t i = 0; i < geometry_msgs_points.size(); i++)
        {
            airlib_points[i] = get_airlib_point_2d(geometry_msgs_points[i]);
        }
        return airlib_points;
    }

    std::vector<float> AirsimWrapper::get_airlib_color(const std_msgs::msg::ColorRGBA& std_msgs_color) const
    {
        return std::vector<float>{std_msgs_color.r, std_msgs_color.g, std_msgs_color.b, std_msgs_color.a};
    }

    std::vector<std::vector<float>> AirsimWrapper::get_airlib_colors(const std::vector<std_msgs::msg::ColorRGBA>& std_msgs_colors) const
    {
        std::vector<std::vector<float>> airlib_colors;
        for (const auto& std_msgs_color : std_msgs_colors)
        {
            airlib_colors.push_back(get_airlib_color(std_msgs_color));
        }
        return airlib_colors;
    }

    geometry_msgs::msg::Transform AirsimWrapper::get_transform_msg_from_airsim(const msr::airlib::Vector3r& position, const msr::airlib::AirSimSettings::Rotation& rotation)
    {
        geometry_msgs::msg::Transform transform;
        transform.translation.x = position.x();
        transform.translation.y = -position.y();
        transform.translation.z = -position.z();
        tf2::Quaternion quat;
        quat.setRPY(rotation.roll * (M_PI / 180.0), rotation.pitch * (M_PI / 180.0), rotation.yaw * (M_PI / 180.0));
        transform.rotation.x = quat.x();
        transform.rotation.y = -quat.y();
        transform.rotation.z = -quat.z();
        transform.rotation.w = quat.w();

        return transform;
    }

    geometry_msgs::msg::Transform AirsimWrapper::get_transform_msg_from_airsim(const msr::airlib::Vector3r& position, const msr::airlib::Quaternionr& quaternion)
    {
        geometry_msgs::msg::Transform transform;
        transform.translation.x = position.x();
        transform.translation.y = -position.y();
        transform.translation.z = -position.z();
        transform.rotation.x = quaternion.x();
        transform.rotation.y = -quaternion.y();
        transform.rotation.z = -quaternion.z();
        transform.rotation.w = quaternion.w();

        return transform;
    }

    // airsim uses nans for zeros in settings.json. we set them to zeros here for handling tfs in ROS
    void AirsimWrapper::set_nans_to_zeros_in_pose(VehicleSetting& vehicle_setting) const
    {
        if (std::isnan(vehicle_setting.position.x()))
            vehicle_setting.position.x() = 0.0;

        if (std::isnan(vehicle_setting.position.y()))
            vehicle_setting.position.y() = 0.0;

        if (std::isnan(vehicle_setting.position.z()))
            vehicle_setting.position.z() = 0.0;

        if (std::isnan(vehicle_setting.rotation.yaw))
            vehicle_setting.rotation.yaw = 0.0;

        if (std::isnan(vehicle_setting.rotation.pitch))
            vehicle_setting.rotation.pitch = 0.0;

        if (std::isnan(vehicle_setting.rotation.roll))
            vehicle_setting.rotation.roll = 0.0;
    }

    // if any nan's in camera pose, set them to match vehicle pose (which has already converted any potential nans to zeros)
    void AirsimWrapper::set_nans_to_zeros_in_pose(const VehicleSetting& vehicle_setting, CameraSetting& camera_setting) const
    {
        if (std::isnan(camera_setting.position.x()))
            camera_setting.position.x() = vehicle_setting.position.x();

        if (std::isnan(camera_setting.position.y()))
            camera_setting.position.y() = vehicle_setting.position.y();

        if (std::isnan(camera_setting.position.z()))
            camera_setting.position.z() = vehicle_setting.position.z();

        if (std::isnan(camera_setting.rotation.yaw))
            camera_setting.rotation.yaw = vehicle_setting.rotation.yaw;

        if (std::isnan(camera_setting.rotation.pitch))
            camera_setting.rotation.pitch = vehicle_setting.rotation.pitch;

        if (std::isnan(camera_setting.rotation.roll))
            camera_setting.rotation.roll = vehicle_setting.rotation.roll;
    }

    geometry_msgs::msg::Transform AirsimWrapper::get_camera_optical_tf() const
    {
        geometry_msgs::msg::Transform optical_tf = geometry_msgs::msg::Transform(); // zero translation
        auto opticalQ = msr::airlib::Quaternionr(optical_tf.rotation.w, optical_tf.rotation.x, optical_tf.rotation.y, optical_tf.rotation.z);
        opticalQ *= msr::airlib::Quaternionr(0.5, -0.5, 0.5, -0.5);
        optical_tf.rotation.w = opticalQ.w();
        optical_tf.rotation.x = opticalQ.x();
        optical_tf.rotation.y = opticalQ.y();
        optical_tf.rotation.z = opticalQ.z();
        return optical_tf;
    }

} // namespace airsim_wrapper