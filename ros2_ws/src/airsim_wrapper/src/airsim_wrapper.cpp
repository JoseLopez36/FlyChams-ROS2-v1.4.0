#include <airsim_wrapper.h>

using namespace std::placeholders;

namespace airsim_wrapper
{
    constexpr char AirsimWrapper::CAM_YML_NAME[];
    constexpr char AirsimWrapper::WIDTH_YML_NAME[];
    constexpr char AirsimWrapper::HEIGHT_YML_NAME[];
    constexpr char AirsimWrapper::K_YML_NAME[];
    constexpr char AirsimWrapper::D_YML_NAME[];
    constexpr char AirsimWrapper::R_YML_NAME[];
    constexpr char AirsimWrapper::P_YML_NAME[];
    constexpr char AirsimWrapper::DMODEL_YML_NAME[];

    const std::unordered_map<int, std::string> AirsimWrapper::image_type_int_to_string_map_ = {
        { 0, "Scene" },
        { 1, "DepthPlanar" },
        { 2, "DepthPerspective" },
        { 3, "DepthVis" },
        { 4, "DisparityNormalized" },
        { 5, "Segmentation" },
        { 6, "SurfaceNormals" },
        { 7, "Infrared" },
        { 8, "OpticalFlow" },
        { 9, "OpticalFlowVis" },
        { 10, "Annotation" }

    };

    // ════════════════════════════════════════════════════════════════════════════
    // CONSTRUCTOR/DESTRUCTOR
    // ════════════════════════════════════════════════════════════════════════════

    AirsimWrapper::AirsimWrapper(const std::shared_ptr<rclcpp::Node> nh, const std::string& host_ip, uint16_t host_port, bool enable_api_control, bool enable_world_plot)
        : airsim_settings_parser_(host_ip, host_port)
        , host_ip_(host_ip)
        , host_port_(host_port)
        , enable_api_control_(enable_api_control)
        , enable_world_plot_(enable_world_plot)
        , airsim_client_state_(nullptr)
        , airsim_client_control_(nullptr)

        , nh_(nh)
        , publish_clock_(false)
        , is_connected_(false)
        , is_running_(false)
    {
        ros_clock_.clock = rclcpp::Time(0);

        if (AirSimSettings::singleton().simmode_name != AirSimSettings::kSimModeTypeMultirotor) {
            RCLCPP_ERROR(nh_->get_logger(), "Unsupported simulation mode: %s", AirSimSettings::singleton().simmode_name.c_str());
            rclcpp::shutdown();
            return;
        }

        // Get clock speed
        clock_speed_ = AirSimSettings::singleton().clock_speed;

        // Create TF broadcaster and static TF broadcaster
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(nh_);
        static_tf_pub_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(nh_);

        // Initialize AirSim clients
        RCLCPP_INFO(nh_->get_logger(), "Initializing AirSim clients...");
        initialize_airsim();

        // Post-initialize AirSim
        try
        {
            // Initialize sim clock
            ros_clock_.clock = client_get_timestamp();

            // Pause and reset simulation
            client_pause(true);
            client_reset();
        }
        catch (rpc::rpc_error& e) {
            std::string msg = e.get_error().as<std::string>();
            RCLCPP_ERROR(nh_->get_logger(), "Exception raised by the API:\n%s", msg.c_str());
            ros_clock_.clock = rclcpp::Time(0);
        }

        // Initialize ROS data and communications
        initialize_ros();

        // Enable control of all vehicles
        try
        {
            if (enable_api_control_)
            {
                RCLCPP_INFO(nh_->get_logger(), "Enabling control of all vehicles...");
                client_enable_control();
            }
        }
        catch (rpc::rpc_error& e) {
            std::string msg = e.get_error().as<std::string>();
            RCLCPP_ERROR(nh_->get_logger(), "Exception raised by the API:\n%s", msg.c_str());
            rclcpp::shutdown();
            return;
        }

        // Publish AirSim status
        RCLCPP_WARN(nh_->get_logger(), "AirsimWrapper successfully initialized!");
        is_connected_.store(true);
        is_running_.store(false);

        // Create timers
        nh_->get_parameter("update_airsim_state_every_n_sec", update_airsim_state_every_n_sec_);
        airsim_state_update_timer_ = nh_->create_wall_timer(std::chrono::duration<double>(update_airsim_state_every_n_sec_), std::bind(&AirsimWrapper::drone_state_timer_cb, this));
        nh_->get_parameter("update_sim_clock_every_n_sec", update_sim_clock_every_n_sec_);
        sim_clock_update_timer_ = nh_->create_wall_timer(std::chrono::duration<double>(update_sim_clock_every_n_sec_), std::bind(&AirsimWrapper::clock_timer_cb, this));
        nh_->get_parameter("update_airsim_status_every_n_sec", update_airsim_status_every_n_sec_);
        airsim_status_update_timer_ = nh_->create_wall_timer(std::chrono::duration<double>(update_airsim_status_every_n_sec_), std::bind(&AirsimWrapper::status_timer_cb, this));
    }

    AirsimWrapper::~AirsimWrapper()
    {
        // Shutdown AirSim
        shutdown();
    }

    void AirsimWrapper::shutdown()
    {
        std::unique_lock lock(control_mutex_, std::defer_lock);

        // Reset simulation
        lock.lock();
        client_pause(true);
        client_reset();
        lock.unlock();
        is_running_.store(false);
        // Destroy timers
        airsim_state_update_timer_.reset();
        sim_clock_update_timer_.reset();
        airsim_status_update_timer_.reset();
    }

    // ════════════════════════════════════════════════════════════════════════════
    // INITIALIZATION
    // ════════════════════════════════════════════════════════════════════════════

    void AirsimWrapper::initialize_airsim()
    {
        try
        {
            // Create airsim client for vehicle state
            airsim_client_state_ = std::unique_ptr<msr::airlib::MultirotorRpcLibClient>(new msr::airlib::MultirotorRpcLibClient(host_ip_, host_port_));
            airsim_client_state_->confirmConnection();

            // Create airsim client for simulation control
            airsim_client_control_ = std::unique_ptr<msr::airlib::MultirotorRpcLibClient>(new msr::airlib::MultirotorRpcLibClient(host_ip_, host_port_));
            airsim_client_control_->confirmConnection();
        }
        catch (rpc::rpc_error& e) {
            std::string msg = e.get_error().as<std::string>();
            RCLCPP_ERROR(nh_->get_logger(), "Exception raised by the API, something went wrong.\n%s", msg.c_str());
            rclcpp::shutdown();
        }
    }

    void AirsimWrapper::initialize_ros()
    {
        // ROS params
        nh_->get_parameter("is_vulkan", is_vulkan_);
        nh_->get_parameter("publish_clock", publish_clock_);
        nh_->get_parameter_or("world_frame_id", world_frame_id_, world_frame_id_);
        nh_->get_parameter_or("odom_frame_id", odom_frame_id_, odom_frame_id_);

        // Create ROS communications
        create_ros_comms_from_settings_json();
    }

    void AirsimWrapper::create_ros_comms_from_settings_json()
    {
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
            vehicle_ros->vehicle_frame_id = vehicle_name;
            vehicle_ros->odom_frame_id = vehicle_name + "/" + odom_frame_id_;
            initialize_vehicle_odom(vehicle_ros.get(), *vehicle_setting);
            initialize_vehicle_tf(vehicle_ros.get(), *vehicle_setting);

            // Initialize vehicle odometry publisher
            vehicle_ros->odom_pub = nh_->create_publisher<nav_msgs::msg::Odometry>(vehicle_name + "/" + odom_frame_id_, 10);
            vehicle_ros->camera_info_array_pub = nh_->create_publisher<airsim_interfaces::msg::CameraInfoArray>(vehicle_name + "/camera_info_array", 10);

            // Initialize vehicle subscribers
            vehicle_ros->vel_cmd_sub = nh_->create_subscription<airsim_interfaces::msg::VelCmd>(
                vehicle_name + "/vel_cmd", 10,
                [this, vehicle_name](const airsim_interfaces::msg::VelCmd::SharedPtr msg) {
                    this->vel_cmd_cb(vehicle_name, msg);
                }
            );
            vehicle_ros->gimbal_angle_cmd_sub = nh_->create_subscription<airsim_interfaces::msg::GimbalAngleCmd>(
                vehicle_name + "/gimbal_angle_cmd", 10,
                [this, vehicle_name](const airsim_interfaces::msg::GimbalAngleCmd::SharedPtr msg) {
                    this->gimbal_angle_cmd_cb(vehicle_name, msg);
                }
            );
            vehicle_ros->camera_fov_cmd_sub = nh_->create_subscription<airsim_interfaces::msg::CameraFovCmd>(
                vehicle_name + "/camera_fov_cmd", 10,
                [this, vehicle_name](const airsim_interfaces::msg::CameraFovCmd::SharedPtr msg) {
                    this->camera_fov_cmd_cb(vehicle_name, msg);
                }
            );

            // Iterate over camera map
            vehicle_ros->camera_info_array = airsim_interfaces::msg::CameraInfoArray();
            vehicle_ros->camera_info_array.infos.clear();
            for (auto& [camera_name, camera_setting] : vehicle_setting->cameras)
            {
                RCLCPP_INFO(nh_->get_logger(), "Initializing camera %s from settings...", camera_name.c_str());
                set_nans_to_zeros_in_pose(*vehicle_setting, camera_setting);

                // Initialize camera ROS instance
                auto camera_ros = std::unique_ptr<CameraROS>(new CameraROS());
                camera_ros->camera_name = camera_name;
                camera_ros->camera_setting = camera_setting;
                camera_ros->body_frame_id = vehicle_name + "/" + camera_name + "/body";
                camera_ros->optical_frame_id = vehicle_name + "/" + camera_name + "/optical";
                initialize_camera_info(camera_ros.get(), camera_setting, camera_setting.capture_settings.at(0));
                initialize_camera_tf(vehicle_ros.get(), camera_ros.get(), camera_setting);

                // Add camera to vehicle's camera map
                vehicle_ros->camera_info_array.infos.push_back(camera_ros->camera_info);
                vehicle_ros->cameras.push_back(std::move(camera_ros));
            }

            // Add vehicle to map
            vehicle_map_.emplace(vehicle_name, std::move(vehicle_ros));
        }

        // Create ROS communications
        RCLCPP_INFO(nh_->get_logger(), "Creating ROS communications...");
        // Create airsim status publisher
        airsim_status_pub_ = nh_->create_publisher<airsim_interfaces::msg::Status>("status", 1);
        // Create clock publisher
        if (publish_clock_)
        {
            clock_pub_ = nh_->create_publisher<rosgraph_msgs::msg::Clock>("/clock", 1);
        }
        // Create global control services
        reset_srvr_ = nh_->create_service<airsim_interfaces::srv::Reset>("reset", std::bind(&AirsimWrapper::reset_srv_cb, this, _1, _2));
        run_srvr_ = nh_->create_service<airsim_interfaces::srv::Run>("run", std::bind(&AirsimWrapper::run_srv_cb, this, _1, _2));
        pause_srvr_ = nh_->create_service<airsim_interfaces::srv::Pause>("pause", std::bind(&AirsimWrapper::pause_srv_cb, this, _1, _2));
        // Create group of robots services
        takeoff_group_srvr_ = nh_->create_service<airsim_interfaces::srv::TakeoffGroup>("group_of_robots/takeoff", std::bind(&AirsimWrapper::takeoff_group_srv_cb, this, _1, _2));
        land_group_srvr_ = nh_->create_service<airsim_interfaces::srv::LandGroup>("group_of_robots/land", std::bind(&AirsimWrapper::land_group_srv_cb, this, _1, _2));
        hover_group_srvr_ = nh_->create_service<airsim_interfaces::srv::HoverGroup>("group_of_robots/hover", std::bind(&AirsimWrapper::hover_group_srv_cb, this, _1, _2));
        // Create group of objects services
        spawn_object_group_srvr_ = nh_->create_service<airsim_interfaces::srv::SpawnObjectGroup>("group_of_objects/spawn", std::bind(&AirsimWrapper::spawn_object_group_cb, this, _1, _2));
        despawn_object_group_srvr_ = nh_->create_service<airsim_interfaces::srv::DespawnObjectGroup>("group_of_objects/despawn", std::bind(&AirsimWrapper::despawn_object_group_cb, this, _1, _2));
        // Create group of objects subscribers
        object_pose_cmd_group_sub_ = nh_->create_subscription<airsim_interfaces::msg::ObjectPoseCmdGroup>("group_of_objects/pose_cmd", 10, std::bind(&AirsimWrapper::object_pose_cmd_group_cb, this, _1));
        // Create group of windows subscribers
        window_image_cmd_group_sub_ = nh_->create_subscription<airsim_interfaces::msg::WindowImageCmdGroup>("group_of_windows/image_cmd", 10, std::bind(&AirsimWrapper::window_image_cmd_group_cb, this, _1));
    }

    void AirsimWrapper::initialize_vehicle_odom(VehicleROS* vehicle_ros, const VehicleSetting& vehicle_setting)
    {
        vehicle_ros->curr_odom.header.frame_id = vehicle_ros->vehicle_frame_id;
        vehicle_ros->curr_odom.child_frame_id = vehicle_ros->odom_frame_id;
        vehicle_ros->curr_odom.header.stamp = get_sim_clock_time();
        vehicle_ros->curr_odom.pose.pose = geometry_msgs::msg::Pose();
        vehicle_ros->curr_odom.twist.twist = geometry_msgs::msg::Twist();
    }

    void AirsimWrapper::initialize_vehicle_tf(VehicleROS* vehicle_ros, const VehicleSetting& vehicle_setting)
    {
        auto& vehicle_tf = vehicle_ros->vehicle_static_tf_msg;
        auto& odom_tf = vehicle_ros->odom_tf_msg;

        // World to Vehicle transform
        vehicle_tf.header.frame_id = world_frame_id_;
        vehicle_tf.child_frame_id = vehicle_ros->vehicle_frame_id;
        vehicle_tf.header.stamp = get_sim_clock_time();
        vehicle_tf.transform = get_transform_msg_from_airsim(vehicle_setting.position, vehicle_setting.rotation);
        convert_tf_msg_to_ros(vehicle_tf);
        static_tf_pub_->sendTransform(vehicle_tf);

        // Vehicle to Odom transform
        odom_tf.header.frame_id = vehicle_ros->vehicle_frame_id;
        odom_tf.child_frame_id = vehicle_ros->odom_frame_id;
        odom_tf.header.stamp = get_sim_clock_time();
        odom_tf.transform = geometry_msgs::msg::Transform();
        convert_tf_msg_to_ros(odom_tf);
        tf_broadcaster_->sendTransform(odom_tf);
    }

    void AirsimWrapper::initialize_camera_info(CameraROS* camera_ros, const CameraSetting& camera_setting, const CaptureSetting& capture_setting)
    {
        // Set camera info header
        camera_ros->camera_info.header.frame_id = camera_ros->optical_frame_id;
        camera_ros->camera_info.header.stamp = get_sim_clock_time();

        // Get width and height (assuming constant)
        const auto& width = capture_setting.width;
        const auto& height = capture_setting.height;

        // Calculate focal length based on FoV
        float f_x = (width / 2.0f) / tan(math_common::deg2rad(capture_setting.fov_degrees / 2.0f));
        float f_y = f_x; // Assuming square pixels

        // Principal point at image center
        float c_x = width / 2.0f;
        float c_y = height / 2.0f;

        // Set camera intrinsic matrix K (3x3)
        // Assuming skew is 0
        camera_ros->camera_info.k = {
            f_x, 0.0, c_x,
            0.0, f_y, c_y,
            0.0, 0.0, 1.0
        };

        // Set projection matrix P (3x4)
        // For an ideal camera, P = K * [R|t] where R=I, t=0
        camera_ros->camera_info.p = {
            f_x, 0.0, c_x, 0.0,
            0.0, f_y, c_y, 0.0,
            0.0, 0.0, 1.0, 0.0
        };
    }

    void AirsimWrapper::initialize_camera_tf(VehicleROS* vehicle_ros, CameraROS* camera_ros, const CameraSetting& camera_setting)
    {
        auto& body_tf = camera_ros->body_tf_msg;
        auto& optical_tf = camera_ros->optical_static_tf_msg;

        // World to Body transform
        if (camera_setting.external)
        {
            body_tf.header.frame_id = world_frame_id_;
        }
        else
        {
            body_tf.header.frame_id = vehicle_ros->vehicle_frame_id;
        }
        body_tf.child_frame_id = camera_ros->body_frame_id;
        body_tf.header.stamp = get_sim_clock_time();
        auto camera_info_data = airsim_client_control_->simGetCameraInfo(camera_ros->camera_name, vehicle_ros->vehicle_name);
        body_tf.transform = get_transform_msg_from_airsim(camera_info_data.pose.position, camera_info_data.pose.orientation);
        convert_tf_msg_to_ros(body_tf);
        tf_broadcaster_->sendTransform(body_tf);

        // Body to Optical transform
        optical_tf.header.frame_id = camera_ros->body_frame_id;
        optical_tf.child_frame_id = camera_ros->optical_frame_id;
        optical_tf.header.stamp = get_sim_clock_time();
        optical_tf.transform = get_camera_optical_tf();
        static_tf_pub_->sendTransform(optical_tf);
    }

    // ════════════════════════════════════════════════════════════════════════════
    // COMMAND CALLBACKS: Thread-safe
    // ════════════════════════════════════════════════════════════════════════════

    void AirsimWrapper::vel_cmd_cb(const std::string& vehicle_name, const airsim_interfaces::msg::VelCmd::SharedPtr vel_cmd_msg)
    {
        RCLCPP_INFO_THROTTLE(nh_->get_logger(), *nh_->get_clock(), 1000.0, "Received velocity command message");

        std::unique_lock lock(control_mutex_, std::defer_lock);

        // Extract message data
        const auto& vx = vel_cmd_msg->vel_cmd_x;
        const auto& vy = vel_cmd_msg->vel_cmd_y;
        const auto& vz = vel_cmd_msg->vel_cmd_z;
        const auto& dt = vel_cmd_msg->vel_cmd_dt;
        try
        {
            // Send command to server
            lock.lock();
            client_move_by_velocity(vx, vy, vz, dt, vehicle_name);
            lock.unlock();
        }
        catch (rpc::rpc_error& e) {
            std::string msg = e.get_error().as<std::string>();
            RCLCPP_ERROR(nh_->get_logger(), "Exception raised by the API:\n%s", msg.c_str());
        }
    }

    void AirsimWrapper::gimbal_angle_cmd_cb(const std::string& vehicle_name, const airsim_interfaces::msg::GimbalAngleCmd::SharedPtr gimbal_angle_cmd_msg)
    {
        RCLCPP_INFO_THROTTLE(nh_->get_logger(), *nh_->get_clock(), 1000.0, "Received gimbal angle commands message");

        std::unique_lock lock(control_mutex_, std::defer_lock);

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
                lock.lock();
                client_set_camera_pose(camera_name, pose, vehicle_name);
                lock.unlock();
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
        RCLCPP_INFO_THROTTLE(nh_->get_logger(), *nh_->get_clock(), 1000.0, "Received camera fov commands message");

        std::unique_lock lock(control_mutex_, std::defer_lock);

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
                lock.lock();
                client_set_camera_fov(camera_name, fov_cmd, vehicle_name);
                lock.unlock();
            }
        }
        catch (rpc::rpc_error& e) {
            std::string msg = e.get_error().as<std::string>();
            RCLCPP_ERROR(nh_->get_logger(), "Exception raised by the API:\n%s", msg.c_str());
            return; // Stop execution of this callback
        }
    }

    void AirsimWrapper::object_pose_cmd_group_cb(const airsim_interfaces::msg::ObjectPoseCmdGroup::SharedPtr object_pose_cmd_group_msg)
    {
        RCLCPP_INFO_THROTTLE(nh_->get_logger(), *nh_->get_clock(), 1000.0, "Received object pose command group message");

        std::unique_lock lock(control_mutex_, std::defer_lock);

        // Extract message data
        const auto& object_names = object_pose_cmd_group_msg->object_names;
        const auto& poses = object_pose_cmd_group_msg->poses;
        try
        {
            // Set object poses
            std::vector<msr::airlib::Vector3r> points;
            for (size_t i = 0; i < object_names.size(); i++)
            {
                const auto& object_name = object_names[i];
                const auto& pose = poses[i];

                // Convert pose to airlib
                msr::airlib::Pose pose_airlib = get_airlib_pose(pose);
                points.push_back(pose_airlib.position);

                // Send pose command to server
                lock.lock();
                client_set_object_pose(object_name, pose_airlib);
                lock.unlock();
            }

            // Plot object points
            if (enable_world_plot_)
            {
                // Send plot command to server
                lock.lock();
                client_plot_points(points, { 1.0, 0.0, 0.0, 1.0 }, 7.5f, (0.25f / clock_speed_) * 1.0f, false);
                lock.unlock();
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
        RCLCPP_INFO_THROTTLE(nh_->get_logger(), *nh_->get_clock(), 1000.0, "Received window image command group message");

        std::unique_lock lock(control_mutex_, std::defer_lock);

        // Extract request data
        const auto& window_indices = window_image_cmd_group_msg->window_indices;
        const auto& vehicle_names = window_image_cmd_group_msg->vehicle_names;
        const auto& camera_names = window_image_cmd_group_msg->camera_names;
        const auto& crop_x = window_image_cmd_group_msg->crop_x;
        const auto& crop_y = window_image_cmd_group_msg->crop_y;
        const auto& crop_w = window_image_cmd_group_msg->crop_w;
        const auto& crop_h = window_image_cmd_group_msg->crop_h;
        try
        {
            for (size_t i = 0; i < window_indices.size(); i++)
            {
                const auto& window_index = window_indices[i];
                if (window_index == -1)
                    continue;
                const auto& vehicle_name = vehicle_names[i];
                const auto& camera_name = camera_names[i];
                const auto& x = crop_x[i];
                const auto& y = crop_y[i];
                const auto& w = crop_w[i];
                const auto& h = crop_h[i];

                // Send command to server
                lock.lock();
                client_set_window_image(window_index, vehicle_name, camera_name, x, y, w, h);
                lock.unlock();
            }
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

        std::unique_lock lock(control_mutex_, std::defer_lock);

        // Pause and reset AirSim
        try
        {
            // Send command to server
            lock.lock();
            client_pause(true);
            client_reset();
            // We need to re-arm the vehicles after resetting
            if (enable_api_control_)
            {
                client_enable_control();
            }
            lock.unlock();

            // Update running state
            is_running_.store(false);
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

        std::unique_lock lock(control_mutex_, std::defer_lock);

        try
        {
            // Send command to server
            lock.lock();
            client_pause(false);
            lock.unlock();

            // Update running state
            is_running_.store(true);
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

        std::unique_lock lock(control_mutex_, std::defer_lock);

        try
        {
            // Send command to server
            lock.lock();
            client_pause(true);
            lock.unlock();

            // Update running state
            is_running_.store(false);
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

    bool AirsimWrapper::takeoff_group_srv_cb(std::shared_ptr<airsim_interfaces::srv::TakeoffGroup::Request> request, std::shared_ptr<airsim_interfaces::srv::TakeoffGroup::Response> response)
    {
        RCLCPP_INFO(nh_->get_logger(), "Taking off group of vehicles");

        std::unique_lock lock(control_mutex_, std::defer_lock);

        try
        {
            // Send command to server
            lock.lock();
            for (const auto& vehicle_name : request->vehicle_names)
            {
                client_takeoff(20, vehicle_name);
            }
            lock.unlock();
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

    bool AirsimWrapper::land_group_srv_cb(std::shared_ptr<airsim_interfaces::srv::LandGroup::Request> request, std::shared_ptr<airsim_interfaces::srv::LandGroup::Response> response)
    {
        RCLCPP_INFO(nh_->get_logger(), "Landing group of vehicles");

        std::unique_lock lock(control_mutex_, std::defer_lock);

        try
        {
            // Send command to server
            lock.lock();
            for (const auto& vehicle_name : request->vehicle_names)
            {
                client_land(60, vehicle_name);
            }
            lock.unlock();
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

    bool AirsimWrapper::hover_group_srv_cb(std::shared_ptr<airsim_interfaces::srv::HoverGroup::Request> request, std::shared_ptr<airsim_interfaces::srv::HoverGroup::Response> response)
    {
        RCLCPP_INFO(nh_->get_logger(), "Hovering group of vehicles");

        std::unique_lock lock(control_mutex_, std::defer_lock);

        try
        {
            // Send command to server
            lock.lock();
            for (const auto& vehicle_name : request->vehicle_names)
            {
                client_hover(vehicle_name);
            }
            lock.unlock();
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

    bool AirsimWrapper::spawn_object_group_cb(const std::shared_ptr<airsim_interfaces::srv::SpawnObjectGroup::Request> request, const std::shared_ptr<airsim_interfaces::srv::SpawnObjectGroup::Response> response)
    {
        RCLCPP_INFO(nh_->get_logger(), "Spawning group of objects");

        std::unique_lock lock(control_mutex_, std::defer_lock);

        // Extract request data
        const auto& object_names = request->object_names;
        const auto& blueprints = request->blueprints;
        const auto& poses = request->poses;
        const auto& scales = request->scales;
        try
        {
            for (size_t i = 0; i < object_names.size(); i++)
            {
                const auto& object_name = object_names[i];
                const auto& blueprint = blueprints[i];
                const auto& pose = poses[i];
                const auto& scale = scales[i];

                // Convert pose to airlib Pose
                msr::airlib::Pose pose_airlib = get_airlib_pose(pose);

                // Convert scale to airlib Vector3r
                msr::airlib::Vector3r scale_airlib = msr::airlib::Vector3r{ scale, scale, scale };

                // Send command to server
                lock.lock();
                //client_spawn_object(object_name, blueprint, pose_airlib, scale_airlib, true);
                lock.unlock();
            }
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

    bool AirsimWrapper::despawn_object_group_cb(const std::shared_ptr<airsim_interfaces::srv::DespawnObjectGroup::Request> request, const std::shared_ptr<airsim_interfaces::srv::DespawnObjectGroup::Response> response)
    {
        RCLCPP_INFO(nh_->get_logger(), "Despawning group of objects");

        std::unique_lock lock(control_mutex_, std::defer_lock);

        // Extract request data
        const auto& object_names = request->object_names;
        try
        {
            for (size_t i = 0; i < object_names.size(); i++)
            {
                const auto& object_name = object_names[i];

                // Send command to server
                lock.lock();
                //client_destroy_object(object_name);
                lock.unlock();
            }
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
    // TIMER CALLBACKS: Thread-safe
    // ════════════════════════════════════════════════════════════════════════════

    void AirsimWrapper::drone_state_timer_cb()
    {
        std::unique_lock lock(state_mutex_, std::defer_lock);

        try
        {
            // Retrieve simulation state
            lock.lock();
            const bool is_paused = client_get_paused();
            rclcpp::Time curr_time = get_sim_clock_time();
            lock.unlock();

            // Update simulation state if not paused
            if (!is_paused)
            {
                // Iterate over drones
                for (auto& [vehicle_name, vehicle_ros] : vehicle_map_)
                {
                    // Retrieve multirotor state from server
                    lock.lock();
                    const msr::airlib::MultirotorState multirotor_state = client_get_multirotor_state(vehicle_name);
                    lock.unlock();

                    // Update vehicle odom and tf
                    vehicle_ros->curr_odom.header.stamp = curr_time;
                    vehicle_ros->odom_tf_msg.header.stamp = curr_time;
                    update_vehicle_odom(vehicle_ros.get(), multirotor_state.kinematics_estimated);
                    vehicle_ros->odom_tf_msg.transform = get_transform_msg_from_airsim(multirotor_state.getPosition(), multirotor_state.getOrientation());
                    convert_tf_msg_to_ros(vehicle_ros->odom_tf_msg);

                    // Iterate over cameras
                    for (size_t i = 0; i < vehicle_ros->cameras.size(); i++)
                    {
                        auto& camera_ros = vehicle_ros->cameras[i];

                        // Retrieve camera info from server
                        lock.lock();
                        const msr::airlib::CameraInfo camera_info_data = client_get_camera_info(vehicle_name, camera_ros->camera_name);
                        lock.unlock();

                        // Update camera info and tf
                        camera_ros->camera_info.header.stamp = curr_time;
                        camera_ros->body_tf_msg.header.stamp = curr_time;
                        update_camera_info(camera_ros.get(), camera_info_data.fov);
                        camera_ros->body_tf_msg.transform = get_transform_msg_from_airsim(camera_info_data.pose.position, camera_info_data.pose.orientation);
                        convert_tf_msg_to_ros(camera_ros->body_tf_msg);
                        vehicle_ros->camera_info_array.infos[i] = camera_ros->camera_info;
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
        std::unique_lock lock(state_mutex_, std::defer_lock);

        try
        {
            // Retrieve timestamp from server
            lock.lock();
            ros_clock_.clock = client_get_timestamp();
            lock.unlock();

            // Publish clock
            if (publish_clock_)
            {
                clock_pub_->publish(ros_clock_);
            }
        }
        catch (rpc::rpc_error& e) {
            std::string msg = e.get_error().as<std::string>();
            RCLCPP_ERROR(nh_->get_logger(), "Exception raised by the API:\n%s", msg.c_str());
        }
    }

    void AirsimWrapper::status_timer_cb()
    {
        // Publish AirSim status
        airsim_interfaces::msg::Status airsim_status_msg;
        airsim_status_msg.is_connected = is_connected_.load();
        airsim_status_msg.is_running = is_running_.load();
        airsim_status_pub_->publish(airsim_status_msg);
    }

    // ════════════════════════════════════════════════════════════════════════════
    // UPDATE FUNCTIONS
    // ════════════════════════════════════════════════════════════════════════════

    void AirsimWrapper::update_vehicle_odom(VehicleROS* vehicle_ros, const msr::airlib::Kinematics::State& kinematics_estimated)
    {
        // Get estimated pose and twist
        auto& est_pose = kinematics_estimated.pose;
        auto& est_twist = kinematics_estimated.twist;

        // Get odom pose and twist
        auto& pose = vehicle_ros->curr_odom.pose.pose;
        auto& twist = vehicle_ros->curr_odom.twist.twist;

        // Update odom pose
        pose.position.x = est_pose.position.x();
        pose.position.y = -est_pose.position.y();
        pose.position.z = -est_pose.position.z();
        pose.orientation.x = est_pose.orientation.x();
        pose.orientation.y = -est_pose.orientation.y();
        pose.orientation.z = -est_pose.orientation.z();
        pose.orientation.w = est_pose.orientation.w();

        // Update odom twist
        twist.linear.x = est_twist.linear.x();
        twist.linear.y = -est_twist.linear.y();
        twist.linear.z = -est_twist.linear.z();
        twist.angular.x = est_twist.angular.x();
        twist.angular.y = -est_twist.angular.y();
        twist.angular.z = -est_twist.angular.z();
    }

    void AirsimWrapper::update_camera_info(CameraROS* camera_ros, float new_fov)
    {
        // Get width and height (assuming constant)
        const auto& width = camera_ros->camera_info.width;
        const auto& height = camera_ros->camera_info.height;

        // Calculate focal length based on FoV
        float f_x = (width / 2.0f) / tan(math_common::deg2rad(new_fov / 2.0f));
        float f_y = f_x; // Assuming square pixels

        // Principal point at image center
        float c_x = width / 2.0f;
        float c_y = height / 2.0f;

        // Set camera intrinsic matrix K (3x3)
        // Assuming skew is 0
        camera_ros->camera_info.k = {
            f_x, 0.0, c_x,
            0.0, f_y, c_y,
            0.0, 0.0, 1.0
        };

        // Set projection matrix P (3x4)
        // For an ideal camera, P = K * [R|t] where R=I, t=0
        camera_ros->camera_info.p = {
            f_x, 0.0, c_x, 0.0,
            0.0, f_y, c_y, 0.0,
            0.0, 0.0, 1.0, 0.0
        };
    }

    // ════════════════════════════════════════════════════════════════════════════
    // PUBLISH FUNCTIONS
    // ════════════════════════════════════════════════════════════════════════════

    void AirsimWrapper::publish_vehicle_state()
    {
        for (auto& [vehicle_name, vehicle_ros] : vehicle_map_)
        {
            // Odom and transforms
            vehicle_ros->odom_pub->publish(vehicle_ros->curr_odom);

            // Publish odom tf
            tf_broadcaster_->sendTransform(vehicle_ros->odom_tf_msg);

            // Camera info
            vehicle_ros->camera_info_array_pub->publish(vehicle_ros->camera_info_array);

            // Publish camera body tfs
            for (size_t i = 0; i < vehicle_ros->cameras.size(); i++)
            {
                auto& camera_ros = vehicle_ros->cameras[i];
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

    void AirsimWrapper::client_takeoff(const float& timeout, const std::string& vehicle_name)
    {
        airsim_client_control_->takeoffAsync(timeout, vehicle_name);
    }

    void AirsimWrapper::client_land(const float& timeout, const std::string& vehicle_name)
    {
        airsim_client_control_->landAsync(timeout, vehicle_name);
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
        airsim_client_control_->simSetCameraFov(camera_name, fov, vehicle_name);
    }

    void AirsimWrapper::client_set_object_pose(const std::string& object_name, const msr::airlib::Pose& pose)
    {
        airsim_client_control_->simSetObjectPose(object_name, pose);
    }

    void AirsimWrapper::client_plot_points(const std::vector<msr::airlib::Vector3r>& points, const std::vector<float>& color, const float& size, const float& duration, const bool& is_persistent)
    {
        airsim_client_control_->simPlotPoints(points, color, size, duration, is_persistent);
    }

    void AirsimWrapper::client_set_window_image(const int& window_index, const std::string& vehicle_name, const std::string& camera_name, const int& x, const int& y, const int& w, const int& h)
    {
        airsim_client_control_->simSetWindowImage(window_index, vehicle_name, camera_name, msr::airlib::Vector2r(x, y), msr::airlib::Vector2r(w, h));
    }

    void AirsimWrapper::client_spawn_object(const std::string& object_name, const std::string& blueprint, const msr::airlib::Pose& pose, const msr::airlib::Vector3r& scale, const bool& physics_enabled)
    {
        airsim_client_control_->simSpawnObject(object_name, blueprint, pose, scale, physics_enabled);
    }

    void AirsimWrapper::client_destroy_object(const std::string& object_name)
    {
        airsim_client_control_->simDestroyObject(object_name);
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

    geometry_msgs::msg::Transform AirsimWrapper::get_transform_msg_from_airsim(const msr::airlib::Vector3r& position, const msr::airlib::AirSimSettings::Rotation& rotation)
    {
        geometry_msgs::msg::Transform transform;
        transform.translation.x = position.x();
        transform.translation.y = position.y();
        transform.translation.z = position.z();
        tf2::Quaternion quat;
        quat.setRPY(rotation.roll * (M_PI / 180.0), rotation.pitch * (M_PI / 180.0), rotation.yaw * (M_PI / 180.0));
        transform.rotation.x = quat.x();
        transform.rotation.y = quat.y();
        transform.rotation.z = quat.z();
        transform.rotation.w = quat.w();

        return transform;
    }

    geometry_msgs::msg::Transform AirsimWrapper::get_transform_msg_from_airsim(const msr::airlib::Vector3r& position, const msr::airlib::Quaternionr& quaternion)
    {
        geometry_msgs::msg::Transform transform;
        transform.translation.x = position.x();
        transform.translation.y = position.y();
        transform.translation.z = position.z();
        transform.rotation.x = quaternion.x();
        transform.rotation.y = quaternion.y();
        transform.rotation.z = quaternion.z();
        transform.rotation.w = quaternion.w();

        return transform;
    }

    geometry_msgs::msg::Pose AirsimWrapper::get_pose_msg_from_airsim(const msr::airlib::Vector3r& position, const msr::airlib::AirSimSettings::Rotation& rotation)
    {
        geometry_msgs::msg::Pose pose;
        pose.position.x = position.x();
        pose.position.y = position.y();
        pose.position.z = position.z();
        tf2::Quaternion quat;
        quat.setRPY(rotation.roll * (M_PI / 180.0), rotation.pitch * (M_PI / 180.0), rotation.yaw * (M_PI / 180.0));
        pose.orientation.x = quat.x();
        pose.orientation.y = quat.y();
        pose.orientation.z = quat.z();
        pose.orientation.w = quat.w();

        return pose;
    }

    geometry_msgs::msg::Pose AirsimWrapper::get_pose_msg_from_airsim(const msr::airlib::Vector3r& position, const msr::airlib::Quaternionr& quaternion)
    {
        geometry_msgs::msg::Pose pose;
        pose.position.x = position.x();
        pose.position.y = position.y();
        pose.position.z = position.z();
        pose.orientation.x = quaternion.x();
        pose.orientation.y = quaternion.y();
        pose.orientation.z = quaternion.z();
        pose.orientation.w = quaternion.w();

        return pose;
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

    void AirsimWrapper::convert_tf_msg_to_enu(geometry_msgs::msg::TransformStamped& tf_msg)
    {
        std::swap(tf_msg.transform.translation.x, tf_msg.transform.translation.y);
        std::swap(tf_msg.transform.rotation.x, tf_msg.transform.rotation.y);
        tf_msg.transform.translation.z = -tf_msg.transform.translation.z;
        tf_msg.transform.rotation.z = -tf_msg.transform.rotation.z;
    }

    void AirsimWrapper::convert_tf_msg_to_ros(geometry_msgs::msg::TransformStamped& tf_msg)
    {
        tf_msg.transform.translation.z = -tf_msg.transform.translation.z;
        tf_msg.transform.translation.y = -tf_msg.transform.translation.y;
        tf_msg.transform.rotation.z = -tf_msg.transform.rotation.z;
        tf_msg.transform.rotation.y = -tf_msg.transform.rotation.y;
    }

    void AirsimWrapper::convert_pose_msg_to_enu(geometry_msgs::msg::Pose& pose_msg)
    {
        std::swap(pose_msg.position.x, pose_msg.position.y);
        std::swap(pose_msg.orientation.x, pose_msg.orientation.y);
        pose_msg.position.z = -pose_msg.position.z;
        pose_msg.orientation.z = -pose_msg.orientation.z;
    }

    void AirsimWrapper::convert_pose_msg_to_ros(geometry_msgs::msg::Pose& pose_msg)
    {
        pose_msg.position.z = -pose_msg.position.z;
        pose_msg.position.y = -pose_msg.position.y;
        pose_msg.orientation.z = -pose_msg.orientation.z;
        pose_msg.orientation.y = -pose_msg.orientation.y;
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