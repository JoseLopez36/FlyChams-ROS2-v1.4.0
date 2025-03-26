#include "flychams_dashboard/gui/gui_controller.hpp"

using namespace std::chrono_literals;
using namespace flychams::core;

namespace flychams::dashboard
{
    // ════════════════════════════════════════════════════════════════════════════
    // CONSTRUCTOR: Constructor and destructor
    // ════════════════════════════════════════════════════════════════════════════

    void GuiController::onInit()
    {
        // Get parameters from parameter server
        // Get windows IDs
        fixed_window_ids_ = RosUtils::getParameter<IDs>(node_, "fixed_window_ids");
        central_window_id_ = RosUtils::getParameter<std::string>(node_, "central_window_id");
        tracking_window_ids_ = RosUtils::getParameter<IDs>(node_, "tracking_window_ids");
        // Get fixed camera IDs
        fixed_camera_ids_ = RosUtils::getParameter<IDs>(node_, "fixed_camera_ids");

        // Get window count
        num_fixed_windows_ = static_cast<int>(fixed_window_ids_.size());
        num_tracking_windows_ = static_cast<int>(tracking_window_ids_.size());
        num_windows_ = num_fixed_windows_ + 1 + num_tracking_windows_;

        // Replace fixed camera placeholders with agent ID
        for (auto& camera_id : fixed_camera_ids_)
            camera_id = RosUtils::replace(camera_id, "AGENTID", agent_id_);

        // Get central camera ID
        central_camera_id_ = config_tools_->getAgent(agent_id_)->central_head_id;

        // Initialize tracking command vectors
        tracking_vehicle_id_cmds_.resize(num_tracking_windows_);
        tracking_camera_id_cmds_.resize(num_tracking_windows_);
        tracking_crop_x_cmds_.resize(num_tracking_windows_);
        tracking_crop_y_cmds_.resize(num_tracking_windows_);
        tracking_crop_w_cmds_.resize(num_tracking_windows_);
        tracking_crop_h_cmds_.resize(num_tracking_windows_);
        for (int i = 0; i < num_tracking_windows_; i++)
        {
            tracking_vehicle_id_cmds_[i] = agent_id_;
            tracking_camera_id_cmds_[i] = "";
            tracking_crop_x_cmds_[i] = 0;
            tracking_crop_y_cmds_[i] = 0;
            tracking_crop_w_cmds_[i] = 0;
            tracking_crop_h_cmds_[i] = 0;
        }

        // Initialize digital tracking drawing data
        // Rectangle data
        dig_rect_color_.r = 0.0f;
        dig_rect_color_.g = 1.0f;
        dig_rect_color_.b = 1.0f;
        dig_rect_color_.a = 0.5f;
        dig_rect_thickness_ = 2.0f;
        for (size_t i = 0; i < tracking_window_ids_.size(); i++)
        {
            PointMsg point;
            point.x = HUGE_VALF;
            point.y = HUGE_VALF;
            dig_rect_corners_.push_back(point);
            dig_rect_sizes_.push_back(point);
        }
        // String data
        dig_string_color_.r = 0.0f;
        dig_string_color_.g = 1.0f;
        dig_string_color_.b = 1.0f;
        dig_string_color_.a = 0.5f;
        dig_string_scale_ = 128.0f;
        for (size_t i = 0; i < tracking_window_ids_.size(); i++)
        {
            PointMsg point;
            point.x = HUGE_VALF;
            point.y = HUGE_VALF;
            dig_strings_.push_back("TW" + std::to_string(i));
            dig_string_pos_.push_back(point);
        }

        // Initialize GUI data
        goal_ = TrackingGoalMsg();
        has_goal_ = false;

        // Subscribe to tracking goal
        tracking_sub_ = topic_tools_->createAgentTrackingGoalSubscriber(agent_id_,
            std::bind(&GuiController::trackingCallback, this, std::placeholders::_1));
    }

    void GuiController::onShutdown()
    {
        // Destroy subscriber
        tracking_sub_.reset();
    }

    // ════════════════════════════════════════════════════════════════════════════
    // SAFE CALLBACKS: Thread-safe callback functions
    // ════════════════════════════════════════════════════════════════════════════

    void GuiController::trackingCallback(const core::TrackingGoalMsg::SharedPtr msg)
    {
        // Update tracking goal under lock
        std::lock_guard<std::mutex> lock(mutex_);
        goal_ = *msg;
        has_goal_ = true;
    }

    // ════════════════════════════════════════════════════════════════════════════
    // UPDATE: Update GUI and listen for user input
    // ════════════════════════════════════════════════════════════════════════════

    void GuiController::setFixedWindows()
    {
        // Set each fixed window parameters
        IDs fixed_vehicle_id_cmds(num_fixed_windows_);
        IDs fixed_camera_id_cmds(num_fixed_windows_);
        std::vector<int> fixed_crop_x_cmds(num_fixed_windows_);
        std::vector<int> fixed_crop_y_cmds(num_fixed_windows_);
        std::vector<int> fixed_crop_w_cmds(num_fixed_windows_);
        std::vector<int> fixed_crop_h_cmds(num_fixed_windows_);
        for (int i = 0; i < num_fixed_windows_; i++)
        {
            fixed_vehicle_id_cmds[i] = agent_id_;
            fixed_camera_id_cmds[i] = fixed_camera_ids_[i];
            fixed_crop_x_cmds[i] = 0;
            fixed_crop_y_cmds[i] = 0;
            fixed_crop_w_cmds[i] = 0;
            fixed_crop_h_cmds[i] = 0;
        }
        framework_tools_->setWindowImageGroup(fixed_window_ids_, fixed_vehicle_id_cmds, fixed_camera_id_cmds, fixed_crop_x_cmds, fixed_crop_y_cmds, fixed_crop_w_cmds, fixed_crop_h_cmds);
    }

    void GuiController::setCentralWindow()
    {
        // Set central window parameters
        framework_tools_->setWindowImageGroup({ central_window_id_ }, { agent_id_ }, { central_camera_id_ }, { 0 }, { 0 }, { 0 }, { 0 });
    }

    void GuiController::setTrackingWindows()
    {
        // Get tracking goal under lock
        TrackingGoalMsg goal;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (!has_goal_)
                return;
            goal = goal_;
        }

        // Update tracking command vectors
        const int num_goals = static_cast<int>(goal.unit_types.size());
        for (int i = 0; i < std::min(num_tracking_windows_, num_goals); i++)
        {
            const auto& goal_type = static_cast<TrackingUnitType>(goal.unit_types[i]);

            switch (goal_type)
            {
            case TrackingUnitType::Physical:
            {
                // Physical tracking unit data
                const ID head_id = goal.head_ids[i];
                // Add tracking window (no crop)
                tracking_camera_id_cmds_[i] = head_id;
                tracking_crop_x_cmds_[i] = 0;
                tracking_crop_y_cmds_[i] = 0;
                tracking_crop_w_cmds_[i] = 0;
                tracking_crop_h_cmds_[i] = 0;
            }
            break;

            case TrackingUnitType::Digital:
            {
                // Digital tracking unit data
                const ID camera_id = goal.camera_id;
                const CropMsg crop = goal.crops[i];
                // Check if crop is out of bounds
                if (crop.is_out_of_bounds)
                {
                    tracking_camera_id_cmds_[i] = "";
                    continue;
                }
                // Add tracking window (with crop)
                tracking_camera_id_cmds_[i] = camera_id;
                tracking_crop_x_cmds_[i] = crop.x;
                tracking_crop_y_cmds_[i] = crop.y;
                tracking_crop_w_cmds_[i] = crop.w;
                tracking_crop_h_cmds_[i] = crop.h;
                // Update rectangle corners and sizes
                dig_rect_corners_[i].x = crop.x;
                dig_rect_corners_[i].y = crop.y;
                dig_rect_sizes_[i].x = crop.w;
                dig_rect_sizes_[i].y = crop.h;
                // Update string positions
                dig_string_pos_[i].x = crop.x;
                dig_string_pos_[i].y = crop.y - 200.0f;
            }
            break;

            default:
            {
                RCLCPP_ERROR(node_->get_logger(), "GuiController: Invalid tracking unit type: %d", static_cast<int>(goal_type));
                // Set empty image to current window
                tracking_camera_id_cmds_[i] = "";
                tracking_crop_x_cmds_[i] = 0;
                tracking_crop_y_cmds_[i] = 0;
                tracking_crop_w_cmds_[i] = 0;
                tracking_crop_h_cmds_[i] = 0;
            }
            break;
            };
        }

        // Set tracking view windows
        framework_tools_->setWindowImageGroup(tracking_window_ids_, tracking_vehicle_id_cmds_, tracking_camera_id_cmds_, tracking_crop_x_cmds_, tracking_crop_y_cmds_, tracking_crop_w_cmds_, tracking_crop_h_cmds_);
    }

    void GuiController::drawOnCentralWindow()
    {
        // Draw rectangles on central window
        framework_tools_->setWindowRectangles(central_window_id_, dig_rect_corners_, dig_rect_sizes_, dig_rect_color_, dig_rect_thickness_);

        // Draw strings on central window
        framework_tools_->setWindowStrings(central_window_id_, dig_strings_, dig_string_pos_, dig_string_color_, dig_string_scale_);
    }

} // namespace flychams::dashboard