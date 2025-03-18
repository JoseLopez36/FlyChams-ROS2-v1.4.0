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
        // Get window IDs
        scene_window_id_ = RosUtils::getParameter<core::ID>(node_, "window_ids.scene_id");
        agent_window_id_ = RosUtils::getParameter<core::ID>(node_, "window_ids.agent_id");
        central_window_id_ = RosUtils::getParameter<core::ID>(node_, "window_ids.central_id");
        tracking_window_ids_ = RosUtils::getParameter<std::vector<core::ID>>(node_, "window_ids.tracking_ids");
        num_windows_ = 3 + static_cast<int>(tracking_window_ids_.size());
        // Get fixed camera IDs
        scene_camera_id_ = RosUtils::getParameter<core::ID>(node_, "camera_ids.scene_id");
        agent_camera_id_ = RosUtils::replacePlaceholder(RosUtils::getParameter<core::ID>(node_, "camera_ids.agent_id"), "AGENTID", agent_id_);
        central_camera_id_ = config_tools_->getAgent(agent_id_)->central_head_id;

        // Initialize GUI data
        goal_ = TrackingGoalMsg();
        has_goal_ = false;

        // Create command vectors
        window_ids_ = core::IDs(num_windows_);
        vehicle_ids_ = core::IDs(num_windows_);
        camera_ids_ = core::IDs(num_windows_);
        crop_x_ = std::vector<int>(num_windows_);
        crop_y_ = std::vector<int>(num_windows_);
        crop_w_ = std::vector<int>(num_windows_);
        crop_h_ = std::vector<int>(num_windows_);

        // Set scene view window
        window_ids_[0] = scene_window_id_;
        vehicle_ids_[0] = "";
        camera_ids_[0] = scene_camera_id_;
        crop_x_[0] = 0;
        crop_y_[0] = 0;
        crop_w_[0] = 0;
        crop_h_[0] = 0;

        // Set agent view window
        window_ids_[1] = agent_window_id_;
        vehicle_ids_[1] = agent_id_;
        camera_ids_[1] = agent_camera_id_;
        crop_x_[1] = 0;
        crop_y_[1] = 0;
        crop_w_[1] = 0;
        crop_h_[1] = 0;

        // Set central view window
        window_ids_[2] = central_window_id_;
        vehicle_ids_[2] = agent_id_;
        camera_ids_[2] = central_camera_id_;
        crop_x_[2] = 0;
        crop_y_[2] = 0;
        crop_w_[2] = 0;
        crop_h_[2] = 0;

        // Initialize tracking windows
        for (size_t i = 3, j = 0; i < num_windows_; i++, j++)
        {
            window_ids_[i] = tracking_window_ids_[j];
            vehicle_ids_[i] = agent_id_;
            camera_ids_[i] = "";
            crop_x_[i] = 0;
            crop_y_[i] = 0;
            crop_w_[i] = 0;
            crop_h_[i] = 0;
        }

        // Initialize drawing data
        // Rectangle data
        // Cyan color
        rectangle_color_.r = 0.0f;
        rectangle_color_.g = 1.0f;
        rectangle_color_.b = 1.0f;
        rectangle_color_.a = 0.5f;
        rectangle_thickness_ = 2.0f;
        for (size_t i = 0; i < tracking_window_ids_.size(); i++)
        {
            PointMsg infinite_point;
            infinite_point.x = HUGE_VALF;
            infinite_point.y = HUGE_VALF;
            rectangle_corners_.push_back(infinite_point);
            rectangle_sizes_.push_back(infinite_point);
        }
        // String data
        // Cyan color
        string_color_.r = 0.0f;
        string_color_.g = 1.0f;
        string_color_.b = 1.0f;
        string_color_.a = 0.5f;
        string_scale_ = 128.0f;
        for (size_t i = 0; i < tracking_window_ids_.size(); i++)
        {
            PointMsg infinite_point;
            infinite_point.x = HUGE_VALF;
            infinite_point.y = HUGE_VALF;
            strings_.push_back("TW" + std::to_string(i));
            string_positions_.push_back(infinite_point);
        }

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

    void GuiController::updateAgent()
    {
        // Set target view windows
        ext_tools_->setWindowImageGroup(window_ids_, vehicle_ids_, camera_ids_, crop_x_, crop_y_, crop_w_, crop_h_);
    }

    void GuiController::updateTracking()
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
        for (size_t i = 3, j = 0; i < std::min(num_windows_, 3 + static_cast<int>(goal.window_ids.size())); i++, j++)
        {
            const auto& goal_window_id = goal.window_ids[j];
            const auto& goal_type = static_cast<TrackingUnitType>(goal.unit_types[j]);

            switch (goal_type)
            {
            case TrackingUnitType::Physical:
            {
                // Physical tracking unit data
                const ID head_id = goal.head_ids[j];
                // Add tracking window (no crop)
                camera_ids_[i] = head_id;
                crop_x_[i] = 0;
                crop_y_[i] = 0;
                crop_w_[i] = 0;
                crop_h_[i] = 0;
            }
            break;

            case TrackingUnitType::Digital:
            {
                // Digital tracking unit data
                const ID camera_id = goal.camera_id;
                const CropMsg crop = goal.crops[j];
                // Check if crop is valid
                if (crop.x <= 0.0f || crop.y <= 0.0f || crop.w <= 0.0f || crop.h <= 0.0f)
                {
                    camera_ids_[i] = "";
                    continue;
                }
                // Add tracking window (with crop)
                camera_ids_[i] = camera_id;
                crop_x_[i] = crop.x;
                crop_y_[i] = crop.y;
                crop_w_[i] = crop.w;
                crop_h_[i] = crop.h;
                // Update rectangle corners and sizes
                rectangle_corners_[j].x = crop.x;
                rectangle_corners_[j].y = crop.y;
                rectangle_sizes_[j].x = crop.w;
                rectangle_sizes_[j].y = crop.h;
                // Update string positions
                string_positions_[j].x = crop.x;
                string_positions_[j].y = crop.y - 200.0f;
            }
            break;

            default:
            {
                RCLCPP_ERROR(node_->get_logger(), "GuiController: Invalid tracking unit type: %d", static_cast<int>(goal_type));
                // Set empty image to current window
                camera_ids_[i] = "";
                crop_x_[i] = 0;
                crop_y_[i] = 0;
                crop_w_[i] = 0;
                crop_h_[i] = 0;
            }
            break;
            };
        }

        // Set tracking view windows
        ext_tools_->setWindowImageGroup(window_ids_, vehicle_ids_, camera_ids_, crop_x_, crop_y_, crop_w_, crop_h_);

        // Draw rectangles on central window
        ext_tools_->setWindowRectangles(central_window_id_, rectangle_corners_, rectangle_sizes_, rectangle_color_, rectangle_thickness_);

        // Draw strings on central window
        ext_tools_->setWindowStrings(central_window_id_, strings_, string_positions_, string_color_, string_scale_);
    }

} // namespace flychams::dashboard