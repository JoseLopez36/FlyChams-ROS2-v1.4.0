#include "flychams_bringup/target/target_controller.hpp"

using namespace std::chrono_literals;
using namespace flychams::core;

namespace flychams::bringup
{
    // ════════════════════════════════════════════════════════════════════════════
    // CONSTRUCTOR: Constructor and destructor
    // ════════════════════════════════════════════════════════════════════════════

    void TargetController::onInit()
    {
        // Get parameters from parameter server
        // Get update rate
        float update_rate = RosUtils::getParameterOr<float>(node_, "target_registration.target_update_rate", 20.0f);

        // Initialize target info publisher
        info_pub_ = topic_tools_->createTargetInfoPublisher(target_id_);
    }

    void TargetController::onShutdown()
    {
        // Destroy trajectory
        trajectory_.reset();
        // Destroy publisher
        info_pub_.reset();
    }

    // ════════════════════════════════════════════════════════════════════════════
    // PUBLIC METHODS: Methods for initializing and updating the target
    // ════════════════════════════════════════════════════════════════════════════

    void TargetController::initializeTrajectory(const std::string& trajectory_path)
    {
        // Initialize trajectory
        trajectory_ = std::make_shared<TargetTrajectory>();
        trajectory_->parse(trajectory_path);

        // Initialize position
        position_.x = trajectory_->getStartPoint().x;
        position_.y = trajectory_->getStartPoint().y;
        position_.z = trajectory_->getStartPoint().z;
    }

    void TargetController::updateControl(const float& dt)
    {
        // Update target position
        const auto& curr_point = updatePosition(dt);
        position_.x = curr_point.x;
        position_.y = curr_point.y;
        position_.z = curr_point.z;
    }

    PointMsg TargetController::getPosition() const
    {
        return position_;
    }

    // ════════════════════════════════════════════════════════════════════════════
    // PRIVATE METHODS: Methods for updating the target
    // ════════════════════════════════════════════════════════════════════════════

    TargetTrajectory::Point TargetController::updatePosition(const float& dt)
    {
        // Update trajectory
        TargetTrajectory::Point point = trajectory_->update(dt);

        // Create target info message
        TargetInfoMsg msg;
        msg.header = RosUtils::createHeader(node_, tf_tools_->getGlobalFrame());
        msg.position.x = point.x;
        msg.position.y = point.y;
        msg.position.z = point.z;

        // Publish target info
        info_pub_->publish(msg);

        // Return current point
        return point;
    }

} // namespace flychams::bringup