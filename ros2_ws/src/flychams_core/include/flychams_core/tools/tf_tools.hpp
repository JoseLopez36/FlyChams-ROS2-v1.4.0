#pragma once

// Core includes
#include "flychams_core/types/core_types.hpp"
#include "flychams_core/types/ros_types.hpp"
#include "flychams_core/utils/ros_utils.hpp"

namespace flychams::core
{
    /**
     * ════════════════════════════════════════════════════════════════
     * @brief Transform Manager for handling transforms
     *
     * @details
     * This class provides utilities for managing transforms between different frames
     * in the ROS TF tree.
     * ════════════════════════════════════════════════════════════════
     * @author Jose Francisco Lopez Ruiz
     * @date 2025-02-28
     * ════════════════════════════════════════════════════════════════
     */
    class TfTools
    {
    public: // Types
        using SharedPtr = std::shared_ptr<TfTools>;
        // Agent frame patterns
        struct AgentFrames
        {
            std::string agent_local_pattern_;
            std::string agent_body_pattern_;
            std::string camera_body_pattern_;
            std::string camera_optical_pattern_;
        };

    private: // Data
        // Frames
        std::string global_frame_;
        AgentFrames agent_frames_;

        // ROS components
        NodePtr node_;
        BufferPtr tf_buffer_;
        ListenerPtr tf_listener_;

    public: // Constructor/Destructor
        TfTools(NodePtr node)
            : node_(node)
        {
            // Initialize ROS components
            tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
            tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

            // Get general frames
            global_frame_ = RosUtils::getParameter<std::string>(node_, "global_frame");

            // Get agent frames
            agent_frames_.agent_local_pattern_ = RosUtils::getParameter<std::string>(node_, "agent_frames.agent_local");
            agent_frames_.agent_body_pattern_ = RosUtils::getParameter<std::string>(node_, "agent_frames.agent_body");
            agent_frames_.camera_body_pattern_ = RosUtils::getParameter<std::string>(node_, "agent_frames.camera_body");
            agent_frames_.camera_optical_pattern_ = RosUtils::getParameter<std::string>(node_, "agent_frames.camera_optical");
        }

        ~TfTools()
        {
            shutdown();
        }

        void shutdown()
        {
            // Destroy tf listener and buffer
            tf_listener_.reset();
            tf_buffer_.reset();
            // Destroy node
            node_.reset();
        }

    public: // Frame getters
        std::string getGlobalFrame()
        {
            return global_frame_;
        }

        std::string getAgentLocalFrame(const std::string& agent_id)
        {
            return RosUtils::replacePlaceholder(agent_frames_.agent_local_pattern_, "AGENTID", agent_id);
        }

        std::string getAgentBodyFrame(const std::string& agent_id)
        {
            return RosUtils::replacePlaceholder(agent_frames_.agent_body_pattern_, "AGENTID", agent_id);
        }

        std::string getCameraBodyFrame(const std::string& agent_id, const std::string& head_id)
        {
            std::string pattern = RosUtils::replacePlaceholder(agent_frames_.camera_body_pattern_, "AGENTID", agent_id);
            return RosUtils::replacePlaceholder(pattern, "HEADID", head_id);
        }

        std::string getCameraOpticalFrame(const std::string& agent_id, const std::string& head_id)
        {
            std::string pattern = RosUtils::replacePlaceholder(agent_frames_.camera_optical_pattern_, "AGENTID", agent_id);
            return RosUtils::replacePlaceholder(pattern, "HEADID", head_id);
        }

    public: // Transformation utilities
        TransformStampedMsg createTransformMsg(const std::string& parent_frame, const std::string& child_frame, const core::Pose& pose)
        {
            TransformStampedMsg transform;
            transform.header.stamp = RosUtils::getTimeNow(node_);
            transform.header.frame_id = parent_frame;
            transform.child_frame_id = child_frame;

            // Set translation
            transform.transform.translation.x = pose.position.x();
            transform.transform.translation.y = pose.position.y();
            transform.transform.translation.z = pose.position.z();

            // Set rotation
            transform.transform.rotation.x = pose.orientation.x();
            transform.transform.rotation.y = pose.orientation.y();
            transform.transform.rotation.z = pose.orientation.z();
            transform.transform.rotation.w = pose.orientation.w();

            return transform;
        }

        PointMsg transformPointMsg(const PointMsg& point_msg, const std::string& from_frame, const std::string& to_frame)
        {
            if (from_frame == to_frame)
            {
                return point_msg;
            }

            // Stamp the pose
            PointStampedMsg point_stamped_msg;
            point_stamped_msg.header.frame_id = from_frame;
            point_stamped_msg.header.stamp = rclcpp::Time(0);
            point_stamped_msg.point = point_msg;

            try
            {
                // Transform the pose
                return tf_buffer_->transform(point_stamped_msg, to_frame).point;
            }
            catch (const tf2::TransformException& ex)
            {
                RCLCPP_ERROR(node_->get_logger(), "Transform failed: %s", ex.what());
                return point_msg;  // Return original point on failure
            }
        }

        Vector3Msg transformVelocityMsg(const Vector3Msg& velocity_msg, const std::string& from_frame, const std::string& to_frame)
        {
            if (from_frame == to_frame)
            {
                return velocity_msg;
            }

            // Get the transform between the source and target frames
            TransformStampedMsg transform;
            try
            {
                transform = tf_buffer_->lookupTransform(to_frame, from_frame, tf2::TimePointZero);
            }
            catch (const tf2::TransformException& ex)
            {
                RCLCPP_ERROR(node_->get_logger(), "Transform failed: %s", ex.what());
                return velocity_msg;  // Return original velocity on failure
            }

            // Convert to tf2 types for transformation
            tf2::Vector3 tf_velocity(velocity_msg.x, velocity_msg.y, velocity_msg.z);

            // Create transform object
            tf2::Quaternion rotation;
            tf2::fromMsg(transform.transform.rotation, rotation);

            // Rotate vector
            tf2::Vector3 tf_velocity_rotated = tf2::quatRotate(rotation, tf_velocity);

            // Pack results back into Vector3Msg
            Vector3Msg transformed_velocity_msg;
            transformed_velocity_msg.x = tf_velocity_rotated.x();
            transformed_velocity_msg.y = tf_velocity_rotated.y();
            transformed_velocity_msg.z = tf_velocity_rotated.z();

            return transformed_velocity_msg;
        }

        PoseMsg transformPoseMsg(const PoseMsg& pose_msg, const std::string& from_frame, const std::string& to_frame)
        {
            if (from_frame == to_frame)
            {
                return pose_msg;
            }

            // Stamp the pose
            PoseStampedMsg pose_stamped_msg;
            pose_stamped_msg.header.frame_id = from_frame;
            pose_stamped_msg.header.stamp = rclcpp::Time(0);
            pose_stamped_msg.pose = pose_msg;

            try
            {
                // Transform the pose
                return tf_buffer_->transform(pose_stamped_msg, to_frame).pose;
            }
            catch (const tf2::TransformException& ex)
            {
                RCLCPP_ERROR(node_->get_logger(), "Transform failed: %s", ex.what());
                return pose_msg;  // Return original pose on failure
            }
        }

        TwistMsg transformTwistMsg(const TwistMsg& twist_msg, const std::string& from_frame, const std::string& to_frame)
        {
            if (from_frame == to_frame)
            {
                return twist_msg;
            }

            // Get the transform between the source and target frames
            TransformStampedMsg transform;
            try
            {
                transform = tf_buffer_->lookupTransform(to_frame, from_frame, tf2::TimePointZero);
            }
            catch (const tf2::TransformException& ex)
            {
                RCLCPP_ERROR(node_->get_logger(), "Transform failed: %s", ex.what());
                return twist_msg;  // Return original twist on failure
            }

            // Convert to tf2 types for transformation
            tf2::Vector3 tf_linear(twist_msg.linear.x, twist_msg.linear.y, twist_msg.linear.z);
            tf2::Vector3 tf_angular(twist_msg.angular.x, twist_msg.angular.y, twist_msg.angular.z);

            // Create transform object
            tf2::Quaternion rotation;
            tf2::fromMsg(transform.transform.rotation, rotation);

            // Rotate vectors
            tf2::Vector3 tf_linear_rotated = tf2::quatRotate(rotation, tf_linear);
            tf2::Vector3 tf_angular_rotated = tf2::quatRotate(rotation, tf_angular);

            // Pack results back into TwistMsg
            TwistMsg transformed_twist_msg;
            transformed_twist_msg.linear.x = tf_linear_rotated.x();
            transformed_twist_msg.linear.y = tf_linear_rotated.y();
            transformed_twist_msg.linear.z = tf_linear_rotated.z();
            transformed_twist_msg.angular.x = tf_angular_rotated.x();
            transformed_twist_msg.angular.y = tf_angular_rotated.y();
            transformed_twist_msg.angular.z = tf_angular_rotated.z();

            return transformed_twist_msg;
        }

        OdometryMsg transformOdometryMsg(const OdometryMsg& odom, const std::string& target_frame)
        {
            if (odom.header.frame_id == target_frame)
            {
                return odom;
            }

            OdometryMsg transformed_odom_msg = odom;
            transformed_odom_msg.header.frame_id = target_frame;

            try
            {
                // Transform pose
                transformed_odom_msg.pose.pose = transformPoseMsg(odom.pose.pose, odom.header.frame_id, target_frame);

                // Transform twist
                transformed_odom_msg.twist.twist = transformTwistMsg(odom.twist.twist, odom.child_frame_id, target_frame);

                return transformed_odom_msg;
            }
            catch (const tf2::TransformException& ex)
            {
                RCLCPP_ERROR(node_->get_logger(), "Transform failed: %s", ex.what());
                return odom;  // Return original odometry on failure
            }
        }

        TransformMsg getTransformBetweenFrames(const std::string& from_frame, const std::string& to_frame)
        {
            // Initialize transform message
            TransformMsg transform_msg;

            try {
                // Get the transform from tf buffer with timeout
                geometry_msgs::msg::TransformStamped transform_stamped =
                    tf_buffer_->lookupTransform(from_frame, to_frame, tf2::TimePointZero);

                // Convert to TransformMsg
                transform_msg.translation = transform_stamped.transform.translation;
                transform_msg.rotation = transform_stamped.transform.rotation;
            }
            catch (const tf2::TransformException& ex) {
                RCLCPP_WARN(node_->get_logger(), "Could not get the transform from %s to %s: %s",
                    from_frame.c_str(), to_frame.c_str(), ex.what());

                // Set identity transform if transformation failed
                transform_msg = TransformMsg();
            }

            return transform_msg;
        }
    };

} // namespace flychams::core