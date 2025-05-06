#pragma once

// Tools includes
#include "flychams_core/config/config_tools.hpp"

namespace flychams::core
{
    /**
     * ════════════════════════════════════════════════════════════════
     * @brief Transform Manager for handling ROS2 transforms
     *
     * @details
     * This class provides utilities for managing ROS2 transforms between
     * different frames in the ROS TF tree.
     *
     * ════════════════════════════════════════════════════════════════
     * @author Jose Francisco Lopez Ruiz
     * @date 2025-02-28
     * ════════════════════════════════════════════════════════════════
     */
    class TransformTools
    {
    public: // Constructor/Destructor
        TransformTools(NodePtr node, const ConfigTools::SharedPtr& config_tools)
            : node_(node), config_tools_(config_tools)
        {
            // Initialize ROS components
            tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
            tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
            tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);
            static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node_);

            // Get frame config
            const auto& frame_config = config_tools_->getFrames();

            // Get general frames
            world_frame_ = frame_config.world;

            // Get agent frames
            agent_frames_.agent_local_pattern_ = frame_config.agent_local;
            agent_frames_.agent_body_pattern_ = frame_config.agent_body;
            agent_frames_.camera_body_pattern_ = frame_config.camera_body;
            agent_frames_.camera_optical_pattern_ = frame_config.camera_optical;
        }

        ~TransformTools()
        {
            shutdown();
        }

        void shutdown()
        {
            // Destroy tf listener and buffer
            tf_listener_.reset();
            tf_buffer_.reset();
            tf_broadcaster_.reset();
            static_tf_broadcaster_.reset();
            // Destroy config tools
            config_tools_.reset();
            // Destroy node
            node_.reset();
        }

    public: // Types
        using SharedPtr = std::shared_ptr<TransformTools>;
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
        std::string world_frame_;
        AgentFrames agent_frames_;

        // ROS components
        NodePtr node_;
        BufferPtr tf_buffer_;
        ListenerPtr tf_listener_;
        BroadcasterPtr tf_broadcaster_;
        StaticBroadcasterPtr static_tf_broadcaster_;

        // Config tools
        ConfigTools::SharedPtr config_tools_;

    public: // Frame getters
        std::string getGlobalFrame()
        {
            return world_frame_;
        }

        std::string getAgentLocalFrame(const std::string& agent_id)
        {
            return RosUtils::replace(agent_frames_.agent_local_pattern_, "AGENTID", agent_id);
        }

        std::string getAgentBodyFrame(const std::string& agent_id)
        {
            return RosUtils::replace(agent_frames_.agent_body_pattern_, "AGENTID", agent_id);
        }

        std::string getCameraBodyFrame(const std::string& agent_id, const std::string& camera_id)
        {
            std::string pattern = RosUtils::replace(agent_frames_.camera_body_pattern_, "AGENTID", agent_id);
            return RosUtils::replace(pattern, "HEADID", camera_id);
        }

        std::string getCameraOpticalFrame(const std::string& agent_id, const std::string& camera_id)
        {
            std::string pattern = RosUtils::replace(agent_frames_.camera_optical_pattern_, "AGENTID", agent_id);
            return RosUtils::replace(pattern, "HEADID", camera_id);
        }

    public: // Transform utilities
        TransformMsg getTransform(const std::string& from_frame, const std::string& to_frame)
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

    public: // Transform broadcast utilities
        void broadcastTransform(const std::string& from_frame, const std::string& to_frame, const Matrix4r& transform)
        {
            // Create stamped transform message
            geometry_msgs::msg::TransformStamped transform_msg;
            transform_msg.header.stamp = RosUtils::now(node_);
            transform_msg.header.frame_id = from_frame;
            transform_msg.child_frame_id = to_frame;
            RosUtils::toMsg(transform, transform_msg.transform);

            // Broadcast transform
            tf_broadcaster_->sendTransform(transform_msg);
        }

        void broadcastStaticTransform(const std::string& from_frame, const std::string& to_frame, const Matrix4r& transform)
        {
            // Create stamped transform message
            geometry_msgs::msg::TransformStamped transform_msg;
            transform_msg.header.stamp = RosUtils::now(node_);
            transform_msg.header.frame_id = from_frame;
            transform_msg.child_frame_id = to_frame;
            RosUtils::toMsg(transform, transform_msg.transform);

            // Broadcast static transform
            static_tf_broadcaster_->sendTransform(transform_msg);
        }
    };

} // namespace flychams::core