#pragma once

// Tools includes
#include "flychams_core/config/config_tools.hpp"

namespace flychams::core
{
    /**
     * ════════════════════════════════════════════════════════════════
     * @brief Topic Manager for handling topics
     *
     * @details
     * This class provides utilities for managing topics
     * ════════════════════════════════════════════════════════════════
     * @author Jose Francisco Lopez Ruiz
     * @date 2025-02-28
     * ════════════════════════════════════════════════════════════════
     */
    class TopicTools
    {
    public: // Constructor/Destructor
        TopicTools(NodePtr node, const ConfigTools::SharedPtr& config_tools)
            : node_(node), config_tools_(config_tools)
        {
            // Get topic config
            const auto& topic_config = config_tools_->getTopics();

            // Get global topics
            global_topics_.registration = topic_config.registration;
            global_topics_.metrics = topic_config.global_metrics;

            // Get agent topics
            agent_topics_.global_odom_pattern = topic_config.agent_global_odom;
            agent_topics_.position_goal_pattern = topic_config.agent_position_goal;
            agent_topics_.tracking_info_pattern = topic_config.agent_tracking_info;
            agent_topics_.tracking_goal_pattern = topic_config.agent_tracking_goal;
            agent_topics_.metrics_pattern = topic_config.agent_metrics;
            agent_topics_.markers_pattern = topic_config.agent_markers;

            // Get target topics
            target_topics_.info_pattern = topic_config.target_info;
            target_topics_.metrics_pattern = topic_config.target_metrics;
            target_topics_.markers_pattern = topic_config.target_markers;

            // Get cluster topics
            cluster_topics_.info_pattern = topic_config.cluster_info;
            cluster_topics_.metrics_pattern = topic_config.cluster_metrics;
            cluster_topics_.markers_pattern = topic_config.cluster_markers;
        }

        ~TopicTools()
        {
            shutdown();
        }

        void shutdown()
        {
            // Destroy config tools
            config_tools_.reset();
            // Destroy node
            node_.reset();
        }

    public: // Types
        using SharedPtr = std::shared_ptr<TopicTools>;
        // Global topics
        struct GlobalTopics
        {
            std::string registration;
            std::string metrics;
        };
        // Agent topics
        struct AgentTopics
        {
            std::string global_odom_pattern;
            std::string position_goal_pattern;
            std::string tracking_info_pattern;
            std::string tracking_goal_pattern;
            std::string metrics_pattern;
            std::string markers_pattern;
        };
        // Target topics
        struct TargetTopics
        {
            std::string info_pattern;
            std::string metrics_pattern;
            std::string markers_pattern;
        };
        // Cluster topics
        struct ClusterTopics
        {
            std::string info_pattern;
            std::string metrics_pattern;
            std::string markers_pattern;
        };

    private: // Data
        // Topics
        GlobalTopics global_topics_;
        AgentTopics agent_topics_;
        TargetTopics target_topics_;
        ClusterTopics cluster_topics_;

        // ROS components
        NodePtr node_;

        // Config tools
        ConfigTools::SharedPtr config_tools_;

    public: // Topic getters
        // Global topics
        std::string getRegistrationTopic()
        {
            return global_topics_.registration;
        }
        std::string getGlobalMetricsTopic()
        {
            return global_topics_.metrics;
        }
        // Agent topics
        std::string getAgentOdomTopic(const ID& agent_id)
        {
            return RosUtils::replace(agent_topics_.global_odom_pattern, "AGENTID", agent_id);
        }
        std::string getAgentPositionGoalTopic(const ID& agent_id)
        {
            return RosUtils::replace(agent_topics_.position_goal_pattern, "AGENTID", agent_id);
        }
        std::string getAgentTrackingInfoTopic(const ID& agent_id)
        {
            return RosUtils::replace(agent_topics_.tracking_info_pattern, "AGENTID", agent_id);
        }
        std::string getAgentTrackingGoalTopic(const ID& agent_id)
        {
            return RosUtils::replace(agent_topics_.tracking_goal_pattern, "AGENTID", agent_id);
        }
        std::string getAgentMetricsTopic(const ID& agent_id)
        {
            return RosUtils::replace(agent_topics_.metrics_pattern, "AGENTID", agent_id);
        }
        std::string getAgentMarkersTopic(const ID& agent_id)
        {
            return RosUtils::replace(agent_topics_.markers_pattern, "AGENTID", agent_id);
        }
        // Target topics
        std::string getTargetInfoTopic(const ID& target_id)
        {
            return RosUtils::replace(target_topics_.info_pattern, "TARGETID", target_id);
        }
        std::string getTargetMetricsTopic(const ID& target_id)
        {
            return RosUtils::replace(target_topics_.metrics_pattern, "TARGETID", target_id);
        }
        std::string getTargetMarkersTopic(const ID& target_id)
        {
            return RosUtils::replace(target_topics_.markers_pattern, "TARGETID", target_id);
        }
        // Cluster topics
        std::string getClusterInfoTopic(const ID& cluster_id)
        {
            return RosUtils::replace(cluster_topics_.info_pattern, "CLUSTERID", cluster_id);
        }
        std::string getClusterMetricsTopic(const ID& cluster_id)
        {
            return RosUtils::replace(cluster_topics_.metrics_pattern, "CLUSTERID", cluster_id);
        }
        std::string getClusterMarkersTopic(const ID& cluster_id)
        {
            return RosUtils::replace(cluster_topics_.markers_pattern, "CLUSTERID", cluster_id);
        }

    public: // Topic creation utilities
        // Publishers
        // Global publishers
        PublisherPtr<RegistrationMsg> createRegistrationPublisher()
        {
            rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
            return node_->create_publisher<RegistrationMsg>(getRegistrationTopic(), qos);
        }
        PublisherPtr<GlobalMetricsMsg> createGlobalMetricsPublisher()
        {
            return node_->create_publisher<GlobalMetricsMsg>(getGlobalMetricsTopic(), 10);
        }
        // Agent publishers
        PublisherPtr<PositionGoalMsg> createAgentPositionGoalPublisher(const ID& agent_id)
        {
            return node_->create_publisher<PositionGoalMsg>(getAgentPositionGoalTopic(agent_id), 10);
        }
        PublisherPtr<TrackingInfoMsg> createAgentTrackingInfoPublisher(const ID& agent_id)
        {
            return node_->create_publisher<TrackingInfoMsg>(getAgentTrackingInfoTopic(agent_id), 10);
        }
        PublisherPtr<TrackingGoalMsg> createAgentTrackingGoalPublisher(const ID& agent_id)
        {
            return node_->create_publisher<TrackingGoalMsg>(getAgentTrackingGoalTopic(agent_id), 10);
        }
        PublisherPtr<AgentMetricsMsg> createAgentMetricsPublisher(const ID& agent_id)
        {
            return node_->create_publisher<AgentMetricsMsg>(getAgentMetricsTopic(agent_id), 10);
        }
        PublisherPtr<MarkerArrayMsg> createAgentMarkersPublisher(const ID& agent_id)
        {
            return node_->create_publisher<MarkerArrayMsg>(getAgentMarkersTopic(agent_id), 10);
        }
        // Target publishers
        PublisherPtr<TargetInfoMsg> createTargetInfoPublisher(const ID& target_id)
        {
            return node_->create_publisher<TargetInfoMsg>(getTargetInfoTopic(target_id), 10);
        }
        PublisherPtr<TargetMetricsMsg> createTargetMetricsPublisher(const ID& target_id)
        {
            return node_->create_publisher<TargetMetricsMsg>(getTargetMetricsTopic(target_id), 10);
        }
        PublisherPtr<MarkerArrayMsg> createTargetMarkersPublisher(const ID& target_id)
        {
            return node_->create_publisher<MarkerArrayMsg>(getTargetMarkersTopic(target_id), 10);
        }
        // Cluster publishers
        PublisherPtr<ClusterInfoMsg> createClusterInfoPublisher(const ID& cluster_id)
        {
            return node_->create_publisher<ClusterInfoMsg>(getClusterInfoTopic(cluster_id), 10);
        }
        PublisherPtr<ClusterMetricsMsg> createClusterMetricsPublisher(const ID& cluster_id)
        {
            return node_->create_publisher<ClusterMetricsMsg>(getClusterMetricsTopic(cluster_id), 10);
        }
        PublisherPtr<MarkerArrayMsg> createClusterMarkersPublisher(const ID& cluster_id)
        {
            return node_->create_publisher<MarkerArrayMsg>(getClusterMarkersTopic(cluster_id), 10);
        }

        // Subscribers
        // Global subscribers
        SubscriberPtr<RegistrationMsg> createRegistrationSubscriber(const std::function<void(const RegistrationMsg::SharedPtr)>& callback, const rclcpp::SubscriptionOptions& options = rclcpp::SubscriptionOptions())
        {
            rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
            return node_->create_subscription<RegistrationMsg>(getRegistrationTopic(), qos, callback, options);
        }
        // Agent subscribers
        SubscriberPtr<OdometryMsg> createAgentOdomSubscriber(const ID& agent_id, const std::function<void(const OdometryMsg::SharedPtr)>& callback, const rclcpp::SubscriptionOptions& options = rclcpp::SubscriptionOptions())
        {
            return node_->create_subscription<OdometryMsg>(getAgentOdomTopic(agent_id), 10, callback, options);
        }
        SubscriberPtr<PositionGoalMsg> createAgentPositionGoalSubscriber(const ID& agent_id, const std::function<void(const PositionGoalMsg::SharedPtr)>& callback, const rclcpp::SubscriptionOptions& options = rclcpp::SubscriptionOptions())
        {
            return node_->create_subscription<PositionGoalMsg>(getAgentPositionGoalTopic(agent_id), 10, callback, options);
        }
        SubscriberPtr<TrackingInfoMsg> createAgentTrackingInfoSubscriber(const ID& agent_id, const std::function<void(const TrackingInfoMsg::SharedPtr)>& callback, const rclcpp::SubscriptionOptions& options = rclcpp::SubscriptionOptions())
        {
            return node_->create_subscription<TrackingInfoMsg>(getAgentTrackingInfoTopic(agent_id), 10, callback, options);
        }
        SubscriberPtr<TrackingGoalMsg> createAgentTrackingGoalSubscriber(const ID& agent_id, const std::function<void(const TrackingGoalMsg::SharedPtr)>& callback, const rclcpp::SubscriptionOptions& options = rclcpp::SubscriptionOptions())
        {
            return node_->create_subscription<TrackingGoalMsg>(getAgentTrackingGoalTopic(agent_id), 10, callback, options);
        }
        // Target subscribers
        SubscriberPtr<TargetInfoMsg> createTargetInfoSubscriber(const ID& target_id, const std::function<void(const TargetInfoMsg::SharedPtr)>& callback, const rclcpp::SubscriptionOptions& options = rclcpp::SubscriptionOptions())
        {
            return node_->create_subscription<TargetInfoMsg>(getTargetInfoTopic(target_id), 10, callback, options);
        }
        // Cluster subscribers
        SubscriberPtr<ClusterInfoMsg> createClusterInfoSubscriber(const ID& cluster_id, const std::function<void(const ClusterInfoMsg::SharedPtr)>& callback, const rclcpp::SubscriptionOptions& options = rclcpp::SubscriptionOptions())
        {
            return node_->create_subscription<ClusterInfoMsg>(getClusterInfoTopic(cluster_id), 10, callback, options);
        }
    };

} // namespace flychams::core 