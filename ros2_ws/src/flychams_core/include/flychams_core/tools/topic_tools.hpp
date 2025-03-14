#pragma once

// Core includes
#include "flychams_core/types/core_types.hpp"
#include "flychams_core/types/ros_types.hpp"
#include "flychams_core/utils/ros_utils.hpp"

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
    public: // Types
        using SharedPtr = std::shared_ptr<TopicTools>;
        // Global topics
        struct GlobalTopics
        {
            std::string metrics;
        };
        // Agent topics
        struct AgentTopics
        {
            std::string registration;
            std::string odom_pattern;
            std::string goal_pattern;
            std::string info_pattern;
            std::string tracking_goal_pattern;
            std::string metrics_pattern;
            std::string markers_pattern;
        };
        // Target topics
        struct TargetTopics
        {
            std::string registration;
            std::string info_pattern;
            std::string metrics_pattern;
            std::string markers_pattern;
        };
        // Cluster topics
        struct ClusterTopics
        {
            std::string registration;
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

    public: // Constructor/Destructor
        TopicTools(NodePtr node)
            : node_(node)
        {
            // Get global topics
            global_topics_.metrics = RosUtils::getParameter<std::string>(node_, "global_topics.metrics");

            // Get agent topics
            agent_topics_.registration = RosUtils::getParameter<std::string>(node_, "agent_topics.registration");
            agent_topics_.odom_pattern = RosUtils::getParameter<std::string>(node_, "agent_topics.odom");
            agent_topics_.goal_pattern = RosUtils::getParameter<std::string>(node_, "agent_topics.goal");
            agent_topics_.info_pattern = RosUtils::getParameter<std::string>(node_, "agent_topics.info");
            agent_topics_.tracking_goal_pattern = RosUtils::getParameter<std::string>(node_, "agent_topics.tracking_goal");
            agent_topics_.metrics_pattern = RosUtils::getParameter<std::string>(node_, "agent_topics.metrics");
            agent_topics_.markers_pattern = RosUtils::getParameter<std::string>(node_, "agent_topics.markers");

            // Get target topics
            target_topics_.registration = RosUtils::getParameter<std::string>(node_, "target_topics.registration");
            target_topics_.info_pattern = RosUtils::getParameter<std::string>(node_, "target_topics.info");
            target_topics_.metrics_pattern = RosUtils::getParameter<std::string>(node_, "target_topics.metrics");
            target_topics_.markers_pattern = RosUtils::getParameter<std::string>(node_, "target_topics.markers");

            // Get cluster topics
            cluster_topics_.registration = RosUtils::getParameter<std::string>(node_, "cluster_topics.registration");
            cluster_topics_.info_pattern = RosUtils::getParameter<std::string>(node_, "cluster_topics.info");
            cluster_topics_.metrics_pattern = RosUtils::getParameter<std::string>(node_, "cluster_topics.metrics");
            cluster_topics_.markers_pattern = RosUtils::getParameter<std::string>(node_, "cluster_topics.markers");
        }

        ~TopicTools()
        {
            shutdown();
        }

        void shutdown()
        {
            // Destroy node
            node_.reset();
        }

    public: // Topic getters
        std::string getGlobalMetricsTopic()
        {
            return global_topics_.metrics;
        }
        std::string getAgentRegistrationTopic()
        {
            return agent_topics_.registration;
        }
        std::string getAgentOdomTopic(const ID& agent_id)
        {
            return RosUtils::replacePlaceholder(agent_topics_.odom_pattern, "AGENTID", agent_id);
        }
        std::string getAgentGoalTopic(const ID& agent_id)
        {
            return RosUtils::replacePlaceholder(agent_topics_.goal_pattern, "AGENTID", agent_id);
        }
        std::string getAgentInfoTopic(const ID& agent_id)
        {
            return RosUtils::replacePlaceholder(agent_topics_.info_pattern, "AGENTID", agent_id);
        }
        std::string getTrackingGoalTopic(const ID& agent_id)
        {
            return RosUtils::replacePlaceholder(agent_topics_.tracking_goal_pattern, "AGENTID", agent_id);
        }
        std::string getAgentMetricsTopic(const ID& agent_id)
        {
            return RosUtils::replacePlaceholder(agent_topics_.metrics_pattern, "AGENTID", agent_id);
        }
        std::string getAgentMarkersTopic(const ID& agent_id)
        {
            return RosUtils::replacePlaceholder(agent_topics_.markers_pattern, "AGENTID", agent_id);
        }
        std::string getTargetRegistrationTopic()
        {
            return target_topics_.registration;
        }
        std::string getTargetInfoTopic(const ID& target_id)
        {
            return RosUtils::replacePlaceholder(target_topics_.info_pattern, "TARGETID", target_id);
        }
        std::string getTargetMetricsTopic(const ID& target_id)
        {
            return RosUtils::replacePlaceholder(target_topics_.metrics_pattern, "TARGETID", target_id);
        }
        std::string getTargetMarkersTopic(const ID& target_id)
        {
            return RosUtils::replacePlaceholder(target_topics_.markers_pattern, "TARGETID", target_id);
        }
        std::string getClusterRegistrationTopic()
        {
            return cluster_topics_.registration;
        }
        std::string getClusterInfoTopic(const ID& cluster_id)
        {
            return RosUtils::replacePlaceholder(cluster_topics_.info_pattern, "CLUSTERID", cluster_id);
        }
        std::string getClusterMetricsTopic(const ID& cluster_id)
        {
            return RosUtils::replacePlaceholder(cluster_topics_.metrics_pattern, "CLUSTERID", cluster_id);
        }
        std::string getClusterMarkersTopic(const ID& cluster_id)
        {
            return RosUtils::replacePlaceholder(cluster_topics_.markers_pattern, "CLUSTERID", cluster_id);
        }

    public: // Topic creation utilities
        // Publishers
        PublisherPtr<GlobalMetricsMsg> createGlobalMetricsPublisher()
        {
            return node_->create_publisher<GlobalMetricsMsg>(getGlobalMetricsTopic(), 10);
        }
        PublisherPtr<RegistrationMsg> createAgentRegistrationPublisher()
        {
            rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
            return node_->create_publisher<RegistrationMsg>(getAgentRegistrationTopic(), qos);
        }
        PublisherPtr<AgentGoalMsg> createAgentGoalPublisher(const ID& agent_id)
        {
            return node_->create_publisher<AgentGoalMsg>(getAgentGoalTopic(agent_id), 10);
        }
        PublisherPtr<AgentInfoMsg> createAgentInfoPublisher(const ID& agent_id)
        {
            return node_->create_publisher<AgentInfoMsg>(getAgentInfoTopic(agent_id), 10);
        }
        PublisherPtr<TrackingGoalMsg> createTrackingGoalPublisher(const ID& agent_id)
        {
            return node_->create_publisher<TrackingGoalMsg>(getTrackingGoalTopic(agent_id), 10);
        }
        PublisherPtr<AgentMetricsMsg> createAgentMetricsPublisher(const ID& agent_id)
        {
            return node_->create_publisher<AgentMetricsMsg>(getAgentMetricsTopic(agent_id), 10);
        }
        PublisherPtr<MarkerArrayMsg> createAgentMarkersPublisher(const ID& agent_id)
        {
            return node_->create_publisher<MarkerArrayMsg>(getAgentMarkersTopic(agent_id), 10);
        }
        PublisherPtr<RegistrationMsg> createTargetRegistrationPublisher()
        {
            rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
            return node_->create_publisher<RegistrationMsg>(getTargetRegistrationTopic(), qos);
        }
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
        PublisherPtr<RegistrationMsg> createClusterRegistrationPublisher()
        {
            rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
            return node_->create_publisher<RegistrationMsg>(getClusterRegistrationTopic(), qos);
        }
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
        SubscriberPtr<RegistrationMsg> createAgentRegistrationSubscriber(const std::function<void(const RegistrationMsg::SharedPtr)>& callback, const rclcpp::SubscriptionOptions& options = rclcpp::SubscriptionOptions())
        {
            rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
            return node_->create_subscription<RegistrationMsg>(getAgentRegistrationTopic(), qos, callback, options);
        }
        SubscriberPtr<OdometryMsg> createAgentOdomSubscriber(const ID& agent_id, const std::function<void(const OdometryMsg::SharedPtr)>& callback, const rclcpp::SubscriptionOptions& options = rclcpp::SubscriptionOptions())
        {
            return node_->create_subscription<OdometryMsg>(getAgentOdomTopic(agent_id), 10, callback, options);
        }
        SubscriberPtr<AgentGoalMsg> createAgentGoalSubscriber(const ID& agent_id, const std::function<void(const AgentGoalMsg::SharedPtr)>& callback, const rclcpp::SubscriptionOptions& options = rclcpp::SubscriptionOptions())
        {
            return node_->create_subscription<AgentGoalMsg>(getAgentGoalTopic(agent_id), 10, callback, options);
        }
        SubscriberPtr<AgentInfoMsg> createAgentInfoSubscriber(const ID& agent_id, const std::function<void(const AgentInfoMsg::SharedPtr)>& callback, const rclcpp::SubscriptionOptions& options = rclcpp::SubscriptionOptions())
        {
            return node_->create_subscription<AgentInfoMsg>(getAgentInfoTopic(agent_id), 10, callback, options);
        }
        SubscriberPtr<TrackingGoalMsg> createTrackingGoalSubscriber(const ID& agent_id, const std::function<void(const TrackingGoalMsg::SharedPtr)>& callback, const rclcpp::SubscriptionOptions& options = rclcpp::SubscriptionOptions())
        {
            return node_->create_subscription<TrackingGoalMsg>(getTrackingGoalTopic(agent_id), 10, callback, options);
        }
        SubscriberPtr<RegistrationMsg> createTargetRegistrationSubscriber(const std::function<void(const RegistrationMsg::SharedPtr)>& callback, const rclcpp::SubscriptionOptions& options = rclcpp::SubscriptionOptions())
        {
            rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
            return node_->create_subscription<RegistrationMsg>(getTargetRegistrationTopic(), qos, callback, options);
        }
        SubscriberPtr<TargetInfoMsg> createTargetInfoSubscriber(const ID& target_id, const std::function<void(const TargetInfoMsg::SharedPtr)>& callback, const rclcpp::SubscriptionOptions& options = rclcpp::SubscriptionOptions())
        {
            return node_->create_subscription<TargetInfoMsg>(getTargetInfoTopic(target_id), 10, callback, options);
        }
        SubscriberPtr<RegistrationMsg> createClusterRegistrationSubscriber(const std::function<void(const RegistrationMsg::SharedPtr)>& callback, const rclcpp::SubscriptionOptions& options = rclcpp::SubscriptionOptions())
        {
            rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
            return node_->create_subscription<RegistrationMsg>(getClusterRegistrationTopic(), qos, callback, options);
        }
        SubscriberPtr<ClusterInfoMsg> createClusterInfoSubscriber(const ID& cluster_id, const std::function<void(const ClusterInfoMsg::SharedPtr)>& callback, const rclcpp::SubscriptionOptions& options = rclcpp::SubscriptionOptions())
        {
            return node_->create_subscription<ClusterInfoMsg>(getClusterInfoTopic(cluster_id), 10, callback, options);
        }
    };

} // namespace flychams::core 