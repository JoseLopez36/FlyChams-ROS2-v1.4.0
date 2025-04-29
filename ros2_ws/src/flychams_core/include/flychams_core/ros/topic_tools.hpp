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
            agent_topics_.status_pattern = topic_config.agent_status;
            agent_topics_.position_pattern = topic_config.agent_position;
            agent_topics_.assignment_pattern = topic_config.agent_assignment;
            agent_topics_.clusters_pattern = topic_config.agent_clusters;
            agent_topics_.position_setpoint_pattern = topic_config.agent_position_setpoint;
            agent_topics_.tracking_setpoints_pattern = topic_config.agent_tracking_setpoints;
            agent_topics_.metrics_pattern = topic_config.agent_metrics;
            agent_topics_.markers_pattern = topic_config.agent_markers;

            // Get target topics
            target_topics_.true_position_pattern = topic_config.target_true_position;
            target_topics_.est_position_pattern = topic_config.target_est_position;
            target_topics_.metrics_pattern = topic_config.target_metrics;
            target_topics_.markers_pattern = topic_config.target_markers;

            // Get cluster topics
            cluster_topics_.assignment_pattern = topic_config.cluster_assignment;
            cluster_topics_.geometry_pattern = topic_config.cluster_geometry;
            cluster_topics_.metrics_pattern = topic_config.cluster_metrics;
            cluster_topics_.markers_pattern = topic_config.cluster_markers;

            // Get GUI topics
            gui_topics_.setpoints_pattern = topic_config.gui_setpoints;
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
            std::string status_pattern;
            std::string position_pattern;
            std::string assignment_pattern;
            std::string clusters_pattern;
            std::string position_setpoint_pattern;
            std::string tracking_setpoints_pattern;
            std::string metrics_pattern;
            std::string markers_pattern;
        };
        // Target topics
        struct TargetTopics
        {
            std::string true_position_pattern;
            std::string est_position_pattern;
            std::string metrics_pattern;
            std::string markers_pattern;
        };
        // Cluster topics
        struct ClusterTopics
        {
            std::string assignment_pattern;
            std::string geometry_pattern;
            std::string metrics_pattern;
            std::string markers_pattern;
        };
        // GUI topics
        struct GuiTopics
        {
            std::string setpoints_pattern;
        };

    private: // Data
        // Topics
        GlobalTopics global_topics_;
        AgentTopics agent_topics_;
        TargetTopics target_topics_;
        ClusterTopics cluster_topics_;
        GuiTopics gui_topics_;

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
        std::string getAgentStatusTopic(const ID& agent_id)
        {
            return RosUtils::replace(agent_topics_.status_pattern, "AGENTID", agent_id);
        }
        std::string getAgentPositionTopic(const ID& agent_id)
        {
            return RosUtils::replace(agent_topics_.position_pattern, "AGENTID", agent_id);
        }
        std::string getAgentAssignmentTopic(const ID& agent_id)
        {
            return RosUtils::replace(agent_topics_.assignment_pattern, "AGENTID", agent_id);
        }
        std::string getAgentClustersTopic(const ID& agent_id)
        {
            return RosUtils::replace(agent_topics_.clusters_pattern, "AGENTID", agent_id);
        }
        std::string getAgentPositionSetpointTopic(const ID& agent_id)
        {
            return RosUtils::replace(agent_topics_.position_setpoint_pattern, "AGENTID", agent_id);
        }
        std::string getAgentTrackingSetpointsTopic(const ID& agent_id)
        {
            return RosUtils::replace(agent_topics_.tracking_setpoints_pattern, "AGENTID", agent_id);
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
        std::string getTargetTruePositionTopic(const ID& target_id)
        {
            return RosUtils::replace(target_topics_.true_position_pattern, "TARGETID", target_id);
        }
        std::string getTargetEstPositionTopic(const ID& target_id)
        {
            return RosUtils::replace(target_topics_.est_position_pattern, "TARGETID", target_id);
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
        std::string getClusterAssignmentTopic(const ID& cluster_id)
        {
            return RosUtils::replace(cluster_topics_.assignment_pattern, "CLUSTERID", cluster_id);
        }
        std::string getClusterGeometryTopic(const ID& cluster_id)
        {
            return RosUtils::replace(cluster_topics_.geometry_pattern, "CLUSTERID", cluster_id);
        }
        std::string getClusterMetricsTopic(const ID& cluster_id)
        {
            return RosUtils::replace(cluster_topics_.metrics_pattern, "CLUSTERID", cluster_id);
        }
        std::string getClusterMarkersTopic(const ID& cluster_id)
        {
            return RosUtils::replace(cluster_topics_.markers_pattern, "CLUSTERID", cluster_id);
        }

        // GUI topics
        std::string getGuiSetpointsTopic(const ID& agent_id)
        {
            return RosUtils::replace(gui_topics_.setpoints_pattern, "AGENTID", agent_id);
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
        PublisherPtr<AgentStatusMsg> createAgentStatusPublisher(const ID& agent_id)
        {
            rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
            return node_->create_publisher<AgentStatusMsg>(getAgentStatusTopic(agent_id), qos);
        }
        PublisherPtr<PointStampedMsg> createAgentPositionPublisher(const ID& agent_id)
        {
            return node_->create_publisher<PointStampedMsg>(getAgentPositionTopic(agent_id), 10);
        }
        PublisherPtr<AgentAssignmentMsg> createAgentAssignmentPublisher(const ID& agent_id)
        {
            rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
            return node_->create_publisher<AgentAssignmentMsg>(getAgentAssignmentTopic(agent_id), qos);
        }
        PublisherPtr<AgentClustersMsg> createAgentClustersPublisher(const ID& agent_id)
        {
            return node_->create_publisher<AgentClustersMsg>(getAgentClustersTopic(agent_id), 10);
        }
        PublisherPtr<PointStampedMsg> createAgentPositionSetpointPublisher(const ID& agent_id)
        {
            return node_->create_publisher<PointStampedMsg>(getAgentPositionSetpointTopic(agent_id), 10);
        }
        PublisherPtr<AgentTrackingSetpointsMsg> createAgentTrackingSetpointsPublisher(const ID& agent_id)
        {
            return node_->create_publisher<AgentTrackingSetpointsMsg>(getAgentTrackingSetpointsTopic(agent_id), 10);
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
        PublisherPtr<PointStampedMsg> createTargetTruePositionPublisher(const ID& target_id)
        {
            return node_->create_publisher<PointStampedMsg>(getTargetTruePositionTopic(target_id), 10);
        }
        PublisherPtr<PointStampedMsg> createTargetEstPositionPublisher(const ID& target_id)
        {
            return node_->create_publisher<PointStampedMsg>(getTargetEstPositionTopic(target_id), 10);
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
        PublisherPtr<ClusterAssignmentMsg> createClusterAssignmentPublisher(const ID& cluster_id)
        {
            rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
            return node_->create_publisher<ClusterAssignmentMsg>(getClusterAssignmentTopic(cluster_id), qos);
        }
        PublisherPtr<ClusterGeometryMsg> createClusterGeometryPublisher(const ID& cluster_id)
        {
            return node_->create_publisher<ClusterGeometryMsg>(getClusterGeometryTopic(cluster_id), 10);
        }
        PublisherPtr<ClusterMetricsMsg> createClusterMetricsPublisher(const ID& cluster_id)
        {
            return node_->create_publisher<ClusterMetricsMsg>(getClusterMetricsTopic(cluster_id), 10);
        }
        PublisherPtr<MarkerArrayMsg> createClusterMarkersPublisher(const ID& cluster_id)
        {
            return node_->create_publisher<MarkerArrayMsg>(getClusterMarkersTopic(cluster_id), 10);
        }

        // GUI publishers
        PublisherPtr<GuiSetpointsMsg> createGuiSetpointsPublisher(const ID& agent_id)
        {
            return node_->create_publisher<GuiSetpointsMsg>(getGuiSetpointsTopic(agent_id), 10);
        }

        // Subscribers
        // Global subscribers
        SubscriberPtr<RegistrationMsg> createRegistrationSubscriber(const std::function<void(const RegistrationMsg::SharedPtr)>& callback, const rclcpp::SubscriptionOptions& options = rclcpp::SubscriptionOptions())
        {
            rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
            return node_->create_subscription<RegistrationMsg>(getRegistrationTopic(), qos, callback, options);
        }
        SubscriberPtr<GlobalMetricsMsg> createGlobalMetricsSubscriber(const std::function<void(const GlobalMetricsMsg::SharedPtr)>& callback, const rclcpp::SubscriptionOptions& options = rclcpp::SubscriptionOptions())
        {
            return node_->create_subscription<GlobalMetricsMsg>(getGlobalMetricsTopic(), 10, callback, options);
        }

        // Agent subscribers
        SubscriberPtr<AgentStatusMsg> createAgentStatusSubscriber(const ID& agent_id, const std::function<void(const AgentStatusMsg::SharedPtr)>& callback, const rclcpp::SubscriptionOptions& options = rclcpp::SubscriptionOptions())
        {
            rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
            return node_->create_subscription<AgentStatusMsg>(getAgentStatusTopic(agent_id), qos, callback, options);
        }
        SubscriberPtr<PointStampedMsg> createAgentPositionSubscriber(const ID& agent_id, const std::function<void(const PointStampedMsg::SharedPtr)>& callback, const rclcpp::SubscriptionOptions& options = rclcpp::SubscriptionOptions())
        {
            return node_->create_subscription<PointStampedMsg>(getAgentPositionTopic(agent_id), 10, callback, options);
        }
        SubscriberPtr<AgentAssignmentMsg> createAgentAssignmentSubscriber(const ID& agent_id, const std::function<void(const AgentAssignmentMsg::SharedPtr)>& callback, const rclcpp::SubscriptionOptions& options = rclcpp::SubscriptionOptions())
        {
            rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
            return node_->create_subscription<AgentAssignmentMsg>(getAgentAssignmentTopic(agent_id), qos, callback, options);
        }
        SubscriberPtr<AgentClustersMsg> createAgentClustersSubscriber(const ID& agent_id, const std::function<void(const AgentClustersMsg::SharedPtr)>& callback, const rclcpp::SubscriptionOptions& options = rclcpp::SubscriptionOptions())
        {
            return node_->create_subscription<AgentClustersMsg>(getAgentClustersTopic(agent_id), 10, callback, options);
        }
        SubscriberPtr<PointStampedMsg> createAgentPositionSetpointSubscriber(const ID& agent_id, const std::function<void(const PointStampedMsg::SharedPtr)>& callback, const rclcpp::SubscriptionOptions& options = rclcpp::SubscriptionOptions())
        {
            return node_->create_subscription<PointStampedMsg>(getAgentPositionSetpointTopic(agent_id), 10, callback, options);
        }
        SubscriberPtr<AgentTrackingSetpointsMsg> createAgentTrackingSetpointsSubscriber(const ID& agent_id, const std::function<void(const AgentTrackingSetpointsMsg::SharedPtr)>& callback, const rclcpp::SubscriptionOptions& options = rclcpp::SubscriptionOptions())
        {
            return node_->create_subscription<AgentTrackingSetpointsMsg>(getAgentTrackingSetpointsTopic(agent_id), 10, callback, options);
        }
        SubscriberPtr<AgentMetricsMsg> createAgentMetricsSubscriber(const ID& agent_id, const std::function<void(const AgentMetricsMsg::SharedPtr)>& callback, const rclcpp::SubscriptionOptions& options = rclcpp::SubscriptionOptions())
        {
            return node_->create_subscription<AgentMetricsMsg>(getAgentMetricsTopic(agent_id), 10, callback, options);
        }
        SubscriberPtr<MarkerArrayMsg> createAgentMarkersSubscriber(const ID& agent_id, const std::function<void(const MarkerArrayMsg::SharedPtr)>& callback, const rclcpp::SubscriptionOptions& options = rclcpp::SubscriptionOptions())
        {
            return node_->create_subscription<MarkerArrayMsg>(getAgentMarkersTopic(agent_id), 10, callback, options);
        }

        // Target subscribers
        SubscriberPtr<PointStampedMsg> createTargetTruePositionSubscriber(const ID& target_id, const std::function<void(const PointStampedMsg::SharedPtr)>& callback, const rclcpp::SubscriptionOptions& options = rclcpp::SubscriptionOptions())
        {
            return node_->create_subscription<PointStampedMsg>(getTargetTruePositionTopic(target_id), 10, callback, options);
        }
        SubscriberPtr<PointStampedMsg> createTargetEstPositionSubscriber(const ID& target_id, const std::function<void(const PointStampedMsg::SharedPtr)>& callback, const rclcpp::SubscriptionOptions& options = rclcpp::SubscriptionOptions())
        {
            return node_->create_subscription<PointStampedMsg>(getTargetEstPositionTopic(target_id), 10, callback, options);
        }
        SubscriberPtr<TargetMetricsMsg> createTargetMetricsSubscriber(const ID& target_id, const std::function<void(const TargetMetricsMsg::SharedPtr)>& callback, const rclcpp::SubscriptionOptions& options = rclcpp::SubscriptionOptions())
        {
            return node_->create_subscription<TargetMetricsMsg>(getTargetMetricsTopic(target_id), 10, callback, options);
        }
        SubscriberPtr<MarkerArrayMsg> createTargetMarkersSubscriber(const ID& target_id, const std::function<void(const MarkerArrayMsg::SharedPtr)>& callback, const rclcpp::SubscriptionOptions& options = rclcpp::SubscriptionOptions())
        {
            return node_->create_subscription<MarkerArrayMsg>(getTargetMarkersTopic(target_id), 10, callback, options);
        }

        // Cluster subscribers
        SubscriberPtr<ClusterAssignmentMsg> createClusterAssignmentSubscriber(const ID& cluster_id, const std::function<void(const ClusterAssignmentMsg::SharedPtr)>& callback, const rclcpp::SubscriptionOptions& options = rclcpp::SubscriptionOptions())
        {
            rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
            return node_->create_subscription<ClusterAssignmentMsg>(getClusterAssignmentTopic(cluster_id), qos, callback, options);
        }
        SubscriberPtr<ClusterGeometryMsg> createClusterGeometrySubscriber(const ID& cluster_id, const std::function<void(const ClusterGeometryMsg::SharedPtr)>& callback, const rclcpp::SubscriptionOptions& options = rclcpp::SubscriptionOptions())
        {
            return node_->create_subscription<ClusterGeometryMsg>(getClusterGeometryTopic(cluster_id), 10, callback, options);
        }
        SubscriberPtr<ClusterMetricsMsg> createClusterMetricsSubscriber(const ID& cluster_id, const std::function<void(const ClusterMetricsMsg::SharedPtr)>& callback, const rclcpp::SubscriptionOptions& options = rclcpp::SubscriptionOptions())
        {
            return node_->create_subscription<ClusterMetricsMsg>(getClusterMetricsTopic(cluster_id), 10, callback, options);
        }
        SubscriberPtr<MarkerArrayMsg> createClusterMarkersSubscriber(const ID& cluster_id, const std::function<void(const MarkerArrayMsg::SharedPtr)>& callback, const rclcpp::SubscriptionOptions& options = rclcpp::SubscriptionOptions())
        {
            return node_->create_subscription<MarkerArrayMsg>(getClusterMarkersTopic(cluster_id), 10, callback, options);
        }

        // GUI subscribers
        SubscriberPtr<GuiSetpointsMsg> createGuiSetpointsSubscriber(const ID& agent_id, const std::function<void(const GuiSetpointsMsg::SharedPtr)>& callback, const rclcpp::SubscriptionOptions& options = rclcpp::SubscriptionOptions())
        {
            return node_->create_subscription<GuiSetpointsMsg>(getGuiSetpointsTopic(agent_id), 10, callback, options);
        }
    };

} // namespace flychams::core 