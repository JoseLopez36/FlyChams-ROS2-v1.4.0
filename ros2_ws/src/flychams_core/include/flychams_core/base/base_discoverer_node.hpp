#pragma once

// Tools includes
#include "flychams_core/config/config_tools.hpp"
#include "flychams_core/framework/framework_tools.hpp"
#include "flychams_core/ros/topic_tools.hpp"
#include "flychams_core/ros/transform_tools.hpp"

namespace flychams::core
{
    /**
     * ════════════════════════════════════════════════════════════════
     * @brief Discoverer node for discovering the different elements
     * in the simulation
     *
     * @details
     * This class implements the discoverer node for discovering agents,
     * targets, and clusters with the help of the various tools. It serves
     * as a base class for the different nodes that need to discover elements
     * dynamically in the simulation.
     *
     * ════════════════════════════════════════════════════════════════
     * @author Jose Francisco Lopez Ruiz
     * @date 2025-02-28
     * ════════════════════════════════════════════════════════════════
     */
    class BaseDiscovererNode : public rclcpp::Node
    {
    public: // Constructor/Destructor
        BaseDiscovererNode(const std::string& node_name, const rclcpp::NodeOptions& options)
            : Node(node_name, options), node_name_(node_name)
        {
            // Nothing to do
        }
        void init()
        {
            // Get node pointer
            node_ = this->shared_from_this();
            RCLCPP_INFO(node_->get_logger(), "Starting %s node...", node_name_.c_str());

            // Create tools
            config_tools_ = std::make_shared<ConfigTools>(node_);
            framework_tools_ = createFrameworkTools(node_, config_tools_);
            topic_tools_ = std::make_shared<TopicTools>(node_, config_tools_);
            transform_tools_ = std::make_shared<TransformTools>(node_, config_tools_);

            // Create callback group
            discovery_cb_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
            sub_options_with_discovery_cb_group_.callback_group = discovery_cb_group_;

            // Initialize discovery subscriber
            elements_.clear();
            discovery_sub_ = topic_tools_->createRegistrationSubscriber(
                std::bind(&BaseDiscovererNode::onDiscovery, this, std::placeholders::_1),
                sub_options_with_discovery_cb_group_);

            // Call on init overridable method
            onInit();
            RCLCPP_INFO(node_->get_logger(), "%s node running", node_name_.c_str());
        }

        virtual ~BaseDiscovererNode()
        {
            shutdown();
        }

        void shutdown()
        {
            RCLCPP_INFO(node_->get_logger(), "Shutting down %s node...", node_name_.c_str());
            // Call on shutdown overridable method
            onShutdown();
            // Destroy discovery subscriber
            elements_.clear();
            discovery_sub_.reset();
            // Destroy tools
            config_tools_.reset();
            framework_tools_.reset();
            topic_tools_.reset();
            transform_tools_.reset();
        }

    public: // Types
        using SharedPtr = std::shared_ptr<BaseDiscovererNode>;

    protected: // Overridable methods
        virtual void onInit() = 0;
        virtual void onShutdown() = 0;
        virtual void onAddAgent(const ID& agent_id) {}
        virtual void onRemoveAgent(const ID& agent_id) {}
        virtual void onAddTarget(const ID& target_id) {}
        virtual void onRemoveTarget(const ID& target_id) {}
        virtual void onAddCluster(const ID& cluster_id) {}
        virtual void onRemoveCluster(const ID& cluster_id) {}

    private: // Discovery callback
        void onDiscovery(const RegistrationMsg::SharedPtr msg)
        {
            // Track elements in this message
            std::unordered_set<ID> current_elements;
            for (const auto& element : msg->elements)
            {
                // Get element id and type
                const auto element_id = element.id;
                const auto element_type = static_cast<ElementType>(element.type);

                // Add element ID to current set
                current_elements.insert(element_id);

                // Add element if it doesn't exist already
                if (elements_.find(element_id) == elements_.end())
                {
                    elements_.insert({ element_id, element_type });

                    // Call corresponding add callback
                    switch (element_type)
                    {
                    case ElementType::Agent:
                        // Add vehicle to FrameworkTools
                        framework_tools_->addVehicle(element_id);
                        // Call onAddAgent callback
                        onAddAgent(element_id);
                        break;

                    case ElementType::Target:
                        // Call onAddTarget callback
                        onAddTarget(element_id);
                        break;

                    case ElementType::Cluster:
                        // Call onAddCluster callback
                        onAddCluster(element_id);
                        break;
                    }
                }
            }

            // Remove elements that are no longer present
            std::vector<ID> to_remove;
            for (const auto& [element_id, element_type] : elements_)
            {
                if (current_elements.find(element_id) == current_elements.end())
                {
                    to_remove.push_back(element_id);
                }
            }

            // Remove elements that are not in the current message
            for (const auto& element_id : to_remove)
            {
                const auto element_type = elements_[element_id];
                elements_.erase(element_id);

                // Call corresponding remove callback
                switch (element_type)
                {
                case ElementType::Agent:
                    // Remove vehicle from FrameworkTools
                    framework_tools_->removeVehicle(element_id);
                    // Call onRemoveAgent callback
                    onRemoveAgent(element_id);
                    break;

                case ElementType::Target:
                    // Call onRemoveTarget callback
                    onRemoveTarget(element_id);
                    break;

                case ElementType::Cluster:
                    // Call onRemoveCluster callback
                    onRemoveCluster(element_id);
                    break;
                }
            }
        }

    protected: // Components
        // Node
        NodePtr node_;
        const std::string node_name_;
        // Tools
        ConfigTools::SharedPtr config_tools_;
        FrameworkTools::SharedPtr framework_tools_;
        TopicTools::SharedPtr topic_tools_;
        TransformTools::SharedPtr transform_tools_;
        // Discovered elements
        std::unordered_map<ID, ElementType> elements_;
        // Callback group
        CallbackGroupPtr discovery_cb_group_;
        rclcpp::SubscriptionOptions sub_options_with_discovery_cb_group_;
        // Discovery subscriber
        SubscriberPtr<RegistrationMsg> discovery_sub_;
    };

} // namespace flychams::core