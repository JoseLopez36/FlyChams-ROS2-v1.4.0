#include "flychams_core/base/base_discoverer_node.hpp"

namespace flychams::core
{
    // ════════════════════════════════════════════════════════════════════════════
    // CONSTRUCTOR: Constructor and destructor
    // ════════════════════════════════════════════════════════════════════════════

    BaseDiscovererNode::BaseDiscovererNode(const std::string& node_name, const rclcpp::NodeOptions& options)
        : Node(node_name, options), node_name_(node_name)
    {
        // Nothing to do
    }

    void BaseDiscovererNode::init()
    {
        // Get node pointer
        node_ = this->shared_from_this();
        RCLCPP_INFO(node_->get_logger(), "Starting %s node...", node_name_.c_str());

        // Create tools
        config_tools_ = std::make_shared<ConfigTools>(node_);
        ext_tools_ = externalToolsFactory(node_, Framework::AirSim);
        topic_tools_ = std::make_shared<TopicTools>(node_);
        tf_tools_ = std::make_shared<TfTools>(node_);

        // Initialize discovery subscriber
        elements_.clear();
        discovery_sub_ = topic_tools_->createRegistrationSubscriber(
            std::bind(&BaseDiscovererNode::onDiscovery, this, std::placeholders::_1));

        // Call on init overridable method
        onInit();
        RCLCPP_INFO(node_->get_logger(), "%s node running", node_name_.c_str());
    }

    BaseDiscovererNode::~BaseDiscovererNode()
    {
        shutdown();
    }

    void BaseDiscovererNode::shutdown()
    {
        // Lock elements map
        std::lock_guard<std::mutex> lock(elements_mutex_);

        RCLCPP_INFO(node_->get_logger(), "Shutting down %s node...", node_name_.c_str());
        // Call on shutdown overridable method
        onShutdown();
        // Destroy discovery subscriber
        elements_.clear();
        discovery_sub_.reset();
        // Destroy tools
        config_tools_.reset();
        ext_tools_.reset();
        topic_tools_.reset();
        tf_tools_.reset();
    }

    // ════════════════════════════════════════════════════════════════════════════
    // CALLBACKS: Callbacks for adding or removing elements
    // ════════════════════════════════════════════════════════════════════════════

    void BaseDiscovererNode::onDiscovery(const RegistrationMsg::SharedPtr msg)
    {
        // Lock elements map
        std::lock_guard<std::mutex> lock(elements_mutex_);

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
                    // Add vehicle to ExternalTools
                    ext_tools_->addVehicle(element_id);
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
                // Remove vehicle from ExternalTools
                ext_tools_->removeVehicle(element_id);
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

} // namespace flychams::core