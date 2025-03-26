#pragma once

// Standard includes
#include <mutex>
#include <unordered_map>

// Tools includes
#include "flychams_core/config/config_tools.hpp"
#include "flychams_core/framework/framework_tools.hpp"
#include "flychams_core/ros/topic_tools.hpp"
#include "flychams_core/ros/transform_tools.hpp"

namespace flychams::core
{
    /**
     * ════════════════════════════════════════════════════════════════
     * @brief Registrator node for registering the different elements
     * in the simulation
     *
     * @details
     * This class implements the registrator node for registering agents,
     * targets, and clusters with the help of the various tools. It serves
     * as a base class for the different nodes that need to register elements
     * dynamically in the simulation.
     *
     * ════════════════════════════════════════════════════════════════
     * @author Jose Francisco Lopez Ruiz
     * @date 2025-02-28
     * ════════════════════════════════════════════════════════════════
     */
    class BaseRegistratorNode : public rclcpp::Node
    {
    public: // Constructor/Destructor
        BaseRegistratorNode(const std::string& node_name, const rclcpp::NodeOptions& options)
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

            // Initialize registration publisher
            elements_.clear();
            registration_pub_ = topic_tools_->createRegistrationPublisher();

            // Initialize update timer (1 Hz)
            update_timer_ = RosUtils::createWallTimer(node_, 1.0f, [this]() { publishRegistration(); });

            // Call on init overridable method
            onInit();
            RCLCPP_INFO(node_->get_logger(), "%s node running", node_name_.c_str());
        }

        virtual ~BaseRegistratorNode()
        {
            shutdown();
        }

        void shutdown()
        {
            // Lock elements map
            std::lock_guard<std::mutex> lock(mutex_);

            RCLCPP_INFO(node_->get_logger(), "Shutting down %s node...", node_name_.c_str());
            // Call on shutdown overridable method
            onShutdown();
            // Destroy update timer
            update_timer_.reset();
            // Destroy registration publisher
            elements_.clear();
            registration_pub_.reset();
            // Destroy tools
            config_tools_.reset();
            framework_tools_.reset();
            topic_tools_.reset();
            transform_tools_.reset();
        }

    public: // Types
        using SharedPtr = std::shared_ptr<BaseRegistratorNode>;

    protected: // Overridable methods
        virtual void onInit() = 0;
        virtual void onShutdown() = 0;

    protected: // Registration methods
        // Element registration
        void registerElement(const ID& element_id, const ElementType& element_type)
        {
            // Lock elements map
            std::lock_guard<std::mutex> lock(mutex_);

            // Add element to map (only if not already registered)
            if (elements_.find(element_id) != elements_.end())
                return;
            elements_.insert({ element_id, element_type });

            // Register element in tools
            switch (element_type)
            {
            case ElementType::Agent:
                framework_tools_->addVehicle(element_id);
                break;
            }

            RCLCPP_INFO(node_->get_logger(), "Element %s registered", element_id.c_str());
        }

        void unregisterElement(const ID& element_id, const ElementType& element_type)
        {
            // Lock elements map
            std::lock_guard<std::mutex> lock(mutex_);

            // Remove element from map (only if registered)
            if (elements_.find(element_id) == elements_.end())
                return;
            elements_.erase(element_id);

            // Unregister element in tools
            switch (element_type)
            {
            case ElementType::Agent:
                framework_tools_->removeVehicle(element_id);
                break;
            }

            RCLCPP_INFO(node_->get_logger(), "Element %s unregistered", element_id.c_str());
        }

    private: // Update
        void publishRegistration()
        {
            // Lock elements map
            std::lock_guard<std::mutex> lock(mutex_);

            // Create and publish registration message
            RegistrationMsg msg;
            for (const auto& [id, type] : elements_)
            {
                ElementMsg element;
                element.id = id;
                element.type = static_cast<uint8_t>(type);
                msg.elements.push_back(element);
            }
            registration_pub_->publish(msg);
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
        // Registered elements
        std::unordered_map<ID, ElementType> elements_;
        std::mutex mutex_;
        // Registration publisher
        PublisherPtr<RegistrationMsg> registration_pub_;
        // Update timer
        TimerPtr update_timer_;
    };

} // namespace flychams::core