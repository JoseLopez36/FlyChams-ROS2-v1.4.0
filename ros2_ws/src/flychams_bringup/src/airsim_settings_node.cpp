#include "rclcpp/rclcpp.hpp"

// Core includes
#include "flychams_core/base/registrator_node.hpp"

using namespace flychams::core;

/**
 * ════════════════════════════════════════════════════════════════
 * @brief Airsim settings node for creating the settings.json file
 *
 * @details
 * This class implements the airsim settings node for creating the
 * settings.json file.
 *
 * ════════════════════════════════════════════════════════════════
 * @author Jose Francisco Lopez Ruiz
 * @date 2025-03-02
 * ════════════════════════════════════════════════════════════════
 */
class AirsimSettingsNode : public rclcpp::Node
{
public: // Constructor/Destructor
    AirsimSettingsNode(const std::string& node_name, const rclcpp::NodeOptions& options)
        : rclcpp::Node(node_name, options)
    {
        // Nothing to do
    }

    void init()
    {
        // Get node pointer
        node_ = this->shared_from_this();
        RCLCPP_INFO(node_->get_logger(), "Starting %s node...", node_name_.c_str());

        // Create config tools
        config_tools_ = std::make_shared<ConfigTools>(node_);

        // Create settings.json file
        config_tools_->createAirsimSettings();
    }

    ~AirsimSettingsNode()
    {
        shutdown();
    }

    void shutdown()
    {
        RCLCPP_INFO(node_->get_logger(), "Shutting down %s node...", node_name_.c_str());
        // Destroy config tools
        config_tools_.reset();
    }

private: // Data
    // Node
    NodePtr node_;
    const std::string node_name_;
    // Tools
    ConfigTools::SharedPtr config_tools_;
};

int main(int argc, char** argv)
{
    // Initialize ROS
    rclcpp::init(argc, argv);
    // Create node options
    rclcpp::NodeOptions options;
    options.allow_undeclared_parameters(true);
    options.automatically_declare_parameters_from_overrides(true);
    // Create and initialize node
    auto node = std::make_shared<AirsimSettingsNode>("airsim_settings_node", options);
    node->init();
    // Spin node
    rclcpp::spin(node);
    // Shutdown
    rclcpp::shutdown();
    return 0;
}