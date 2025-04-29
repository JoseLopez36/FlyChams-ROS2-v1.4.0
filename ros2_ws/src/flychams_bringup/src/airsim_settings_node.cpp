#include "rclcpp/rclcpp.hpp"

// Config tools includes
#include "flychams_core/config/config_tools.hpp"

using namespace flychams::core;

/**
 * ════════════════════════════════════════════════════════════════
 * @brief Airsim settings node for creating the settings.json file
 * ════════════════════════════════════════════════════════════════
 * @author Jose Francisco Lopez Ruiz
 * @date 2025-03-02
 * ════════════════════════════════════════════════════════════════
 */

int main(int argc, char** argv)
{
    // Initialize ROS
    rclcpp::init(argc, argv);
    // Create node options
    rclcpp::NodeOptions options;
    options.allow_undeclared_parameters(true);
    options.automatically_declare_parameters_from_overrides(true);
    // Create temporary node
    auto node = rclcpp::Node::make_shared("airsim_settings_creator", options);

    // Create config tools
    auto config_tools = std::make_shared<ConfigTools>(node);

    // Create the settings.json file
    config_tools->createAirsimSettings();

    // Finish the node
    RCLCPP_INFO(node->get_logger(), "Closing airsim_settings_node");
    rclcpp::shutdown();
    return 0;
}