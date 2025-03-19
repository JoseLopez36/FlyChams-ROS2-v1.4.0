#include "rclcpp/rclcpp.hpp"

// Standard includes
#include <iostream>
#include <cstdlib>

// Core includes
#include "flychams_core/base/base_registrator_node.hpp"

using namespace flychams::core;

/**
 * ════════════════════════════════════════════════════════════════
 * @brief Registrator node for registering the different elements
 * in the simulation
 *
 * @details
 * This class implements the registrator node for registering the
 * different elements in the simulation. It bases on the registrator node
 * to register the different elements, so that they can be discovered by
 * the different nodes.
 *
 * ════════════════════════════════════════════════════════════════
 * @author Jose Francisco Lopez Ruiz
 * @date 2025-02-28
 * ════════════════════════════════════════════════════════════════
 */
class RegistratorNode : public BaseRegistratorNode
{
public: // Constructor/Destructor
    RegistratorNode(const std::string& node_name, const rclcpp::NodeOptions& options)
        : BaseRegistratorNode(node_name, options)
    {
        // Nothing to do
    }

    void onInit() override
    {
        // Register agents
        std::string first_agent_id = "";
        for (const auto& [agent_id, agent_config] : config_tools_->getAgents())
        {
            registerElement(agent_id, ElementType::Agent);

            // Get first agent ID
            if (first_agent_id == "")
                first_agent_id = agent_id;
        }

        // Register targets
        for (const auto& [target_id, target_config] : config_tools_->getTargets())
        {
            registerElement(target_id, ElementType::Target);
        }

        // Register clusters
        for (const auto& [agent_id, agent_config] : config_tools_->getAgents())
        {
            // Extract maximum number of assignments per agent
            const int max_assign = agent_config->max_assignments;

            // Cycle through all assignments and register clusters
            for (int i = 0; i < max_assign; i++)
            {
                // Generate cluster ID
                std::stringstream ss;
                ss << "CLUSTER" << std::setw(2) << std::setfill('0') << i;
                const ID cluster_id = ss.str();

                // Register cluster
                registerElement(cluster_id, ElementType::Cluster);
            }
        }

        // Wait for 2 seconds
        std::this_thread::sleep_for(std::chrono::seconds(2));

        // Check if there are any agents available
        if (config_tools_->getAgents().empty()) {
            RCLCPP_ERROR(node_->get_logger(), "No agents found. Cannot initialize GUI.");
            return; // Exit early if no agents are available
        }

        // Get window IDs
        const ID scene_window_id = RosUtils::getParameter<ID>(node_, "window_ids.scene_id");
        const ID agent_1_window_id = RosUtils::getParameter<ID>(node_, "window_ids.agent_1_id");
        const ID agent_2_window_id = RosUtils::getParameter<ID>(node_, "window_ids.agent_2_id");
        const ID map_window_id = RosUtils::getParameter<ID>(node_, "window_ids.map_id");
        const ID central_window_id = RosUtils::getParameter<ID>(node_, "window_ids.central_id");
        const IDs tracking_window_ids = RosUtils::getParameter<IDs>(node_, "window_ids.tracking_ids");

        // Check if tracking window IDs are valid
        if (tracking_window_ids.size() < 4) {
            RCLCPP_WARN(node_->get_logger(), "Not enough tracking window IDs provided. Defaulting to empty.");
        }

        // Get camera IDs
        const ID scene_camera_id = RosUtils::getParameter<ID>(node_, "camera_ids.scene_id");
        const ID agent_1_camera_id = RosUtils::replacePlaceholder(RosUtils::getParameter<ID>(node_, "camera_ids.agent_1_id"), "AGENTID", first_agent_id);
        const ID agent_2_camera_id = RosUtils::replacePlaceholder(RosUtils::getParameter<ID>(node_, "camera_ids.agent_2_id"), "AGENTID", first_agent_id);
        const ID central_camera_id = config_tools_->getAgent(first_agent_id)->central_head_id;

        // Initialize GUI
        IDs window_ids(9, "");          // Initialize with size 9
        IDs vehicle_ids(9, "");         // Initialize with size 9
        IDs camera_ids(9, "");          // Initialize with size 9
        std::vector<int> crop_x(9, 0);  // Initialize with size 9 and default value 0
        std::vector<int> crop_y(9, 0);  // Initialize with size 9 and default value 0
        std::vector<int> crop_w(9, 0);  // Initialize with size 9 and default value 0
        std::vector<int> crop_h(9, 0);  // Initialize with size 9 and default value 0

        // Set window depending on index
        window_ids[0] = scene_window_id;
        vehicle_ids[0] = "";
        camera_ids[0] = scene_camera_id;

        window_ids[1] = agent_1_window_id;
        vehicle_ids[1] = first_agent_id;
        camera_ids[1] = agent_1_camera_id;

        window_ids[2] = agent_2_window_id;
        vehicle_ids[2] = first_agent_id;
        camera_ids[2] = agent_2_camera_id;

        window_ids[3] = map_window_id;
        vehicle_ids[3] = "";
        camera_ids[3] = "";

        window_ids[4] = central_window_id;
        vehicle_ids[4] = first_agent_id;
        camera_ids[4] = central_camera_id;

        for (size_t i = 5; i < 9; i++)
        {
            if (i - 5 < tracking_window_ids.size())
                window_ids[i] = tracking_window_ids[i - 5];
            else
                window_ids[i] = "";
        }

        ext_tools_->setWindowImageGroup(window_ids, vehicle_ids, camera_ids, crop_x, crop_y, crop_w, crop_h);
    }

    void onShutdown() override
    {
        // Nothing to do
    }
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
    auto node = std::make_shared<RegistratorNode>("registrator_node", options);
    node->init();
    // Spin node
    rclcpp::spin(node);
    // Shutdown
    rclcpp::shutdown();
    return 0;
}