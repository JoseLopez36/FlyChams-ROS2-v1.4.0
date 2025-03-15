#include "rclcpp/rclcpp.hpp"

// Standard includes
#include <iostream>
#include <cstdlib>

// Core includes
#include "flychams_core/base/registrator_node.hpp"

using namespace flychams::core;

/**
 * ════════════════════════════════════════════════════════════════
 * @brief Agent registration node for registering the different agents
 * in the simulation
 *
 * @details
 * This class implements the agent registration node for registering the
 * different agents in the simulation. It uses the registrator node to
 * register the different agents, so that they can be discovered by the
 * different nodes.
 *
 * ════════════════════════════════════════════════════════════════
 * @author Jose Francisco Lopez Ruiz
 * @date 2025-02-28
 * ════════════════════════════════════════════════════════════════
 */
class AgentRegistrationNode : public RegistratorNode
{
public: // Constructor/Destructor
    AgentRegistrationNode(const std::string& node_name, const rclcpp::NodeOptions& options)
        : RegistratorNode(node_name, options)
    {
        // Nothing to do
    }

    void onInit() override
    {
        // Cycle through all agents in the config and register them
        int instance = 0;
        for (const auto& [id, _] : config_tools_->getAgents())
        {
            // MAVROS parameters
            std::string control_ip = "172.17.0.2";
            int udp_port = 14030 + instance;
            int remote_port = 14280 + instance;

            // Create MAVROS instance unique to the agent
            // Format: ros2 launch flychams_bringup mavros.launch.py tgt_system:=1 fcu_url:=udp://:14030@172.17.0.2:14280 agent_id:=AGENT01
            std::string tgt_system_cmd = " tgt_system:=" + std::to_string(instance + 1);
            std::string fcu_url_cmd = " fcu_url:=udp://:" + std::to_string(udp_port) + "@" + control_ip + ":" + std::to_string(remote_port);
            std::string agent_id_cmd = " agent_id:=" + id;
            std::string cmd = "ros2 launch flychams_bringup mavros.launch.py" + tgt_system_cmd + fcu_url_cmd + agent_id_cmd;
            int ret = std::system(cmd.c_str());
            if (ret != 0) {
                RCLCPP_ERROR(this->get_logger(), "Failed to launch MAVROS instance for agent %s", id.c_str());
                rclcpp::shutdown();
                return;
            }

            // Register agent
            registerAgent(id);

            instance++;
        }
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
    auto node = std::make_shared<AgentRegistrationNode>("agent_registration_node", options);
    node->init();
    // Spin node
    rclcpp::spin(node);
    // Shutdown
    rclcpp::shutdown();
    return 0;
}