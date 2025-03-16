#include "rclcpp/rclcpp.hpp"

// Perception includes
#include "flychams_perception/clustering/target_clustering.hpp"

// Core includes
#include "flychams_core/base/base_discoverer_node.hpp"

using namespace flychams::core;
using namespace flychams::perception;

/**
 * ════════════════════════════════════════════════════════════════
 * @brief Clustering node for clustering the different targets
 * in the simulation
 *
 * @details
 * This class implements the clustering node for clustering the
 * different targets in the simulation. It uses the discoverer node to
 * discover the different targets and then creates a clustering for each
 * target discovered.
 *
 * ════════════════════════════════════════════════════════════════
 * @author Jose Francisco Lopez Ruiz
 * @date 2025-03-01
 * ════════════════════════════════════════════════════════════════
 */
class ClusteringNode : public BaseDiscovererNode
{
public: // Constructor/Destructor
    ClusteringNode(const std::string& node_name, const rclcpp::NodeOptions& options)
        : BaseDiscovererNode(node_name, options)
    {
        // Nothing to do
    }

    void onInit() override
    {
        // Initialize target clustering
        target_clustering_ = std::make_shared<TargetClustering>(node_, config_tools_, ext_tools_, topic_tools_, tf_tools_);

        // Remove all clusters from simulation
        ext_tools_->removeAllClusters();

        // Wait 2 seconds to ensure clusters are removed
        std::this_thread::sleep_for(std::chrono::seconds(2));
    }

    void onShutdown() override
    {
        // Destroy target clustering system
        target_clustering_.reset();
    }

private: // Element management
    void onAddTarget(const ID& target_id) override
    {
        // Add target to clustering
        target_clustering_->addTarget(target_id);
    }

    void onRemoveTarget(const ID& target_id) override
    {
        // Remove target from clustering
        target_clustering_->removeTarget(target_id);
    }

    void onAddCluster(const ID& cluster_id) override
    {
        // Add cluster to clustering
        target_clustering_->addCluster(cluster_id);
    }

    void onRemoveCluster(const ID& cluster_id) override
    {
        // Remove cluster from clustering
        target_clustering_->removeCluster(cluster_id);
    }

private: // Components
    // Target clustering system
    TargetClustering::SharedPtr target_clustering_;
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
    auto node = std::make_shared<ClusteringNode>("clustering_node", options);
    node->init();
    // Create executor and add node
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    // Spin node
    executor.spin();
    // Shutdown
    rclcpp::shutdown();
    return 0;
}