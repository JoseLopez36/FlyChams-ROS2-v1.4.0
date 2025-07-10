#pragma once

// Base module include
#include "flychams_core/base/base_module.hpp"

namespace flychams::simulation
{
    /**
     * ════════════════════════════════════════════════════════════════
     * @brief Controller for targets and clusters in the simulation
     *
     * @details
     * This class is responsible for managing the control of targets and
     * clusters in the simulation. It handles the control of the target
     * and its clusters, mainly their instantaneus position.
     *
     * ════════════════════════════════════════════════════════════════
     * @author Jose Francisco Lopez Ruiz
     * @date 2025-03-27
     * ════════════════════════════════════════════════════════════════
     */
    class TargetControl : public core::BaseModule
    {
    public: // Constructor/Destructor
        TargetControl(core::NodePtr node, core::ConfigTools::SharedPtr config_tools, core::FrameworkTools::SharedPtr framework_tools, core::TopicTools::SharedPtr topic_tools, core::TransformTools::SharedPtr transform_tools, core::CallbackGroupPtr module_cb_group)
            : BaseModule(node, config_tools, framework_tools, topic_tools, transform_tools, module_cb_group)
        {
            init();
        }

    protected: // Overrides
        void onInit() override;
        void onShutdown() override;

    public: // Types
        using SharedPtr = std::shared_ptr<TargetControl>;
        struct Target
        {
            // Position
            core::PointMsg position;
            bool has_position;
            // Subscriber
            core::SubscriberPtr<core::PointStampedMsg> position_sub;
            // Constructor
            Target()
                : position(), has_position(false), position_sub()
            {
            }
        };
        struct Cluster
        {
            // Geometry
            core::PointMsg position;
            float radius;
            bool has_geometry;
            // Subscriber
            core::SubscriberPtr<core::ClusterGeometryMsg> geometry_sub;
            // Constructor
            Cluster()
                : position(), radius(), has_geometry(false), geometry_sub()
            {
            }
        };

    private: // Parameters
        float update_rate_;

    private: // Data
        // Targets
        std::unordered_map<core::ID, Target> targets_;
        // Clusters
        std::unordered_map<core::ID, Cluster> clusters_;
        // Other
        int spawn_index_;

    public: // Public methods
        void addCluster(const core::ID& cluster_id);
        void addTarget(const core::ID& target_id);
        void removeCluster(const core::ID& cluster_id);
        void removeTarget(const core::ID& target_id);

    private: // Callbacks
        void targetPositionCallback(const core::ID& target_id, const core::PointStampedMsg::SharedPtr msg);
        void clusterGeometryCallback(const core::ID& cluster_id, const core::ClusterGeometryMsg::SharedPtr msg);

    private: // Control management
        void update();

    private: // Control methods
        void destroyTargets();
        void destroyClusters();
        void spawnTarget(const core::ID& target_id, const core::PointMsg& initial_position, const core::TargetType& target_type);
        void spawnCluster(const core::ID& cluster_id, const core::PointMsg& initial_center, const float& initial_radius);
        void updateTargets();
        void updateClusters();

    private:
        // Timer
        core::TimerPtr update_timer_;
    };

} // namespace flychams::simulation