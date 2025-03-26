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
     * @brief Base module for all sub-nodes in the system
     *
     * @details
     * This class is the base class for all sub-nodes present in the
     * system. It provides a common interface for all sub-nodes and a
     * set of utilities for the modules to use.
     *
     * ════════════════════════════════════════════════════════════════
     * @author Jose Francisco Lopez Ruiz
     * @date 2025-02-28
     * ════════════════════════════════════════════════════════════════
     */
    class BaseModule
    {
    public: // Constructor/Destructor
        BaseModule(NodePtr node, ConfigTools::SharedPtr config_tools, FrameworkTools::SharedPtr framework_tools, TopicTools::SharedPtr topic_tools, TransformTools::SharedPtr transform_tools)
            : node_(node), config_tools_(config_tools), framework_tools_(framework_tools), topic_tools_(topic_tools), transform_tools_(transform_tools)
        {
            // Nothing to do
        }
        void init()
        {
            // Call on init overridable method
            onInit();
        }
        virtual ~BaseModule()
        {
            shutdown();
        }
        void shutdown()
        {
            // Call on shutdown overridable method
            onShutdown();
            // Destroy tools
            config_tools_.reset();
            framework_tools_.reset();
            topic_tools_.reset();
            transform_tools_.reset();
            // Destroy node
            node_.reset();
        }

    public: // Types
        using SharedPtr = std::shared_ptr<BaseModule>;

    protected: // Overridable methods
        virtual void onInit() = 0;
        virtual void onShutdown() = 0;

    protected: // Components
        // Node
        NodePtr node_;
        // Tools
        ConfigTools::SharedPtr config_tools_;
        FrameworkTools::SharedPtr framework_tools_;
        TopicTools::SharedPtr topic_tools_;
        TransformTools::SharedPtr transform_tools_;
    };

} // namespace flychams::core