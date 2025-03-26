#include "flychams_core/framework/framework_tools.hpp"

// Derived classes includes
#include "flychams_core/framework/airsim_tools.hpp"

namespace flychams::core
{
    FrameworkTools::SharedPtr createFrameworkTools(NodePtr node, const ConfigTools::SharedPtr& config_tools)
    {
        // Get framework
        const Framework framework = config_tools->getSystem().framework;

        // Create framework tools based on framework
        switch (framework)
        {
        case Framework::AirSim:
            return std::make_shared<AirsimTools>(node, config_tools);
        default:
            throw std::runtime_error("Unknown framework: " + std::to_string(static_cast<int>(framework)));
        }
    }

} // namespace flychams::core