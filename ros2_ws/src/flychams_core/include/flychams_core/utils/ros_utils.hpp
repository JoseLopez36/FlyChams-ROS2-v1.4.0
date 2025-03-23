#pragma once

// Standard includes
#include <regex>
#include <unordered_set>
#include <set>

// Core includes
#include "flychams_core/types/core_types.hpp"
#include "flychams_core/types/ros_types.hpp"

namespace flychams::core
{
    /**
     * ════════════════════════════════════════════════════════════════
     * @brief ROS utilities
     *
     * @details
     * This class contains ROS utilities
     * ════════════════════════════════════════════════════════════════
     */
    class RosUtils
    {
    public:
        // ════════════════════════════════════════════════════════════════════════════
        // TIMER: Timer utilities
        // ════════════════════════════════════════════════════════════════════════════

        static Time getTimeNow(NodePtr node)
        {
            return node->get_clock()->now();
        }

        static TimerPtr createTimer(NodePtr node, const std::function<void()>& callback, const std::chrono::milliseconds& period, CallbackGroupPtr callback_group = nullptr)
        {
            if (callback_group == nullptr)
            {
                return node->create_timer(period, callback);
            }
            else
            {
                return node->create_timer(period, callback, callback_group);
            }
        }

        static TimerPtr createTimerByRate(NodePtr node, float rate, const std::function<void()>& callback, CallbackGroupPtr callback_group = nullptr)
        {
            if (callback_group == nullptr)
            {
                return node->create_timer(std::chrono::duration<float>(1.0f / rate), callback);
            }
            else
            {
                return node->create_timer(std::chrono::duration<float>(1.0f / rate), callback, callback_group);
            }
        }

        static TimerPtr createWallTimer(NodePtr node, const std::function<void()>& callback, const std::chrono::milliseconds& period, CallbackGroupPtr callback_group = nullptr)
        {
            if (callback_group == nullptr)
            {
                return node->create_wall_timer(period, callback);
            }
            else
            {
                return node->create_wall_timer(period, callback, callback_group);
            }
        }

        static TimerPtr createWallTimerByRate(NodePtr node, float rate, const std::function<void()>& callback, CallbackGroupPtr callback_group = nullptr)
        {
            if (callback_group == nullptr)
            {
                return node->create_wall_timer(std::chrono::duration<float>(1.0f / rate), callback);
            }
            else
            {
                return node->create_wall_timer(std::chrono::duration<float>(1.0f / rate), callback, callback_group);
            }
        }

        // ════════════════════════════════════════════════════════════════════════════
        // PARAMETER: Parameter utilities
        // ════════════════════════════════════════════════════════════════════════════

        template <typename T>
        static T getParameter(NodePtr node, const std::string& param_name)
        {
            T value;
            if (!node->get_parameter(param_name, value))
            {
                RCLCPP_ERROR(node->get_logger(), "Failed to get parameter '%s'. Shutting down node '%s'", param_name.c_str(), node->get_name());
                throw rclcpp::exceptions::ParameterNotDeclaredException(param_name);
            }
            return value;
        }

        template <typename T>
        static T getParameterOr(NodePtr node, const std::string& param_name, const T& default_value)
        {
            return node->get_parameter_or(param_name, default_value);
        }

        // ════════════════════════════════════════════════════════════════════════════
        // SERVICE: Service utilities
        // ════════════════════════════════════════════════════════════════════════════

        template<typename T>
        static bool sendRequestSync(NodePtr node, ClientPtr<T> client, typename T::Request::SharedPtr request, int wait_time_ms = 1000)
        {
            // First, wait for service to be available
            if (!client->wait_for_service(std::chrono::milliseconds(wait_time_ms)))
            {
                RCLCPP_ERROR(node->get_logger(), "Service %s wait timed out", client->get_service_name());
                return false;
            }

            // Send the request and wait for the response
            client->async_send_request(request);
            return true;
        }

        // ════════════════════════════════════════════════════════════════════════════
        // OTHER: Other utilities
        // ════════════════════════════════════════════════════════════════════════════

        static std::string replacePlaceholder(const std::string& topic_name, const std::string& placeholder, const std::string& value)
        {
            return std::regex_replace(topic_name, std::regex(placeholder), value);
        }

        static HeaderMsg createHeader(NodePtr node, const std::string& frame_id)
        {
            HeaderMsg header;
            header.frame_id = frame_id;
            header.stamp = getTimeNow(node);
            return header;
        }

        static bool addToSet(NodePtr node, std::unordered_set<ID>& set, const ID& id)
        {
            // Check if element already exists
            if (set.find(id) != set.end())
            {
                RCLCPP_INFO(node->get_logger(), "Element %s already exists. Skipping addition", id.c_str());
                return false;
            }
            // Insert element
            set.insert(id);
            return true;
        }

        static bool removeFromSet(NodePtr node, std::unordered_set<ID>& set, const ID& id)
        {
            // Check if element exists
            if (set.find(id) == set.end())
            {
                RCLCPP_INFO(node->get_logger(), "Element %s does not exist. Skipping removal", id.c_str());
                return false;
            }
            // Remove element
            set.erase(id);
            return true;
        }

        static bool addToSet(NodePtr node, std::set<ID>& set, const ID& id)
        {
            // Check if element already exists
            if (set.find(id) != set.end())
            {
                RCLCPP_INFO(node->get_logger(), "Element %s already exists. Skipping addition", id.c_str());
                return false;
            }
            // Insert element
            set.insert(id);
            return true;
        }

        static bool removeFromSet(NodePtr node, std::set<ID>& set, const ID& id)
        {
            // Check if element exists
            if (set.find(id) == set.end())
            {
                RCLCPP_INFO(node->get_logger(), "Element %s does not exist. Skipping removal", id.c_str());
                return false;
            }
            // Remove element
            set.erase(id);
            return true;
        }
    };

} // namespace flychams::core