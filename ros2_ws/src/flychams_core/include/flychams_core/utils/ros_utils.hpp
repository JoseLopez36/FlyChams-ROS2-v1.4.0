#pragma once

// Standard includes
#include <regex>

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
        // TIMERS: Timer utilities
        // ════════════════════════════════════════════════════════════════════════════

        static Time now(NodePtr node)
        {
            return node->get_clock()->now();
        }

        static TimerPtr createTimer(NodePtr node, float rate, const std::function<void()>& callback, CallbackGroupPtr callback_group = nullptr)
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

        static TimerPtr createWallTimer(NodePtr node, float rate, const std::function<void()>& callback, CallbackGroupPtr callback_group = nullptr)
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
        // PARAMETERS: Parameter utilities
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
        // SERVICES: Service utilities
        // ════════════════════════════════════════════════════════════════════════════

        template<typename T>
        static bool sendRequest(NodePtr node, ClientPtr<T> client, typename T::Request::SharedPtr request, int wait_time_ms = 1000)
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
        // MESSAGES: Message utilities
        // ════════════════════════════════════════════════════════════════════════════

        static Vector3r fromMsg(const PointMsg& point)
        {
            return Vector3r{ static_cast<float>(point.x), static_cast<float>(point.y), static_cast<float>(point.z) };
        }

        static Vector3r fromMsg(const Vector3Msg& vector)
        {
            return Vector3r{ static_cast<float>(vector.x), static_cast<float>(vector.y), static_cast<float>(vector.z) };
        }

        static Quaternionr fromMsg(const QuaternionMsg& quat)
        {
            return Quaternionr{ static_cast<float>(quat.x), static_cast<float>(quat.y), static_cast<float>(quat.z), static_cast<float>(quat.w) };
        }

        static Matrix4r fromMsg(const TransformMsg& transform)
        {
            Matrix4r T = Matrix4r::Identity();
            T.block<3, 1>(0, 3) = fromMsg(transform.translation);
            T.block<3, 3>(0, 0) = MathUtils::quaternionToRotationMatrix(fromMsg(transform.rotation));
            return T;
        }

        static void toMsg(const Vector3r& vector, PointMsg& point)
        {
            point.x = static_cast<double>(vector.x());
            point.y = static_cast<double>(vector.y());
            point.z = static_cast<double>(vector.z());
        }

        static void toMsg(const Vector3r& vector, Vector3Msg& vec)
        {
            vec.x = static_cast<double>(vector.x());
            vec.y = static_cast<double>(vector.y());
            vec.z = static_cast<double>(vector.z());
        }

        static void toMsg(const Quaternionr& orientation, QuaternionMsg& quat)
        {
            quat.x = static_cast<double>(orientation.x());
            quat.y = static_cast<double>(orientation.y());
            quat.z = static_cast<double>(orientation.z());
            quat.w = static_cast<double>(orientation.w());
        }

        static void toMsg(const Matrix4r& matrix, TransformMsg& transform)
        {
            toMsg(matrix.block<3, 1>(0, 3), transform.translation);
            toMsg(MathUtils::rotationMatrixToQuaternion(matrix.block<3, 3>(0, 0)), transform.rotation);
        }

        static void toMsg(const Crop& crop, CropMsg& crop_msg)
        {
            crop_msg.x = crop.x;
            crop_msg.y = crop.y;
            crop_msg.w = crop.w;
            crop_msg.h = crop.h;
            crop_msg.is_out_of_bounds = crop.is_out_of_bounds;
        }

        // ════════════════════════════════════════════════════════════════════════════
        // OTHER: Other utilities
        // ════════════════════════════════════════════════════════════════════════════

        static std::string replace(const std::string& topic_name, const std::string& placeholder, const std::string& value)
        {
            return std::regex_replace(topic_name, std::regex(placeholder), value);
        }

        static HeaderMsg createHeader(NodePtr node, const std::string& frame_id)
        {
            HeaderMsg header;
            header.frame_id = frame_id;
            header.stamp = now(node);
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
    };

} // namespace flychams::core