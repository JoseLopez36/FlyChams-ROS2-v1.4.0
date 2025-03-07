#include "flychams_dashboard/visualization/markers_factory.hpp"

using namespace flychams::core;

namespace flychams::dashboard
{
    // ════════════════════════════════════════════════════════════════════════════
    // MARKER ARRAYS CREATION
    // ════════════════════════════════════════════════════════════════════════════

    core::MarkerArrayMsg MarkersFactory::createAgentMarkers(const std::string& frame_id, const float& duration)
    {
        int marker_idx = 0;
        MarkerArrayMsg markers;

        // Create marker for UAV body (a blue point)
        MarkerMsg point_marker = createAgentPoint(marker_idx++, frame_id, duration);
        markers.markers.push_back(point_marker);

        // Create marker for UAV goal (a yellow point)
        MarkerMsg goal_marker = createAgentGoalPoint(marker_idx++, frame_id, duration);
        markers.markers.push_back(goal_marker);

        return markers;
    }

    core::MarkerArrayMsg MarkersFactory::createTargetMarkers(const std::string& frame_id, const float& duration)
    {
        int marker_idx = 0;
        MarkerArrayMsg markers;

        // Create marker for target point (a red point)
        MarkerMsg point_marker = createTargetPoint(marker_idx++, frame_id, duration);
        markers.markers.push_back(point_marker);

        return markers;
    }

    core::MarkerArrayMsg MarkersFactory::createClusterMarkers(const std::string& frame_id, const float& duration)
    {
        int marker_idx = 0;
        MarkerArrayMsg markers;

        // Create marker for cluster center (a yellow X)
        MarkerMsg center_marker = createClusterCenter(marker_idx++, frame_id, duration);
        markers.markers.push_back(center_marker);

        // Create marker for cluster boundary (a cyan semi-transparent cylinder)
        MarkerMsg boundary_marker = createClusterBoundary(marker_idx++, frame_id, duration);
        markers.markers.push_back(boundary_marker);

        return markers;
    }

    // ════════════════════════════════════════════════════════════════════════════
    // MARKERS CREATION
    // ════════════════════════════════════════════════════════════════════════════

    core::MarkerMsg MarkersFactory::createAgentPoint(const int& marker_idx, const std::string& frame_id, const float& duration)
    {
        // Create marker for UAV body (a blue point)
        MarkerMsg marker;
        marker.header.frame_id = frame_id;
        marker.id = marker_idx;
        marker.type = MarkerMsg::SPHERE;
        marker.action = MarkerMsg::ADD;
        marker.lifetime = rclcpp::Duration::from_seconds(duration);

        // Set initial pose
        marker.pose.position.x = 0.0f;
        marker.pose.position.y = 0.0f;
        marker.pose.position.z = 0.0f;
        marker.pose.orientation.w = 1.0f;

        // Set scale
        marker.scale.x = BASE_MARKER_SIZE * 1.0f; // length
        marker.scale.y = BASE_MARKER_SIZE * 1.0f; // width
        marker.scale.z = BASE_MARKER_SIZE * 1.0f; // height

        // Set color (blue)
        marker.color.r = 0.0f;
        marker.color.g = 0.0f;
        marker.color.b = 1.0f;
        marker.color.a = BASE_MARKER_ALPHA;

        return marker;
    }

    core::MarkerMsg MarkersFactory::createAgentGoalPoint(const int& marker_idx, const std::string& frame_id, const float& duration)
    {
        // Create marker for agent goal (a yellow point)
        MarkerMsg marker;
        marker.header.frame_id = frame_id;
        marker.id = marker_idx;
        marker.type = MarkerMsg::SPHERE;
        marker.action = MarkerMsg::ADD;
        marker.lifetime = rclcpp::Duration::from_seconds(duration);

        // Set initial pose
        marker.pose.position.x = 0.0f;
        marker.pose.position.y = 0.0f;
        marker.pose.position.z = 0.0f;
        marker.pose.orientation.w = 1.0f;

        // Set scale
        marker.scale.x = BASE_MARKER_SIZE * 1.0f; // length
        marker.scale.y = BASE_MARKER_SIZE * 1.0f; // width
        marker.scale.z = BASE_MARKER_SIZE * 1.0f; // height

        // Set color (yellow)
        marker.color.r = 1.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = BASE_MARKER_ALPHA;

        return marker;
    }

    core::MarkerMsg MarkersFactory::createTargetPoint(const int& marker_idx, const std::string& frame_id, const float& duration)
    {
        // Create marker for target (a red point)
        MarkerMsg marker;
        marker.header.frame_id = frame_id;
        marker.id = marker_idx;
        marker.type = MarkerMsg::SPHERE;
        marker.action = MarkerMsg::ADD;
        marker.lifetime = rclcpp::Duration::from_seconds(duration);

        // Set initial pose
        marker.pose.position.x = 0.0f;
        marker.pose.position.y = 0.0f;
        marker.pose.position.z = 0.0f;
        marker.pose.orientation.w = 1.0f;

        // Set scale
        marker.scale.x = BASE_MARKER_SIZE * 0.7f; // length
        marker.scale.y = BASE_MARKER_SIZE * 0.7f; // width
        marker.scale.z = BASE_MARKER_SIZE * 0.7f; // height

        // Set color (red)
        marker.color.r = 1.0f;
        marker.color.g = 0.0f;
        marker.color.b = 0.0f;
        marker.color.a = BASE_MARKER_ALPHA;

        return marker;
    }

    core::MarkerMsg MarkersFactory::createClusterCenter(const int& marker_idx, const std::string& frame_id, const float& duration)
    {
        // Create marker for cluster center (a yellow X)
        MarkerMsg marker;
        marker.header.frame_id = frame_id;
        marker.id = marker_idx;
        marker.type = MarkerMsg::LINE_LIST;
        marker.action = MarkerMsg::ADD;
        marker.lifetime = rclcpp::Duration::from_seconds(duration);

        // Set origin pose
        marker.pose.position.x = 0.0f;
        marker.pose.position.y = 0.0f;
        marker.pose.position.z = 0.0f;
        marker.pose.orientation.w = 1.0f;

        // Create the X shape using line segments
        float half_size = BASE_MARKER_SIZE * 0.5f;
        PointMsg p1, p2, p3, p4;
        // First line of X (top-left to bottom-right)
        p1.x = 0.0f - half_size;
        p1.y = 0.0f - half_size;
        p1.z = 0.0f;
        p2.x = 0.0f + half_size;
        p2.y = 0.0f + half_size;
        p2.z = 0.0f;
        // Second line of X (top-right to bottom-left)
        p3.x = 0.0f + half_size;
        p3.y = 0.0f - half_size;
        p3.z = 0.0f;
        p4.x = 0.0f - half_size;
        p4.y = 0.0f + half_size;
        p4.z = 0.0f;
        // Add points to marker
        marker.points.push_back(p1);
        marker.points.push_back(p2);
        marker.points.push_back(p3);
        marker.points.push_back(p4);

        // Set scale (line width)
        marker.scale.x = BASE_LINE_WIDTH * 0.5f;

        // Set color (yellow)
        marker.color.r = 1.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = BASE_MARKER_ALPHA;

        return marker;
    }

    core::MarkerMsg MarkersFactory::createClusterBoundary(const int& marker_idx, const std::string& frame_id, const float& duration)
    {
        // Create marker for cluster boundary (a cyan semi-transparent cylinder)
        MarkerMsg marker;
        marker.header.frame_id = frame_id;
        marker.id = marker_idx;
        marker.type = MarkerMsg::CYLINDER;
        marker.action = MarkerMsg::ADD;
        marker.lifetime = rclcpp::Duration::from_seconds(duration);

        // Set initial pose
        marker.pose.position.x = 0.0f;
        marker.pose.position.y = 0.0f;
        marker.pose.position.z = 0.0f;
        marker.pose.orientation.w = 1.0f;

        // Set initial scale
        marker.scale.x = 0.0f; // length
        marker.scale.y = 0.0f; // width
        marker.scale.z = 0.0f; // height

        // Set color (cyan, semi-transparent)
        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 1.0f;
        marker.color.a = 0.15f;

        return marker;
    }

    // ════════════════════════════════════════════════════════════════════════════
    // MARKERS UPDATE
    // ════════════════════════════════════════════════════════════════════════════

    void MarkersFactory::updateAgentPoint(const float& curr_x, const float& curr_y, const float& curr_z, const core::Time& time, core::MarkerMsg& marker)
    {
        // Update timestamp
        marker.header.stamp = time;

        // Update pose
        marker.pose.position.x = curr_x;
        marker.pose.position.y = curr_y;
        marker.pose.position.z = curr_z;
    }

    void MarkersFactory::updateAgentGoalPoint(const float& goal_x, const float& goal_y, const float& goal_z, const core::Time& time, core::MarkerMsg& marker)
    {
        // Update timestamp
        marker.header.stamp = time;

        // Update pose
        marker.pose.position.x = goal_x;
        marker.pose.position.y = goal_y;
        marker.pose.position.z = goal_z;
    }

    void MarkersFactory::updateTargetPoint(const float& curr_x, const float& curr_y, const float& curr_z, const core::Time& time, core::MarkerMsg& marker)
    {
        // Update timestamp
        marker.header.stamp = time;

        // Update pose
        marker.pose.position.x = curr_x;
        marker.pose.position.y = curr_y;
        marker.pose.position.z = curr_z;
    }

    void MarkersFactory::updateClusterCenter(const float& center_x, const float& center_y, const float& center_z, const core::Time& time, core::MarkerMsg& marker)
    {
        // Update timestamp
        marker.header.stamp = time;

        // Update X points
        const float half_size = BASE_MARKER_SIZE * 0.5f;
        // First line of X (top-left to bottom-right)
        auto& p1 = marker.points[0];
        p1.x = center_x - half_size;
        p1.y = center_y - half_size;
        p1.z = center_z;
        auto& p2 = marker.points[1];
        p2.x = center_x + half_size;
        p2.y = center_y + half_size;
        p2.z = center_z;
        // Second line of X (top-right to bottom-left)
        auto& p3 = marker.points[2];
        p3.x = center_x + half_size;
        p3.y = center_y - half_size;
        p3.z = center_z;
        auto& p4 = marker.points[3];
        p4.x = center_x - half_size;
        p4.y = center_y + half_size;
        p4.z = center_z;
    }

    void MarkersFactory::updateClusterBoundary(const float& center_x, const float& center_y, const float& center_z, const float& radius, const core::Time& time, core::MarkerMsg& marker)
    {
        // Update timestamp
        marker.header.stamp = time;

        // Update pose
        marker.pose.position.x = center_x;
        marker.pose.position.y = center_y;
        marker.pose.position.z = center_z;

        // Update scale based on cluster radius
        marker.scale.x = 2.0f * radius; // diameter
        marker.scale.y = 2.0f * radius; // diameter
        marker.scale.z = 0.05f;         // height of cylinder
    }

} // namespace flychams::dashboard