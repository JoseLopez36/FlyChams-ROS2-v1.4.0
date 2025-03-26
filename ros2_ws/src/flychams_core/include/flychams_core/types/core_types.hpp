#pragma once

// Standard includes
#include <string>
#include <vector>
#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <set>

// Eigen includes
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace flychams::core
{
    /**
     * ════════════════════════════════════════════════════════════════
     * @brief Types used throughout the project
     *
     * @details
     * This file contains all the types used throughout the project,
     * including basic types, geometry types, and other types.
     * ════════════════════════════════════════════════════════════════
     */

    // ════════════════════════════════════════════════════════════════
    // IDENTIFIER TYPES: Identifier types used throughout the project
    // ════════════════════════════════════════════════════════════════

    using ID = std::string;
    using IDs = std::vector<ID>;
    using UnorderedIDs = std::unordered_set<ID>;
    using OrderedIDs = std::set<ID>;
    using Name = std::string;

    // ════════════════════════════════════════════════════════════════
    // ENUM TYPES: Enum types used throughout the project
    // ════════════════════════════════════════════════════════════════

    /**
     * @brief Framework enumeration
     */
    enum class Framework
    {
        None,
        AirSim
    };
    inline Framework frameworkFromString(const std::string& framework)
    {
        if (framework == "AirSim") return Framework::AirSim;
        return Framework::None;
    }

    /**
     * Enum for tracking modes
     */
    enum class TrackingMode
    {
        None,                   // No tracking
        MultiCameraTracking,    // Multiple orientable and zoom-adjustable cameras tracking targets
        MultiWindowTracking,    // Multiple tracking windows in a single ultra-high-resolution camera
        PriorityHybridTracking  // Hybrid tracking based on priorities
    };
    inline TrackingMode trackingModeFromString(const std::string& tracking_mode)
    {
        if (tracking_mode == "MultiCamera") return TrackingMode::MultiCameraTracking;
        if (tracking_mode == "MultiWindow") return TrackingMode::MultiWindowTracking;
        if (tracking_mode == "PriorityHybrid") return TrackingMode::PriorityHybridTracking;
        return TrackingMode::None;
    }

    /**
     * Enum for head roles
     */
    enum class HeadRole
    {
        None,     // No role assigned
        Central,  // Central view (wide angle)
        Tracking  // Tracking view (zoomed in)
    };
    inline HeadRole headRoleFromString(const std::string& head_role)
    {
        if (head_role == "Central") return HeadRole::Central;
        if (head_role == "Tracking") return HeadRole::Tracking;
        return HeadRole::None;
    }

    /**
     * Enum for target types
     */
    enum class TargetType
    {
        None,     // No type assigned
        Cube,     // Cube target
        Human,    // Human target
        MetaHuman // MetaHuman target (more realistic UE human model)
    };
    inline TargetType targetTypeFromString(const std::string& target_type)
    {
        if (target_type == "Cube") return TargetType::Cube;
        if (target_type == "Human") return TargetType::Human;
        if (target_type == "MetaHuman") return TargetType::MetaHuman;
        return TargetType::None;
    }

    /**
     * Enum for drone types
     */
    enum class DroneType
    {
        None,           // No type assigned
        Quadcopter,     // Quadcopter
        Hexacopter      // Hexacopter
    };
    inline DroneType droneTypeFromString(const std::string& drone_type)
    {
        if (drone_type == "Quadcopter") return DroneType::Quadcopter;
        if (drone_type == "Hexacopter") return DroneType::Hexacopter;
        return DroneType::None;
    }

    /**
     * Enum for camera types
     */
    enum class CameraType
    {
        None,           // No type assigned
        RGB,            // Standard RGB camera
        Infrared,       // Infrared camera
        Depth           // Depth camera
    };
    inline CameraType cameraTypeFromString(const std::string& camera_type)
    {
        if (camera_type == "RGB") return CameraType::RGB;
        if (camera_type == "Infrared") return CameraType::Infrared;
        if (camera_type == "Depth") return CameraType::Depth;
        return CameraType::None;
    }

    /**
     * Enum for gimbal axis types
     */
    enum class AxisType
    {
        None,           // No type assigned
        Roll,           // Roll axis (rotation around X)
        Pitch,          // Pitch axis (rotation around Y)
        Yaw             // Yaw axis (rotation around Z)
    };
    inline AxisType axisTypeFromString(const std::string& axis_type)
    {
        if (axis_type == "Roll") return AxisType::Roll;
        if (axis_type == "Pitch") return AxisType::Pitch;
        if (axis_type == "Yaw") return AxisType::Yaw;
        return AxisType::None;
    }

    /**
     * Enum for region types
     */
    enum class RegionType
    {
        None,           // No type assigned
        Coastal         // Coastal environment
    };
    inline RegionType regionTypeFromString(const std::string& region_type)
    {
        if (region_type == "Coastal") return RegionType::Coastal;
        return RegionType::None;
    }

    /**
     * Enum for autopilot types
     */
    enum class Autopilot
    {
        None,           // No type assigned
        SimpleFlight,   // Simple flight autopilot (AirSim default)
        PX4             // PX4 autopilot
    };
    inline Autopilot autopilotFromString(const std::string& autopilot_type)
    {
        if (autopilot_type == "SimpleFlight") return Autopilot::SimpleFlight;
        if (autopilot_type == "PX4") return Autopilot::PX4;
        return Autopilot::None;
    }

    /**
     * Enum for target priority
     */
    enum class Priority
    {
        None,     // No priority assigned
        Low,      // Low priority
        Medium,   // Medium priority
        High      // High priority
    };
    inline Priority priorityFromString(const std::string& priority)
    {
        if (priority == "Low") return Priority::Low;
        if (priority == "Medium") return Priority::Medium;
        if (priority == "High") return Priority::High;
        return Priority::None;
    }

    /**
     * @brief Tracking unit type enumeration
     */
    enum class TrackingUnitType
    {
        None,
        Physical,
        Digital
    };

    /**
     * @brief Mission status enumeration
     */
    enum class MissionStatus
    {
        NotStarted,
        Running,
        Paused,
        Completed,
        Aborted,
        Error
    };

    /**
     * Enum for element types
     */
    enum class ElementType
    {
        Agent,   // Aerial vehicle (UAV)
        Target,  // Target to track
        Cluster  // Group of targets
    };

    // ════════════════════════════════════════════════════════════════
    // MATHEMATICAL TYPES: Eigen types used throughout the project
    // ════════════════════════════════════════════════════════════════

    using Vector2r = Eigen::Vector2f;
    using Vector2i = Eigen::Vector2i;
    using Vector2d = Eigen::Vector2d;

    using Vector3r = Eigen::Vector3f;
    using Vector3i = Eigen::Vector3i;
    using Vector3d = Eigen::Vector3d;

    using Vector4r = Eigen::Vector4f;
    using Vector4i = Eigen::Vector4i;
    using Vector4d = Eigen::Vector4d;

    using VectorXr = Eigen::VectorXf;
    using VectorXi = Eigen::VectorXi;
    using VectorXd = Eigen::VectorXd;

    using RowVector2r = Eigen::RowVector2f;
    using RowVector2i = Eigen::RowVector2i;
    using RowVector2d = Eigen::RowVector2d;

    using RowVector3r = Eigen::RowVector3f;
    using RowVector3i = Eigen::RowVector3i;
    using RowVector3d = Eigen::RowVector3d;

    using RowVector4r = Eigen::RowVector4f;
    using RowVector4i = Eigen::RowVector4i;
    using RowVector4d = Eigen::RowVector4d;

    using RowVectorXr = Eigen::RowVectorXf;
    using RowVectorXi = Eigen::RowVectorXi;
    using RowVectorXd = Eigen::RowVectorXd;

    using Matrix2r = Eigen::Matrix2f;
    using Matrix2i = Eigen::Matrix2i;
    using Matrix2d = Eigen::Matrix2d;

    using Matrix3r = Eigen::Matrix3f;
    using Matrix3i = Eigen::Matrix3i;
    using Matrix3d = Eigen::Matrix3d;

    using Matrix4r = Eigen::Matrix4f;
    using Matrix4i = Eigen::Matrix4i;
    using Matrix4d = Eigen::Matrix4d;

    using MatrixXr = Eigen::MatrixXf;
    using MatrixXi = Eigen::MatrixXi;
    using MatrixXd = Eigen::MatrixXd;

    using Matrix2Xr = Eigen::Matrix2Xf;
    using Matrix2Xi = Eigen::Matrix2Xi;
    using Matrix2Xd = Eigen::Matrix2Xd;

    using Matrix3Xr = Eigen::Matrix3Xf;
    using Matrix3Xi = Eigen::Matrix3Xi;
    using Matrix3Xd = Eigen::Matrix3Xd;

    using Matrix4Xr = Eigen::Matrix4Xf;
    using Matrix4Xi = Eigen::Matrix4Xi;
    using Matrix4Xd = Eigen::Matrix4Xd;

    using Quaternionr = Eigen::Quaternionf;

    // ════════════════════════════════════════════════════════════════
    // TRACKING TYPES: Tracking-related types used throughout the project
    // ════════════════════════════════════════════════════════════════

    /**
     * Camera parameters
     */
    struct CameraParameters
    {
        // Camera ID
        std::string id;
        // Camera intrinsic matrix K
        Matrix3r k_ref;
        // Focal lengths (m)
        float f_min;
        float f_max;
        float f_ref;
        // Image resolution (pix)
        int width;
        int height;
        // Sensor dimensions (m)
        float sensor_width;
        float sensor_height;
        // Regularized pixel size (m/pix)
        float rho;
    };

    /**
     * Window parameters
     */
    struct WindowParameters
    {
        // Source camera parameters
        CameraParameters camera_params;
        // Resolution factors (0-1)
        float lambda_min;
        float lambda_max;
        float lambda_ref;
        // Full resolution (pix)
        int full_width;
        int full_height;
        // Scene resolution (pix)
        int scene_width;
        int scene_height;
        // Tracking resolution (pix)
        int tracking_width;
        int tracking_height;
    };

    /**
     * Projection parameters
     */
    struct ProjectionParameters
    {
        // Apparent target sizes (pix)
        float s_min_pix;
        float s_max_pix;
        float s_ref_pix;
        // Apparent target sizes (m)
        float s_min;
        float s_max;
        float s_ref;
    };

    /**
     * Tracking parameters
     */
    struct TrackingParameters
    {
        TrackingMode mode;
        int n;                                               // Number of tracking heads or windows
        std::vector<CameraParameters> camera_params;         // Camera parameters for each tracking head
        std::vector<WindowParameters> window_params;         // Window parameters for each tracking window
        std::vector<ProjectionParameters> projection_params; // Projection parameters for each tracking head or window
    };

    /**
     * Crop
     */
    struct Crop
    {
        Vector2i corner; // Top-left corner of the crop
        Vector2i size;   // Width and height of the crop

        Crop() = default;
        Crop(int x, int y, int width, int height)
            : corner(x, y), size(width, height) {
        }
    };

    /**
     * Tracking goal
     */
    struct MultiCameraTrackingGoal
    {
        // Head data
        std::vector<std::string> window_ids;     // IDs of the windows where the tracking image will be displayed
        std::vector<std::string> head_ids;       // IDs of the physical heads
        std::vector<Vector3r> angles;            // Each column is rpy of a head
        std::vector<float> fovs;                 // Each element is the fov of a head

        MultiCameraTrackingGoal() = default;
        MultiCameraTrackingGoal(int n)
            : window_ids(n), head_ids(n), angles(n), fovs(n) {
        }
    };

    struct MultiWindowTrackingGoal
    {
        // Head data
        std::vector<std::string> window_ids;     // IDs of the windows where the tracking image will be displayed
        std::string camera_id;                   // ID of the camera where the crop will be taken from
        std::vector<Crop> crops;                 // Each element is a crop of the original image

        MultiWindowTrackingGoal() = default;
        MultiWindowTrackingGoal(int n)
            : window_ids(n), camera_id(), crops(n) {
        }
    };

    struct PriorityHybridTrackingGoal
    {
        // TODO: Implement
    };

    // ════════════════════════════════════════════════════════════════
    // METRICS TYPES: Metrics types used throughout the project
    // ════════════════════════════════════════════════════════════════

    /**
     * Agent metrics
     */
    struct AgentMetrics
    {
        // Agent data
        float curr_x;
        float curr_y;
        float curr_z;
        float curr_yaw;
        float vel_x;
        float vel_y;
        float vel_z;
        float vel_yaw;
        float goal_x;
        float goal_y;
        float goal_z;
        float goal_yaw;
        // Position and movement metrics
        float total_distance_traveled;
        float current_speed;
        // Goal-related metrics
        float distance_to_goal;
        // Mission metrics
        float time_elapsed;
        // Performance metrics
        float average_speed;
    };

    /**
     * Target metrics
     */
    struct TargetMetrics
    {
        // Target data
        float curr_x;
        float curr_y;
        float curr_z;
        // Position and movement metrics
        float total_distance_traveled;
    };

    /**
     * Cluster metrics
     */
    struct ClusterMetrics
    {
        // Cluster data
        float curr_center_x;
        float curr_center_y;
        float curr_center_z;
        float curr_radius;
        // Position and movement metrics
        float total_distance_traveled;
    };

    /**
     * Global metrics
     */
    struct GlobalMetrics
    {
        // Overall system metrics
        int total_agents;
        int total_targets;
        int total_clusters;

        // Mission metrics
        float mission_time;
    };

    // ════════════════════════════════════════════════════════════════
    // OTHER TYPES: Other types used throughout the project
    // ════════════════════════════════════════════════════════════════

    struct DateTime
    {
        uint32_t year;
        uint32_t month;
        uint32_t day;
    };

    struct HourTime
    {
        uint32_t hours;
        uint32_t minutes;
        uint32_t seconds;
    };

    struct Coordinates
    {
        double latitude;
        double longitude;
        double altitude;
    };

    struct Barometer
    {
        float pressure_factor_sigma;    // Pressure factor sigma
        float uncorrelated_noise_sigma; // Uncorrelated noise sigma
    };

    struct Imu
    {
        float gyro_noise;        // Angular random walk
        float accel_noise;       // Velocity random walk
    };

    struct Gps
    {
        float eph_initial;       // Initial horizontal position accuracy
        float epv_initial;       // Initial vertical position accuracy
        float eph_final;         // Final horizontal position accuracy
        float epv_final;         // Final vertical position accuracy
    };

    struct Magnetometer
    {
        float noise_sigma;       // Sensor noise sigma
        float scale_factor;      // Scale factor for readings
        float noise_bias;        // Noise bias
    };

    struct Link
    {
        float min_angle;
        float max_angle;
        float max_speed;
    };

    struct LensDistortion
    {
        float strength;
        float area_radius;
        float area_falloff;
    };

    struct SensorNoise
    {
        float rand_contrib;
        float rand_size;
        float rand_speed;
    };

} // namespace flychams::core 