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
    // IDENTIFIER TYPES: Identifier types used throughout the project
    // ════════════════════════════════════════════════════════════════

    using ID = std::string;
    using IDs = std::vector<ID>;
    using Name = std::string;

    // ════════════════════════════════════════════════════════════════
    // ENUM TYPES: Enum types used throughout the project
    // ════════════════════════════════════════════════════════════════

    /**
     * Enum for element types
     */
    enum class ElementType
    {
        Agent,   // Aerial vehicle (UAV)
        Target,  // Target to track
        Cluster  // Group of targets
    };

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
     * Enum for tracking modes
     */
    enum class TrackingMode
    {
        None,                   // No tracking
        MultiCamera,            // Multiple orientable and zoom-adjustable cameras tracking targets
        MultiWindow,            // Multiple tracking windows in a single ultra-high-resolution camera
        PriorityHybrid          // Hybrid tracking based on priorities
    };
    inline TrackingMode trackingModeFromString(const std::string& tracking_mode)
    {
        if (tracking_mode == "MultiCamera") return TrackingMode::MultiCamera;
        if (tracking_mode == "MultiWindow") return TrackingMode::MultiWindow;
        if (tracking_mode == "PriorityHybrid") return TrackingMode::PriorityHybrid;
        return TrackingMode::None;
    }

    /**
     * Enum for tracking roles
     */
    enum class TrackingRole
    {
        None,     // No role assigned
        Central,  // Central role
        Tracking  // Tracking role
    };
    inline TrackingRole trackingRoleFromString(const std::string& tracking_role)
    {
        if (tracking_role == "Central") return TrackingRole::Central;
        if (tracking_role == "Tracking") return TrackingRole::Tracking;
        return TrackingRole::None;
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
     * Enum for agent states
     */
    enum class AgentStatus
    {
        IDLE,                // 0: Initial state, UAV is inactive
        DISARMED,            // 1: UAV is disarmed, safe state
        ARMED,               // 2: UAV is armed, ready for takeoff
        TAKING_OFF,          // 3: UAV is taking off
        TAKEN_OFF,           // 4: UAV has taken off
        HOVERING,            // 5: UAV is hovering
        HOVERED,             // 6: UAV has hovered
        TRACKING,            // 7: UAV is tracking targets
        LANDING,             // 8: UAV is landing
        LANDED,              // 9: UAV has landed
        ERROR                // 10: Error state, requires reset
    };

    // ════════════════════════════════════════════════════════════════
    // TRACKING TYPES: Tracking-related types used throughout the project
    // ════════════════════════════════════════════════════════════════

    /**
     * Head parameters
     */
    struct HeadParameters
    {
        // Head ID
        std::string id;
        // Head role
        TrackingRole role;
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
        // Camera intrinsic matrix K
        Matrix3r K;
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
     * Window parameters
     */
    struct WindowParameters
    {
        // Window ID
        std::string id;
        // Window role
        TrackingRole role;
        // Source camera ID
        std::string source_id;
        // Resolution factors (0-1)
        float lambda_min;
        float lambda_max;
        float lambda_ref;
        // Full resolution (pix)
        int full_width;
        int full_height;
        // Tracking resolution (pix)
        int tracking_width;
        int tracking_height;
        // Regularized pixel size (m/pix)
        float rho;
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
        int n_heads;                                      // Number of heads
        int n_windows;                                    // Number of windows
        std::vector<HeadParameters> head_params;          // Parameters for each head
        std::vector<WindowParameters> window_params;      // Parameters for each window
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
        float white_noise_sigma;        // White noise sigma
    };

    struct Imu
    {
        float angular_white_noise_sigma;    // Angular white noise sigma
        float velocity_white_noise_sigma;   // Velocity white noise sigma
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
        float white_noise_sigma;       // White noise sigma
        float white_noise_bias;        // White noise bias
    };

    struct Link
    {
        float min_angle;
        float max_angle;
        float max_speed;
    };

    struct Distortion
    {
        float K1;
        float K2;
        float K3;
        float P1;
        float P2;
    };

    struct SensorNoise
    {
        float rand_contrib;
        float rand_size;
        float rand_speed;
    };

    struct Crop
    {
        int x;
        int y;
        int w;
        int h;
        bool is_out_of_bounds;
    };  

} // namespace flychams::core 