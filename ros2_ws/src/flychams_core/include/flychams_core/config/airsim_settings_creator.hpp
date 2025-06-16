#pragma once

/* JSON Files Management: https://github.com/nlohmann/json */
#include <nlohmann/json.hpp>

// Core includes
#include "flychams_core/types/core_types.hpp"
#include "flychams_core/types/config_types.hpp"
#include "flychams_core/utils/math_utils.hpp"

namespace flychams::core
{
    /**
     * ════════════════════════════════════════════════════════════════
     * @brief AirSim settings creator
     *
     * @details
     * This class is responsible for creating AirSim settings from the
     * given mission configuration.
     *
     * ════════════════════════════════════════════════════════════════
     * @author Jose Francisco Lopez Ruiz
     * @date 2025-03-25
     * ════════════════════════════════════════════════════════════════
     */
    class AirsimSettingsCreator
    {
    public: // Public methods
        static bool createAirsimSettings(const MissionConfigPtr& config_ptr, const std::string& path)
        {
            // Generate settings.json content
            nlohmann::ordered_json settings;
            populateGeneralSettings(config_ptr, settings);
            populateVehicles(config_ptr, settings["Vehicles"]);
            populateSubWindows(config_ptr, settings["SubWindows"]);

            // Write settings to file
            std::ofstream file(path);
            if (file.is_open())
            {
                file << settings.dump(4);
                file.close();
            }
            else
            {
                std::cerr << "Error writing settings to file " << path << std::endl;
                return false;
            }

            return true;
        }

    private: // Implementation methods
        static void populateGeneralSettings(const MissionConfigPtr& config_ptr, nlohmann::ordered_json& settings)
        {
            // General settings
            settings["SettingsVersion"] = 2.0;
            settings["SimMode"] = "Multirotor";
            settings["ClockType"] = "SteppableClock";
            settings["ClockSpeed"] = config_ptr->system.clock_speed;
            settings["ViewMode"] = "NoDisplay";
            settings["LogMessagesVisible"] = false;
            settings["ApiServerPort"] = 41451;

            // Pawn paths
            settings["PawnPaths"] = {
                {"FlyChamsQuadcopter", {{"PawnBP", "Class'/AirSim/Blueprints/BP_FlyChamsQuadcopter.BP_FlyChamsQuadcopter_C'"}}},
                {"FlyChamsHexacopter", {{"PawnBP", "Class'/AirSim/Blueprints/BP_FlyChamsHexacopter.BP_FlyChamsHexacopter_C'"}}} };

            // Origin geopoint
            settings["OriginGeopoint"] = {
                {"Latitude", config_ptr->environment.geopoint.latitude},
                {"Longitude", config_ptr->environment.geopoint.longitude},
                {"Altitude", config_ptr->environment.geopoint.altitude} };

            // Time of day (format: %Y-%m-%d %H:%M:%S)
            const auto& date = config_ptr->start_date;
            const auto& hour = config_ptr->start_hour;
            settings["TimeOfDay"] = {
                {"Enabled", true},
                {"StartDateTime",
                 std::to_string(date.year) + "-" +
                     std::to_string(date.month) + "-" +
                     std::to_string(date.day) + " " +
                     std::to_string(hour.hours) + ":" +
                     std::to_string(hour.minutes) + ":" +
                     std::to_string(hour.seconds)},
                {"CelestialClockSpeed", 1},
                {"StartDateTimeDst", false},
                {"UpdateIntervalSecs", 1} };

            // Wind velocity
            const auto& wind_vel = config_ptr->environment.wind_vel;
            settings["Wind"] = {
                {"X", wind_vel.x()},
                {"Y", -wind_vel.y()},
                {"Z", -wind_vel.z()} };

            // Camera defaults
            settings["CameraDefaults"] = {
                {"CaptureSettings", {
                    {
                        {"ImageType", 0},
                        {"Width", 1920},
                        {"Height", 1080},
                        {"SensorWidth", 0.0132f},
                        {"SensorHeight", 0.007425f},
                        {"FOV_Degrees", 90},
                        {"LumenGIEnable", true},
                        {"LumenReflectionEnable", true},
                        {"LumenFinalQuality", 2},
                        {"LumenSceneDetail", 2},
                        {"LumenSceneLightningDetail", 2}
                    }
                }}
            };
        }

        static void populateVehicles(const MissionConfigPtr& config_ptr, nlohmann::ordered_json& vehicles)
        {
            bool first_time = true;
            int instance = 0;
            for (const auto& [agent_id, agent_ptr] : config_ptr->agent_team)
            {
                // Get relevant config
                const auto& drone = agent_ptr->drone;
                Vector3r ini_pos = agent_ptr->position;
                Vector3r ini_ori = agent_ptr->orientation;

                vehicles[agent_id] = {
                    {"PawnPath", drone.type == DroneType::Quadcopter ? "FlyChamsQuadcopter" : "FlyChamsHexacopter"},
                    {"X", ini_pos.x()},
                    {"Y", -ini_pos.y()},
                    {"Z", -ini_pos.z()},
                    {"Roll", MathUtils::radToDeg(ini_ori.x())},
                    {"Pitch", MathUtils::radToDeg(-ini_ori.y())},
                    {"Yaw", MathUtils::radToDeg(-ini_ori.z())}
                };

                if (config_ptr->autopilot == Autopilot::PX4)
                {
                    vehicles[agent_id]["VehicleType"] = "PX4Multirotor";
                    vehicles[agent_id]["Model"] = drone.type == DroneType::Quadcopter ? "Quadcopter" : "FlyChamsHexacopter";
                    vehicles[agent_id]["UseSerial"] = false;
                    vehicles[agent_id]["LockStep"] = true;
                    vehicles[agent_id]["UseTcp"] = true;
                    vehicles[agent_id]["TcpPort"] = 4560 + instance;
                    vehicles[agent_id]["ControlIp"] = "remote";
                    vehicles[agent_id]["ControlPortLocal"] = 14540 + instance;
                    vehicles[agent_id]["ControlPortRemote"] = 14580 + instance;
                    vehicles[agent_id]["LocalHostIp"] = "172.17.0.1";
                    vehicles[agent_id]["Parameters"] = {
                        {"NAV_RCL_ACT", 0},
                        {"NAV_DLL_ACT", 0},
                        {"COM_OBL_ACT", 1},
                        {"LPE_LAT", config_ptr->environment.geopoint.latitude},
                        {"LPE_LON", config_ptr->environment.geopoint.longitude}
                    };
                }
                else
                {
                    vehicles[agent_id]["VehicleType"] = "SimpleFlight";
                    vehicles[agent_id]["DefaultVehicleState"] = "Armed";
                    vehicles[agent_id]["AutoCreate"] = true;
                }

                // Add sensors to the vehicle
                populateSensors(agent_id, config_ptr, vehicles[agent_id]["Sensors"]);

                // Add cameras to the vehicle
                populateInternalCameras(agent_id, config_ptr, vehicles[agent_id]["Cameras"]);

                // Add external cameras to the vehicle
                populateExternalCameras(agent_id, config_ptr, vehicles[agent_id]["Cameras"]);

                instance++;
            }
        }

        // Helper method: Populate sensors
        static void populateSensors(const ID& agent_id, const MissionConfigPtr& config_ptr, nlohmann::ordered_json& sensors)
        {
            // Get relevant config
            const auto& drone = config_ptr->agent_team[agent_id]->drone;

            sensors = {
                {"Barometer", {
                    {"SensorType", 1},
                        {"Enabled", drone.enable_barometer},
                        {"PressureFactorSigma", 0.0001825f}, // More than 0.0001825 can generate problems with PX4
                        {"UncorrelatedNoiseSigma", drone.barometer.white_noise_sigma}
                    }},
                    {"Imu", {
                        {"SensorType", 2},
                        {"Enabled", drone.enable_imu},
                        {"GenerateNoise", true},
                        {"AngularRandomWalk", drone.imu.angular_white_noise_sigma},
                        {"VelocityRandomWalk", drone.imu.velocity_white_noise_sigma}
                    }},
                    {"Gps", {
                        {"SensorType", 3},
                        {"Enabled", drone.enable_gps},
                        {"EphInitial", drone.gps.eph_initial},
                        {"EpvInitial", drone.gps.epv_initial},
                        {"EphFinal", drone.gps.eph_final},
                        {"EpvFinal", drone.gps.epv_final}
                    }},
                    {"Magnetometer", {
                        {"SensorType", 4},
                        {"Enabled", drone.enable_magnetometer},
                        {"NoiseSigma", drone.magnetometer.white_noise_sigma},
                        {"NoiseBias", drone.magnetometer.white_noise_bias}
                }}
            };
        }

        // Helper method: Populate cameras
        static void populateInternalCameras(const ID& agent_id, const MissionConfigPtr& config_ptr, nlohmann::ordered_json& cameras)
        {
            for (const auto& [head_id, head_ptr] : config_ptr->agent_team[agent_id]->head_set)
            {
                // Get relevant config
                const auto& gimbal = head_ptr->gimbal;
                const auto& camera = head_ptr->camera;
                const auto& distortion = camera.distortion;
                const auto& mount_pos = head_ptr->position;
                const auto& mount_ori = head_ptr->orientation;

                cameras[head_id] = {
                    {"CaptureSettings", {
                        {
                            {"ImageType", 0},
                            {"Width", camera.resolution(0)},
                            {"Height", camera.resolution(1)},
                            {"SensorWidth", camera.sensor_size(0)},
                            {"SensorHeight", camera.sensor_size(1)},
                            {"FOV_Degrees", MathUtils::radToDeg(MathUtils::computeFov(head_ptr->ref_focal, camera.sensor_size(0)))},
                            {"K1", distortion.K1},
                            {"K2", distortion.K2},
                            {"K3", distortion.K3},
                            {"P1", distortion.P1},
                            {"P2", distortion.P2},
                            {"LumenGIEnable", true},
                            {"LumenReflectionEnable", true},
                            {"LumenFinalQuality", 2},
                            {"LumenSceneDetail", 2},
                            {"LumenSceneLightningDetail", 2}
                        }
                    }},
                    {"Gimbal", {
                        {"YawMin", gimbal.yaw.min_angle}, {"PitchMin", gimbal.pitch.min_angle}, {"RollMin", gimbal.roll.min_angle},
                        {"YawMax", gimbal.yaw.max_angle}, {"PitchMax", gimbal.pitch.max_angle}, {"RollMax", gimbal.roll.max_angle},
                        {"YawSpeed", gimbal.yaw.max_speed}, {"PitchSpeed", gimbal.pitch.max_speed}, {"RollSpeed", gimbal.roll.max_speed},
                        {"Roll", MathUtils::radToDeg(mount_ori.x())}, {"Pitch", 0.0f}, {"Yaw", MathUtils::radToDeg(-mount_ori.z())}
                    }},
                    {"NoiseSettings", {
                        {
                            {"Enabled", camera.enable_sensor_noise},
                            {"ImageType", 0},
                            {"RandContrib", camera.sensor_noise.rand_contrib},
                            {"RandSpeed", camera.sensor_noise.rand_speed},
                            {"RandSize", camera.sensor_noise.rand_size},
                            {"RandDensity", 2},
                            {"HorzWaveContrib", 0.0004f},
                            {"HorzWaveStrength", 0.0007f},
                            {"HorzWaveVertSize", 1.0f},
                            {"HorzWaveScreenSize", 1.0f},
                            {"HorzNoiseLinesContrib", 0.0008f},
                            {"HorzNoiseLinesDensityY", 0.0001f},
                            {"HorzNoiseLinesDensityXY", 0.004f},
                            {"HorzDistortionContrib", 0.0f},
                            {"HorzDistortionStrength", 0.0f}
                        }
                    }},
                    {"X", mount_pos.x()}, {"Y", -mount_pos.y()}, {"Z", -mount_pos.z()},
                    {"Roll", 0.0f}, {"Pitch", MathUtils::radToDeg(-mount_ori.y())}, {"Yaw", 0.0f},
                    {"EnableGimbal", true}, {"CameraVisible", true}, {"CameraScale", 0.7f}
                };
            }
        }

        static void populateExternalCameras(const ID& agent_id, const MissionConfigPtr& config_ptr, nlohmann::ordered_json& cameras)
        {
            // Get scenario view camera pose from config
            const auto& scenario_view_pos = config_ptr->system.scenario_camera_position;
            const auto& scenario_view_ori = config_ptr->system.scenario_camera_orientation;

            cameras["SCENARIOCAM"] = {
                {"X", scenario_view_pos.x()},
                {"Y", -scenario_view_pos.y()},
                {"Z", -scenario_view_pos.z()},
                {"Roll", MathUtils::radToDeg(scenario_view_ori.x())},
                {"Pitch", MathUtils::radToDeg(-scenario_view_ori.y())},
                {"Yaw", MathUtils::radToDeg(-scenario_view_ori.z())},
                {"External", true} };

            // Get agent view camera pose from config
            const auto& agent_view_pos = config_ptr->system.agent_camera_position;
            const auto& agent_view_ori = config_ptr->system.agent_camera_orientation;

            // Agent view camera
            cameras["AGENTCAM"] = {
                {"X", agent_view_pos.x()},
                {"Y", -agent_view_pos.y()},
                {"Z", -agent_view_pos.z()},
                {"Roll", MathUtils::radToDeg(agent_view_ori.x())},
                {"Pitch", MathUtils::radToDeg(-agent_view_ori.y())},
                {"Yaw", MathUtils::radToDeg(-agent_view_ori.z())} };

            // Get payload view camera pose from config
            const auto& payload_view_pos = config_ptr->system.payload_camera_position;
            const auto& payload_view_ori = config_ptr->system.payload_camera_orientation;

            // Payload view camera
            cameras["PAYLOADCAM"] = {
                {"X", payload_view_pos.x()},
                {"Y", -payload_view_pos.y()},
                {"Z", -payload_view_pos.z()},
                {"Roll", MathUtils::radToDeg(payload_view_ori.x())},
                {"Pitch", MathUtils::radToDeg(-payload_view_ori.y())},
                {"Yaw", MathUtils::radToDeg(-payload_view_ori.z())} };
        }

        static void populateSubWindows(const MissionConfigPtr& config_ptr, nlohmann::ordered_json& subwindows)
        {
            int idx = 0;
            subwindows = nlohmann::ordered_json::array();

            // Scene view sub-window
            subwindows.push_back({ {"WindowID", idx++},
                                   {"CameraName", ""},
                                   {"ImageType", 0},
                                   {"VehicleName", ""},
                                   {"Visible", true} });

            // Agent view sub-window
            subwindows.push_back({ {"WindowID", idx++},
                                   {"CameraName", ""},
                                   {"ImageType", 0},
                                   {"VehicleName", ""},
                                   {"Visible", true} });

            // Map view sub-window
            subwindows.push_back({ {"WindowID", idx++},
                                   {"CameraName", ""},
                                   {"ImageType", 0},
                                   {"VehicleName", ""},
                                   {"Visible", true} });

            // Payload view sub-window
            subwindows.push_back({ {"WindowID", idx++},
                                   {"CameraName", ""},
                                   {"ImageType", 0},
                                   {"VehicleName", ""},
                                   {"Visible", true} });

            // Tracking views sub-windows
            for (int i = 0; i < config_ptr->system.tracking_view_ids.size(); i++)
            {
                subwindows.push_back({ {"WindowID", idx++},
                                       {"CameraName", ""},
                                       {"ImageType", 0},
                                       {"VehicleName", ""},
                                       {"Visible", true} });
            }
        }
    };

} // namespace flychams::core