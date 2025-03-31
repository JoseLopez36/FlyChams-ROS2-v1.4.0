#pragma once

// Standard includes
#include <sstream>
#include <algorithm>
#include <stdexcept>
#include <type_traits>
#include <fstream>
#include <iostream>
#define _USE_MATH_DEFINES
#include <cmath>
#include <memory>

/* XLSX Files Management: https://github.com/troldal/OpenXLSX */
#define NOMINMAX
#include <OpenXLSX.hpp>

// Core includes
#include "flychams_core/types/core_types.hpp"
#include "flychams_core/types/config_types.hpp"
#include "flychams_core/utils/math_utils.hpp"

namespace flychams::core
{
	/**
	 * ════════════════════════════════════════════════════════════════
	 * @brief Spreadsheet parser for mission configuration
	 *
	 * @details
	 * This class is responsible for parsing the spreadsheet file
	 * containing the mission configuration.
	 *
	 * ════════════════════════════════════════════════════════════════
	 * @author Jose Francisco Lopez Ruiz
	 * @date 2025-01-25
	 * ════════════════════════════════════════════════════════════════
	 */
	class SpreadsheetParser
	{
	public: // Types
		using XLDocument = OpenXLSX::XLDocument;
		using XLWorkbook = OpenXLSX::XLWorkbook;
		using XLWorksheet = OpenXLSX::XLWorksheet;
		using XLRow = OpenXLSX::XLRow;
		using XLCell = OpenXLSX::XLCell;

	public: // Public methods
		static MissionConfigPtr parseSpreadsheet(const std::string& path)
		{
			try
			{
				// Create output
				MissionConfigPtr config_ptr = std::make_shared<MissionConfig>();

				// Open document
				XLDocument doc;
				doc.suppressWarnings(); // Disable warnings
				doc.open(path);
				doc.showWarnings(); // Enable warnings
				auto book = doc.workbook();

				// 1. Parse Mission Sheet
				parseMission(book, config_ptr);

				// 2. Parse Environment Sheet
				parseEnvironment(book, config_ptr);

				// 3. Parse Target Sheet
				parseTargetGroup(book, config_ptr);

				// 4. Parse Agent Sheet
				parseAgentTeam(book, config_ptr);

				// Close document
				doc.close();

				return config_ptr;
			}
			catch (const std::exception& e)
			{
				throw std::runtime_error("Spreadsheet parsing failed: " + std::string(e.what()));
			}
		}

	private: // Implementation methods
		static void parseMission(OpenXLSX::XLWorkbook& book, MissionConfigPtr config_ptr)
		{
			// Open Mission sheet
			auto sheet = book.worksheet("Mission");

			// Iterate through data rows
			for (auto& row : sheet.rows())
			{
				// Skip first two header rows
				if (row.rowNumber() < 3)
					continue;

				try
				{
					// Verify row is not empty. If empty, throw error
					if (isCellEmpty(row.findCell(2)))
					{
						throw std::runtime_error("Mission config not found");
					}

					// Check if the mission is selected
					if (isCellEmpty(row.findCell(1)))
					{
						continue;
					}

					// Read selected value
					std::string selected = getCellValue<std::string>(row.findCell(1));
					if (selected == "X" || selected == "x")
					{
						// Populate fields
						config_ptr->id = getCellValue<std::string>(row.findCell(2));

						config_ptr->name = getCellValue<std::string>(row.findCell(3));

						config_ptr->environment_id = getCellValue<std::string>(row.findCell(4));

						config_ptr->target_group_id = getCellValue<std::string>(row.findCell(5));

						config_ptr->agent_team_id = getCellValue<std::string>(row.findCell(6));

						auto altitude_constraint_str = getCellValue<std::string>(row.findCell(7));
						auto altitude_constraint_vec = parseVector<float>(altitude_constraint_str, 2, ',');
						if (altitude_constraint_vec.size() >= 2)
						{
							config_ptr->altitude_constraint(0) = altitude_constraint_vec[0];
							config_ptr->altitude_constraint(1) = altitude_constraint_vec[1];
						}

						const std::string& autopilot_str = getCellValue<std::string>(row.findCell(8));
						config_ptr->autopilot = autopilotFromString(autopilot_str);

						auto start_date_str = getCellValue<std::string>(row.findCell(9));
						auto start_date_vec = parseVector<int>(start_date_str, 3, '/');
						if (start_date_vec.size() >= 3)
						{
							config_ptr->start_date.year = start_date_vec[2];
							config_ptr->start_date.month = start_date_vec[1];
							config_ptr->start_date.day = start_date_vec[0];
						}

						auto start_hour_str = getCellValue<std::string>(row.findCell(10));
						auto start_hour_vec = parseVector<int>(start_hour_str, 3, ':');
						if (start_hour_vec.size() >= 3)
						{
							config_ptr->start_hour.hours = start_hour_vec[0];
							config_ptr->start_hour.minutes = start_hour_vec[1];
							config_ptr->start_hour.seconds = start_hour_vec[2];
						}

						// Only first found mission is loaded
						return;
					}
				}
				catch (const std::exception& e)
				{
					throw std::runtime_error("Error loading mission config at row " + std::to_string(row.rowNumber()) + ": " + e.what());
				}
			}
		}

		static void parseEnvironment(OpenXLSX::XLWorkbook& book, MissionConfigPtr config_ptr)
		{
			// Open Environment sheet
			auto sheet = book.worksheet("Environment");

			// Create environment config
			EnvironmentConfig environment;

			// Iterate through data rows
			for (auto& row : sheet.rows())
			{
				// Skip first two header rows
				if (row.rowNumber() < 3)
					continue;

				try
				{
					// Verify row is not empty. If empty, throw error
					if (isCellEmpty(row.findCell(1)))
					{
						throw std::runtime_error("Environment config not found");
					}

					// Read primary key
					std::string PK = getCellValue<std::string>(row.findCell(1));

					// Verify if the environment is selected
					if (PK == config_ptr->environment_id)
					{
						// Populate fields
						environment.id = PK;

						environment.name = getCellValue<std::string>(row.findCell(2));

						auto origin_geopoint_str = getCellValue<std::string>(row.findCell(3));
						auto origin_geopoint_vec = parseVector<float>(origin_geopoint_str, 3, ',');
						if (origin_geopoint_vec.size() >= 3)
						{
							environment.geopoint.latitude = origin_geopoint_vec[0];
							environment.geopoint.longitude = origin_geopoint_vec[1];
							environment.geopoint.altitude = origin_geopoint_vec[2];
						}

						auto wind_velocity_str = getCellValue<std::string>(row.findCell(4));
						auto wind_velocity_vec = parseVector<float>(wind_velocity_str, 3, ',');
						if (wind_velocity_vec.size() >= 3)
						{
							environment.wind_vel(0) = wind_velocity_vec[0];
							environment.wind_vel(1) = wind_velocity_vec[1];
							environment.wind_vel(2) = wind_velocity_vec[2];
						}

						// Only first found environment is loaded
						config_ptr->environment = environment;
						return;
					}
				}
				catch (const std::exception& e)
				{
					throw std::runtime_error("Error loading environment config at row " + std::to_string(row.rowNumber()) + ": " + e.what());
				}
			}
		}

		static void parseTargetGroup(OpenXLSX::XLWorkbook& book, MissionConfigPtr config_ptr)
		{
			// Open Target sheet
			auto sheet = book.worksheet("Target");

			// Iterate through data rows
			for (auto& row : sheet.rows())
			{
				// Skip first two header rows
				if (row.rowNumber() < 3)
					continue;

				try
				{
					// Verify row is not empty
					if (isCellEmpty(row.findCell(1)))
						return; // Return filled target group

					// Read foreign key and verify if the target belongs to the group selected
					std::string FK = getCellValue<std::string>(row.findCell(3));

					if (FK == config_ptr->target_group_id)
					{
						// Create instance
						TargetConfigPtr target = std::make_shared<TargetConfig>();

						// Populate fields
						target->id = getCellValue<std::string>(row.findCell(1));

						target->name = getCellValue<std::string>(row.findCell(2));

						target->target_group_id = FK;

						const std::string& target_type_str = getCellValue<std::string>(row.findCell(4));
						target->type = targetTypeFromString(target_type_str);

						target->count = getCellValue<int>(row.findCell(5));

						const std::string& target_priority_str = getCellValue<std::string>(row.findCell(6));
						target->priority = priorityFromString(target_priority_str);

						target->trajectory_folder = getCellValue<std::string>(row.findCell(7));

						// Add target count times to the map
						for (int i = 0; i < target->count; i++)
						{
							// Create instance
							TargetConfigPtr target_unit = std::make_shared<TargetConfig>();

							// Populate ID and the rest of the fields
							std::stringstream ss;
							ss << target->id << "COUNT" << std::setw(2) << std::setfill('0') << i;
							target_unit->id = ss.str();
							target_unit->name = target->name;
							target_unit->target_group_id = target->target_group_id;
							target_unit->target_index = i;
							target_unit->type = target->type;
							target_unit->count = target->count;
							target_unit->priority = target->priority;
							target_unit->trajectory_folder = target->trajectory_folder;

							// Store setting
							config_ptr->target_group.insert({ target_unit->id, target_unit });
						}
					}
				}
				catch (const std::exception& e)
				{
					throw std::runtime_error("Error loading target config at row " + std::to_string(row.rowNumber()) + ": " + e.what());
				}
			}
		}

		static void parseAgentTeam(OpenXLSX::XLWorkbook& book, MissionConfigPtr config_ptr)
		{
			// Open Agent sheet
			auto sheet = book.worksheet("Agent");

			// Iterate through data rows
			for (auto& row : sheet.rows())
			{
				// Skip first two header rows
				if (row.rowNumber() < 3)
					continue;

				try
				{
					// Verify row is not empty
					if (isCellEmpty(row.findCell(1)))
						return; // Return filled agent team

					// Read foreign key and verify if the agent belongs to the team selected
					std::string FK = getCellValue<std::string>(row.findCell(3));

					if (FK == config_ptr->agent_team_id)
					{
						// Create instance
						AgentConfigPtr agent = std::make_shared<AgentConfig>();

						// Populate fields
						agent->id = getCellValue<std::string>(row.findCell(1));

						agent->name = getCellValue<std::string>(row.findCell(2));

						agent->agent_team_id = FK;

						agent->tracking_id = getCellValue<std::string>(row.findCell(4));

						agent->head_payload_id = getCellValue<std::string>(row.findCell(5));

						agent->drone_id = getCellValue<std::string>(row.findCell(6));

						auto position_str = getCellValue<std::string>(row.findCell(7));
						auto position_vec = parseVector<float>(position_str, 3, ',');
						if (position_vec.size() >= 3)
						{
							agent->position(0) = position_vec[0];
							agent->position(1) = position_vec[1];
							agent->position(2) = position_vec[2];
						}

						auto orientation_str = getCellValue<std::string>(row.findCell(8));
						auto orientation_vec = parseVector<float>(orientation_str, 3, ',');
						if (orientation_vec.size() >= 3)
						{
							agent->orientation(0) = MathUtils::degToRad(orientation_vec[0]);
							agent->orientation(1) = MathUtils::degToRad(orientation_vec[1]);
							agent->orientation(2) = MathUtils::degToRad(orientation_vec[2]);
						}

						agent->max_altitude = getCellValue<float>(row.findCell(9));

						agent->safety_radius = getCellValue<float>(row.findCell(10));

						agent->battery_capacity = getCellValue<float>(row.findCell(11));

						// Parse tracking
						parseTracking(book, agent);

						// Parse drone model
						parseDroneModel(book, agent);

						// Parse head payload
						parseHeadPayload(book, agent);

						// Store setting
						config_ptr->agent_team.insert({ agent->id, agent });
					}
				}
				catch (const std::exception& e)
				{
					throw std::runtime_error("Error loading agent config at row " + std::to_string(row.rowNumber()) + ": " + e.what());
				}
			}
		}

		static void parseTracking(OpenXLSX::XLWorkbook& book, AgentConfigPtr agent_ptr)
		{
			// Open Tracking sheet
			auto sheet = book.worksheet("Tracking");

			// Iterate through data rows
			for (auto& row : sheet.rows())
			{
				// Skip first two header rows
				if (row.rowNumber() < 3)
					continue;

				try
				{
					// Verify row is not empty. If empty, throw error
					if (isCellEmpty(row.findCell(1)))
					{
						throw std::runtime_error("Tracking config not found");
					}

					// Read primary key and verify if the tracking is selected
					std::string PK = getCellValue<std::string>(row.findCell(1));

					if (PK == agent_ptr->tracking_id)
					{
						// Create instance
						TrackingConfig tracking;

						// Populate fields
						tracking.id = PK;

						tracking.name = getCellValue<std::string>(row.findCell(2));

						const std::string& tracking_mode_str = getCellValue<std::string>(row.findCell(3));
						tracking.mode = trackingModeFromString(tracking_mode_str);

						tracking.min_target_size = getCellValue<float>(row.findCell(4));

						tracking.max_target_size = getCellValue<float>(row.findCell(5));

						tracking.ref_target_size = getCellValue<float>(row.findCell(6));

						tracking.num_windows = getCellValue<int>(row.findCell(7));

						auto scene_resolution_str = getCellValue<std::string>(row.findCell(8));
						auto scene_resolution_vec = parseVector<int>(scene_resolution_str, 2, 'x');
						if (scene_resolution_vec.size() >= 2)
						{
							tracking.scene_resolution(0) = scene_resolution_vec[0];
							tracking.scene_resolution(1) = scene_resolution_vec[1];
						}

						auto tracking_resolution_str = getCellValue<std::string>(row.findCell(9));
						auto tracking_resolution_vec = parseVector<int>(tracking_resolution_str, 2, 'x');
						if (tracking_resolution_vec.size() >= 2)
						{
							tracking.tracking_resolution(0) = tracking_resolution_vec[0];
							tracking.tracking_resolution(1) = tracking_resolution_vec[1];
						}

						// Only first found tracking is loaded
						agent_ptr->tracking = tracking;
						return;
					}
				}
				catch (const std::exception& e)
				{
					throw std::runtime_error("Error loading tracking config at row " + std::to_string(row.rowNumber()) + ": " + e.what());
				}
			}
		}

		static void parseHeadPayload(OpenXLSX::XLWorkbook& book, AgentConfigPtr agent_ptr)
		{
			// Open Head sheet
			auto sheet = book.worksheet("Head");

			// Iterate through data rows
			for (auto& row : sheet.rows())
			{
				// Skip first two header rows
				if (row.rowNumber() < 3)
					continue;

				try
				{
					// Verify row is not empty
					if (isCellEmpty(row.findCell(1)))
						return; // Return filled head payload

					// Read foreign key and verify if the head belongs to the payload selected
					std::string FK = getCellValue<std::string>(row.findCell(3));

					if (FK == agent_ptr->head_payload_id)
					{
						// Create instance
						HeadConfigPtr head = std::make_shared<HeadConfig>();

						// Populate fields
						head->id = getCellValue<std::string>(row.findCell(1));

						head->name = getCellValue<std::string>(row.findCell(2));

						head->head_payload_id = FK;

						head->gimbal_id = getCellValue<std::string>(row.findCell(4));

						head->camera_id = getCellValue<std::string>(row.findCell(5));

						const std::string& role_str = getCellValue<std::string>(row.findCell(6));
						head->role = headRoleFromString(role_str);

						auto position_str = getCellValue<std::string>(row.findCell(7));
						auto position_vec = parseVector<float>(position_str, 3, ',');
						if (position_vec.size() >= 3)
						{
							head->position(0) = position_vec[0];
							head->position(1) = position_vec[1];
							head->position(2) = position_vec[2];
						}

						auto orientation_str = getCellValue<std::string>(row.findCell(8));
						auto orientation_vec = parseVector<float>(orientation_str, 3, ',');
						if (orientation_vec.size() >= 3)
						{
							head->orientation(0) = MathUtils::degToRad(orientation_vec[0]);
							head->orientation(1) = MathUtils::degToRad(orientation_vec[1]);
							head->orientation(2) = MathUtils::degToRad(orientation_vec[2]);
						}

						head->min_focal = getCellValue<float>(row.findCell(9)) / 1000.0f;

						head->max_focal = getCellValue<float>(row.findCell(10)) / 1000.0f;

						head->ref_focal = getCellValue<float>(row.findCell(11)) / 1000.0f;

						// Parse gimbal model
						parseGimbalModel(book, head);

						// Parse camera model
						parseCameraModel(book, head);

						// Store setting
						agent_ptr->head_payload.insert({ head->id, head });
					}
				}
				catch (const std::exception& e)
				{
					throw std::runtime_error("Error loading head config at row " + std::to_string(row.rowNumber()) + ": " + e.what());
				}
			}
		}

		static void parseDroneModel(OpenXLSX::XLWorkbook& book, AgentConfigPtr agent_ptr)
		{
			// Open Drone sheet
			auto sheet = book.worksheet("Drone");

			// Iterate through data rows
			for (auto& row : sheet.rows())
			{
				// Skip first two header rows
				if (row.rowNumber() < 3)
					continue;

				try
				{
					// Verify row is not empty. If empty, throw error
					if (isCellEmpty(row.findCell(1)))
					{
						throw std::runtime_error("Drone config not found");
					}

					// Read primary key and verify if the drone model is selected
					std::string PK = getCellValue<std::string>(row.findCell(1));

					if (PK == agent_ptr->drone_id)
					{
						// Create instance
						DroneConfig drone;

						// Populate fields
						drone.id = PK;

						drone.name = getCellValue<std::string>(row.findCell(2));

						const std::string& drone_type_str = getCellValue<std::string>(row.findCell(3));
						drone.type = droneTypeFromString(drone_type_str);

						drone.cruise_speed = getCellValue<float>(row.findCell(4));

						drone.max_speed = getCellValue<float>(row.findCell(5));

						drone.enable_barometer = getCellValue<bool>(row.findCell(6));

						auto barometer_str = getCellValue<std::string>(row.findCell(7));
						auto barometer_vec = parseVector<float>(barometer_str, 2, ',');
						if (barometer_vec.size() >= 2)
						{
							drone.barometer.pressure_factor_sigma = barometer_vec[0];
							drone.barometer.uncorrelated_noise_sigma = barometer_vec[1];
						}

						drone.enable_imu = getCellValue<bool>(row.findCell(8));

						auto imu_str = getCellValue<std::string>(row.findCell(9));
						auto imu_vec = parseVector<float>(imu_str, 2, ',');
						if (imu_vec.size() >= 2)
						{
							drone.imu.gyro_noise = imu_vec[0];
							drone.imu.accel_noise = imu_vec[1];
						}

						drone.enable_gps = getCellValue<bool>(row.findCell(10));

						auto gps_str = getCellValue<std::string>(row.findCell(11));
						auto gps_vec = parseVector<float>(gps_str, 4, ',');
						if (gps_vec.size() >= 4)
						{
							drone.gps.eph_initial = gps_vec[0];
							drone.gps.epv_initial = gps_vec[1];
							drone.gps.eph_final = gps_vec[2];
							drone.gps.epv_final = gps_vec[3];
						}

						drone.enable_magnetometer = getCellValue<bool>(row.findCell(12));

						auto magnetometer_str = getCellValue<std::string>(row.findCell(13));
						auto magnetometer_vec = parseVector<float>(magnetometer_str, 3, ',');
						if (magnetometer_vec.size() >= 3)
						{
							drone.magnetometer.noise_sigma = magnetometer_vec[0];
							drone.magnetometer.scale_factor = magnetometer_vec[1];
							drone.magnetometer.noise_bias = magnetometer_vec[2];
						}

						drone.base_weight = getCellValue<float>(row.findCell(14));

						drone.max_payload_weight = getCellValue<float>(row.findCell(15));

						drone.hover_power = getCellValue<float>(row.findCell(16));

						drone.cruise_power = getCellValue<float>(row.findCell(17));

						drone.load_factor = getCellValue<float>(row.findCell(18));

						// Only first found drone is loaded
						agent_ptr->drone = drone;
						return;
					}
				}
				catch (const std::exception& e)
				{
					throw std::runtime_error("Error loading drone config at row " + std::to_string(row.rowNumber()) + ": " + e.what());
				}
			}
		}

		static void parseGimbalModel(OpenXLSX::XLWorkbook& book, HeadConfigPtr head_ptr)
		{
			// Open Gimbal sheet
			auto sheet = book.worksheet("Gimbal");

			// Iterate through data rows
			for (auto& row : sheet.rows())
			{
				// Skip first two header rows
				if (row.rowNumber() < 3)
					continue;

				try
				{
					// Verify row is not empty. If empty, throw error
					if (isCellEmpty(row.findCell(1)))
					{
						throw std::runtime_error("Gimbal config not found");
					}

					// Read primary key and verify if the gimbal model is selected
					std::string PK = getCellValue<std::string>(row.findCell(1));

					if (PK == head_ptr->gimbal_id)
					{
						// Create instance
						GimbalConfig gimbal;

						// Populate fields
						gimbal.id = PK;

						gimbal.name = getCellValue<std::string>(row.findCell(2));

						gimbal.enable_roll = getCellValue<bool>(row.findCell(3));

						auto roll_str = getCellValue<std::string>(row.findCell(4));
						auto roll_vec = parseVector<float>(roll_str, 3, ',');
						if (roll_vec.size() >= 3)
						{
							gimbal.roll.min_angle = roll_vec[0];
							gimbal.roll.max_angle = roll_vec[1];
							gimbal.roll.max_speed = roll_vec[2];
						}

						gimbal.enable_pitch = getCellValue<bool>(row.findCell(5));

						auto pitch_str = getCellValue<std::string>(row.findCell(6));
						auto pitch_vec = parseVector<float>(pitch_str, 3, ',');
						if (pitch_vec.size() >= 3)
						{
							gimbal.pitch.min_angle = pitch_vec[0];
							gimbal.pitch.max_angle = pitch_vec[1];
							gimbal.pitch.max_speed = pitch_vec[2];
						}

						gimbal.enable_yaw = getCellValue<bool>(row.findCell(7));

						auto yaw_str = getCellValue<std::string>(row.findCell(8));
						auto yaw_vec = parseVector<float>(yaw_str, 3, ',');
						if (yaw_vec.size() >= 3)
						{
							gimbal.yaw.min_angle = yaw_vec[0];
							gimbal.yaw.max_angle = yaw_vec[1];
							gimbal.yaw.max_speed = yaw_vec[2];
						}

						gimbal.weight = getCellValue<float>(row.findCell(9));

						gimbal.idle_power = getCellValue<float>(row.findCell(10));

						gimbal.active_power = getCellValue<float>(row.findCell(11));

						// Only first found gimbal is loaded
						head_ptr->gimbal = gimbal;
						return;
					}
				}
				catch (const std::exception& e)
				{
					throw std::runtime_error("Error loading gimbal config at row " + std::to_string(row.rowNumber()) + ": " + e.what());
				}
			}
		}

		static void parseCameraModel(OpenXLSX::XLWorkbook& book, HeadConfigPtr head_ptr)
		{
			// Open Camera sheet
			auto sheet = book.worksheet("Camera");

			// Iterate through data rows
			for (auto& row : sheet.rows())
			{
				// Skip first two header rows
				if (row.rowNumber() < 3)
					continue;

				try
				{
					// Verify row is not empty. If empty, throw error
					if (isCellEmpty(row.findCell(1)))
					{
						throw std::runtime_error("Camera config not found");
					}

					// Read primary key and verify if the camera model is selected
					std::string PK = getCellValue<std::string>(row.findCell(1));

					if (PK == head_ptr->camera_id)
					{
						// Create instance
						CameraConfig camera;

						// Populate fields
						camera.id = PK;

						camera.name = getCellValue<std::string>(row.findCell(2));

						const std::string& type_str = getCellValue<std::string>(row.findCell(3));
						camera.type = cameraTypeFromString(type_str);

						auto resolution_str = getCellValue<std::string>(row.findCell(4));
						auto resolution_vec = parseVector<int>(resolution_str, 2, 'x');
						if (resolution_vec.size() >= 2)
						{
							camera.resolution(0) = resolution_vec[0];
							camera.resolution(1) = resolution_vec[1];
						}

						auto sensor_size_str = getCellValue<std::string>(row.findCell(5));
						auto sensor_size_vec = parseVector<float>(sensor_size_str, 2, 'x');
						if (sensor_size_vec.size() >= 2)
						{
							camera.sensor_size(0) = sensor_size_vec[0] / 1000.0f;
							camera.sensor_size(1) = sensor_size_vec[1] / 1000.0f;
						}

						camera.enable_lens_distortion = getCellValue<bool>(row.findCell(6));

						auto lens_distortion_str = getCellValue<std::string>(row.findCell(7));
						auto lens_distortion_vec = parseVector<float>(lens_distortion_str, 3, ',');
						if (lens_distortion_vec.size() >= 3)
						{
							camera.lens_distortion.strength = lens_distortion_vec[0];
							camera.lens_distortion.area_radius = lens_distortion_vec[1];
							camera.lens_distortion.area_falloff = lens_distortion_vec[2];
						}

						camera.enable_sensor_noise = getCellValue<bool>(row.findCell(8));

						auto sensor_noise_str = getCellValue<std::string>(row.findCell(9));
						auto sensor_noise_vec = parseVector<float>(sensor_noise_str, 3, ',');
						if (sensor_noise_vec.size() >= 3)
						{
							camera.sensor_noise.rand_contrib = sensor_noise_vec[0];
							camera.sensor_noise.rand_size = sensor_noise_vec[1];
							camera.sensor_noise.rand_speed = sensor_noise_vec[2];
						}

						camera.weight = getCellValue<float>(row.findCell(10));

						camera.idle_power = getCellValue<float>(row.findCell(11));

						camera.active_power = getCellValue<float>(row.findCell(12));

						// Only first found camera is loaded
						head_ptr->camera = camera;
						return;
					}
				}
				catch (const std::exception& e)
				{
					throw std::runtime_error("Error loading camera config at row " + std::to_string(row.rowNumber()) + ": " + e.what());
				}
			}
		}

	private: // Helper methods
		// Check if a cell is empty
		static bool isCellEmpty(const OpenXLSX::XLCell& cell)
		{
			return cell.empty() || cell.value().type() == OpenXLSX::XLValueType::Empty;
		}

		// Get the value of a cell or fail
		template <typename T>
		static T getCellValue(const OpenXLSX::XLCell& cell)
		{
			try
			{
				const OpenXLSX::XLValueType cell_type = cell.value().type();
				const std::string cell_ref = cell.cellReference().address();

				// Handle empty cells
				if (isCellEmpty(cell))
				{
					throw std::runtime_error("Empty cell at " + cell_ref + ".");
				}

				// Handle - as empty value (does not care about this)
				if (cell_type == OpenXLSX::XLValueType::String && cell.value().get<std::string>() == "-")
				{
					return T();
				}

				// Direct type match
				if constexpr (std::is_same_v<T, std::string>)
				{
					if (cell_type == OpenXLSX::XLValueType::String)
					{
						std::string val = cell.value().get<std::string>();
						return val;
					}
				}
				else if constexpr (std::is_same_v<T, bool>)
				{
					if (cell_type == OpenXLSX::XLValueType::Boolean)
					{
						bool val = cell.value().get<bool>();
						return val;
					}
				}
				else if constexpr (std::is_arithmetic_v<T>)
				{
					if (cell_type == OpenXLSX::XLValueType::Integer)
					{
						int val = cell.value().get<int>();
						return val;
					}
					else if (cell_type == OpenXLSX::XLValueType::Float)
					{
						float val = cell.value().get<float>();
						return val;
					}
				}
			}
			catch (const std::exception& e)
			{
				throw std::runtime_error("Error loading cell: " + std::string(e.what()));
			}

			return T();
		}

		// Parse a string to a vector of a given type
		template <typename T>
		static std::vector<T> parseVector(const std::string& input, size_t expected_count, const char separator)
		{
			std::vector<T> result;

			// Handle empty input
			if (input.empty())
			{
				return result;
			}

			// Sanitize input: remove all non-numeric/non-delimiter characters
			std::string clean_input = input;
			auto is_valid_char = [separator](char c)
				{
					return std::isdigit(c) || c == '-' || c == '.' || c == separator;
				};

			clean_input.erase(
				std::remove_if(clean_input.begin(), clean_input.end(),
					[&](char c)
					{ return !is_valid_char(c); }),
				clean_input.end());

			std::istringstream iss(clean_input);
			std::string token;
			size_t index = 0;

			try
			{
				while (std::getline(iss, token, separator))
				{
					// Remove leading/trailing whitespace from each token
					token.erase(std::remove_if(token.begin(), token.end(), ::isspace), token.end());

					if (token.empty())
						continue;

					std::istringstream converter(token);
					T value;
					if (!(converter >> value))
					{
						throw std::runtime_error("Failed to convert '" + token + "' to type");
					}

					result.push_back(value);
					index++;
				}

				if (expected_count > 0 && result.size() != expected_count)
				{
					std::cout << "Expected " << expected_count << " elements, got " << result.size() << "." << std::endl;
					throw std::invalid_argument("Expected " + std::to_string(expected_count) +
						" elements, got " + std::to_string(result.size()));
				}
			}
			catch (const std::exception& e)
			{
				std::cout << "Failed to parse: " << e.what() << "." << std::endl;
				throw;
			}

			return result;
		}
	};

} // namespace flychams::core