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

/* JSON Files Management: https://github.com/nlohmann/json */
#include <nlohmann/json.hpp>

// Core includes
#include "flychams_core/types/core_types.hpp"
#include "flychams_core/config/config_types.hpp"
#include "flychams_core/utils/math_utils.hpp"
#include "flychams_core/utils/camera_utils.hpp"

namespace flychams::core
{
	/**
	 * ════════════════════════════════════════════════════════════════
	 * @brief Configuration parser for mission settings
	 *
	 * @details
	 * This class is responsible for parsing the Excel file containing
	 * mission configuration and creating AirSim settings based on
	 * the parsed configuration.
	 *
	 * ════════════════════════════════════════════════════════════════
	 * @author Jose Francisco Lopez Ruiz
	 * @date 2025-01-25
	 * ════════════════════════════════════════════════════════════════
	 */
	class ConfigParser
	{
	public: // Public methods
		static ConfigPtr parseExcelFile(const std::string& path);
		static bool parseAirsimSettings(const ConfigPtr& config_ptr, const std::string& path);

	private: // Configuration parsing methods
		static MissionConfigPtr parseMission(const OpenXLSX::XLWorksheet& sheet);
		static SimulationConfigPtr parseSimulation(const OpenXLSX::XLWorksheet& sheet, const ID& simulation_id);
		static MapConfigPtr parseMap(const OpenXLSX::XLWorksheet& sheet, const ID& map_id);
		static GroupConfigMap parseTargets(const OpenXLSX::XLWorksheet& sheet, const ID& bundle_id);
		static AgentConfigMap parseAgents(OpenXLSX::XLWorkbook& book, const ID& team_id, const Vector2r& mission_altitude_constraint);
		static HeadConfigMap parseAgentPayload(OpenXLSX::XLWorkbook& book, const ID& payload_id);
		static DroneConfigPtr parseDroneModel(const OpenXLSX::XLWorksheet& sheet, const ID& drone_id);
		static GimbalConfigPtr parseGimbalModel(OpenXLSX::XLWorkbook& book, const ID& gimbal_id);
		static GimbalLinkConfigMap parseGimbalLinks(const OpenXLSX::XLWorksheet& sheet, const ID& links_id);
		static CameraConfigPtr parseCameraModel(const OpenXLSX::XLWorksheet& sheet, const ID& camera_id);

	private: // AirSim settings methods
		static void populateGeneralSettings(const ConfigPtr& config_ptr, nlohmann::ordered_json& settings);
		static void populateVehicles(const ConfigPtr& config_ptr, nlohmann::ordered_json& settings);
		static void populateCameras(const ID& agent_id, const ConfigPtr& config_ptr, const HeadConfigMap& heads, nlohmann::ordered_json& cameras);
		static void populateExternalCameras(const ID& agent_id, const bool& is_first_agent, const ConfigPtr& config_ptr, nlohmann::ordered_json& cameras);
		static void populateSubWindows(nlohmann::ordered_json& settings);

	private: // Helper methods
		// Check if a cell is empty
		static bool isCellEmpty(const OpenXLSX::XLCell& cell)
		{
			return cell.empty();
		}

		// Get the value of a cell or fail
		template <typename T>
		static T getCellValueOrFail(const OpenXLSX::XLCell& cell)
		{
			try
			{
				const OpenXLSX::XLValueType cell_type = cell.value().type();
				const std::string cell_ref = cell.cellReference().address();

				// Handle empty cells
				if (cell_type == OpenXLSX::XLValueType::Empty)
				{
					throw std::runtime_error("Empty cell at " + cell_ref + ".");
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

		// Get the value of a cell or a default value if the cell is empty
		template <typename T>
		static T getCellValueOrDefault(const OpenXLSX::XLCell& cell, const T& default_value = T())
		{
			try
			{
				const OpenXLSX::XLValueType cell_type = cell.value().type();
				const std::string cell_ref = cell.cellReference().address();

				// Handle empty cells
				if (cell_type == OpenXLSX::XLValueType::Empty)
				{
					std::cout << "Empty cell at " << cell_ref << ". Using default value." << std::endl;
					return default_value;
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
				std::cout << "Error loading cell: " << e.what() << std::endl;
				throw;
			}

			return default_value;
		}

		// Parse a string to a vector of a given type
		template <typename T>
		static std::vector<T> parseStringToVector(const std::string& input, size_t expected_count, const char separator)
		{
			std::vector<T> result;

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