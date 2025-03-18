#include "flychams_core/config/config_parser.hpp"

using namespace OpenXLSX;

namespace flychams::core
{
	// Parse Excel file using OpenXLSX
	ConfigPtr ConfigParser::parseExcelFile(const std::string& path)
	{
		try
		{
			// Open document
			XLDocument doc;
			doc.suppressWarnings(); // Disable warnings
			doc.open(path);
			doc.showWarnings(); // Enable warnings
			auto book = doc.workbook();

			// 1. Parse Mission Sheet
			MissionConfigPtr mission = parseMission(book.worksheet("Missions"));

			// 2. Parse Simulation Sheet
			SimulationConfigPtr simulation = parseSimulation(book.worksheet("Simulations"), mission->simulation_id);

			// 3. Parse Map Sheet
			MapConfigPtr map = parseMap(book.worksheet("Maps"), mission->map_id);

			// 4. Parse Target Groups
			GroupConfigMap groups = parseTargets(book.worksheet("Targets"), mission->group_bundle_id);

			// 5. Parse Agents
			AgentConfigMap agents = parseAgents(book, mission->agent_team_id, mission->altitude_constraint);

			// Process groups to extract targets
			TargetConfigMap targets;
			for (const auto& [group_id, group] : groups)
			{
				// Extract number of targets in group
				const int& n = group->target_count;

				// Iterate through all targets in group
				for (int i = 0; i < n; i++)
				{
					// Create target config instance
					TargetConfigPtr target = std::make_shared<TargetConfig>();

					// Generate target ID
					std::stringstream ss;
					ss << group_id << "TARGET" << std::setw(2) << std::setfill('0') << i;
					const ID target_id = ss.str();

					// Get trajectory path
					const std::string& root = "/home/testuser/FlyChams-ROS2/config/Trajectories/";
					const std::string& folder = group->trajectory_folder;
					const std::string& file = "/TRAJ" + std::to_string(i + 1) + ".csv";
					const std::string& path = root + folder + file;

					// Populate fields
					target->target_id = target_id;
					target->target_type = group->target_type;
					target->target_priority = group->group_priority;
					target->trajectory_path = path;

					// Insert target config
					targets.insert({ target_id, target });
				}
			}

			// Assemble config message
			ConfigPtr config_ptr = std::make_shared<Config>();
			config_ptr->mission = mission;
			config_ptr->simulation = simulation;
			config_ptr->map = map;
			config_ptr->groups = groups;
			config_ptr->targets = targets;
			config_ptr->agents = agents;
			config_ptr->group_count = static_cast<int>(groups.size());
			config_ptr->agent_count = static_cast<int>(agents.size());

			// Close document
			doc.close();

			return config_ptr;
		}
		catch (const std::exception& e)
		{
			throw std::runtime_error("Excel parsing failed: " + std::string(e.what()));
		}
	}

	MissionConfigPtr ConfigParser::parseMission(const XLWorksheet& sheet)
	{
		// Create output
		MissionConfigPtr mission = std::make_shared<MissionConfig>();

		// Iterate through data rows
		for (auto& row : sheet.rows())
		{
			// Skip first two header rows
			if (row.rowNumber() < 3)
				continue;

			try
			{
				// Verify row is not empty
				if (isCellEmpty(row.findCell(2)))
					return nullptr; // No mission found

				// Read first column to verify if the mission is selected
				if (row.findCell(1).value().type() == OpenXLSX::XLValueType::Empty)
					continue;
				std::string isMissionSelected = getCellValueOrFail<std::string>(row.findCell(1));

				if (isMissionSelected == "X" || isMissionSelected == "x")
				{
					// Populate fields
					mission->mission_id = getCellValueOrFail<std::string>(row.findCell(2));

					mission->mission_name = getCellValueOrFail<std::string>(row.findCell(3));

					mission->simulation_id = getCellValueOrFail<std::string>(row.findCell(4));

					mission->map_id = getCellValueOrFail<std::string>(row.findCell(5));

					mission->group_bundle_id = getCellValueOrFail<std::string>(row.findCell(6));

					mission->parameter_set_id = getCellValueOrFail<std::string>(row.findCell(7));

					mission->agent_team_id = getCellValueOrFail<std::string>(row.findCell(8));

					const std::string& autopilot_str = getCellValueOrFail<std::string>(row.findCell(9));
					mission->autopilot = autopilotFromString(autopilot_str);

					auto altitude_constraint_str = getCellValueOrFail<std::string>(row.findCell(10));
					auto altitude_constraint_vec = parseStringToVector<float>(altitude_constraint_str, 2, ',');
					if (altitude_constraint_vec.size() >= 2)
					{
						mission->altitude_constraint(0) = altitude_constraint_vec[0];
						mission->altitude_constraint(1) = altitude_constraint_vec[1];
					}

					auto tracking_scene_resolution_str = getCellValueOrFail<std::string>(row.findCell(11));
					auto tracking_scene_resolution_vec = parseStringToVector<int>(tracking_scene_resolution_str, 2, 'x');
					if (tracking_scene_resolution_vec.size() >= 2)
					{
						mission->tracking_scene_resolution(0) = tracking_scene_resolution_vec[0];
						mission->tracking_scene_resolution(1) = tracking_scene_resolution_vec[1];
					}

					auto tracking_view_resolution_str = getCellValueOrFail<std::string>(row.findCell(12));
					auto tracking_view_resolution_vec = parseStringToVector<int>(tracking_view_resolution_str, 2, 'x');
					if (tracking_view_resolution_vec.size() >= 2)
					{
						mission->tracking_view_resolution(0) = tracking_view_resolution_vec[0];
						mission->tracking_view_resolution(1) = tracking_view_resolution_vec[1];
					}

					// Only first found mission is loaded
					return mission;
				}
			}
			catch (const std::exception& e)
			{
				throw std::runtime_error("Error loading mission config at row " + std::to_string(row.rowNumber()) + ": " + e.what());
			}
		}

		return nullptr;
	}

	SimulationConfigPtr ConfigParser::parseSimulation(const XLWorksheet& sheet, const ID& simulation_id)
	{
		// Create output
		SimulationConfigPtr simulation = std::make_shared<SimulationConfig>();

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
					return nullptr; // No simulation found

				// Read primary key
				std::string PK = getCellValueOrFail<std::string>(row.findCell(1));

				// Verify if the map is selected
				if (PK == simulation_id)
				{
					// Populate fields
					simulation->simulation_id = PK;

					simulation->simulation_name = getCellValueOrFail<std::string>(row.findCell(2));

					const std::string& autopilot_str = getCellValueOrFail<std::string>(row.findCell(3));
					simulation->autopilot = autopilotFromString(autopilot_str);

					simulation->clock_speed = getCellValueOrFail<float>(row.findCell(4));

					simulation->record_metrics = getCellValueOrFail<bool>(row.findCell(5));

					simulation->draw_rviz_markers = getCellValueOrFail<bool>(row.findCell(6));

					simulation->draw_world_markers = getCellValueOrFail<bool>(row.findCell(7));

					// Only first found simulation is loaded
					return simulation;
				}
			}
			catch (const std::exception& e)
			{
				throw std::runtime_error("Error loading simulation config at row " + std::to_string(row.rowNumber()) + ": " + e.what());
			}
		}

		return nullptr;
	}

	MapConfigPtr ConfigParser::parseMap(const XLWorksheet& sheet, const ID& map_id)
	{
		// Create output
		MapConfigPtr map = std::make_shared<MapConfig>();

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
					return nullptr; // No map found

				// Read primary key
				std::string PK = getCellValueOrFail<std::string>(row.findCell(1));

				// Verify if the map is selected
				if (PK == map_id)
				{
					// Populate fields
					map->map_id = PK;

					map->map_name = getCellValueOrFail<std::string>(row.findCell(2));

					const std::string& region_type_str = getCellValueOrFail<std::string>(row.findCell(3));
					map->region_type = regionTypeFromString(region_type_str);

					auto origin_geopoint_str = getCellValueOrFail<std::string>(row.findCell(4));
					auto origin_geopoint_vec = parseStringToVector<float>(origin_geopoint_str, 3, ',');
					if (origin_geopoint_vec.size() >= 3)
					{
						map->origin_geopoint.latitude = origin_geopoint_vec[0];
						map->origin_geopoint.longitude = origin_geopoint_vec[1];
						map->origin_geopoint.altitude = origin_geopoint_vec[2];
					}

					auto start_date_str = getCellValueOrFail<std::string>(row.findCell(5));
					auto start_date_vec = parseStringToVector<int>(start_date_str, 3, '/');
					if (start_date_vec.size() >= 3)
					{
						map->start_time.year = start_date_vec[2];
						map->start_time.month = start_date_vec[1];
						map->start_time.day = start_date_vec[0];
					}

					auto start_hour_str = getCellValueOrFail<std::string>(row.findCell(6));
					auto start_hour_vec = parseStringToVector<int>(start_hour_str, 3, ':');
					if (start_hour_vec.size() >= 3)
					{
						map->start_time.hours = start_hour_vec[0];
						map->start_time.minutes = start_hour_vec[1];
						map->start_time.seconds = start_hour_vec[2];
					}

					auto wind_velocity_str = getCellValueOrFail<std::string>(row.findCell(7));
					auto wind_velocity_vec = parseStringToVector<float>(wind_velocity_str, 3, ',');
					if (wind_velocity_vec.size() >= 3)
					{
						map->wind_velocity(0) = wind_velocity_vec[0];
						map->wind_velocity(1) = wind_velocity_vec[1];
						map->wind_velocity(2) = wind_velocity_vec[2];
					}

					// Only first found map is loaded
					return map;
				}
			}
			catch (const std::exception& e)
			{
				throw std::runtime_error("Error loading map config at row " + std::to_string(row.rowNumber()) + ": " + e.what());
			}
		}

		return nullptr;
	}

	GroupConfigMap ConfigParser::parseTargets(const XLWorksheet& sheet, const ID& bundle_id)
	{
		// Create output
		GroupConfigMap groups;

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
					return groups; // Return filled groups

				// Read foreign key and verify if the target belongs to the group selected
				std::string FK = getCellValueOrFail<std::string>(row.findCell(2));

				if (FK == bundle_id)
				{
					// Create instance
					GroupConfigPtr group = std::make_shared<GroupConfig>();

					// Populate fields
					group->group_id = getCellValueOrFail<std::string>(row.findCell(1));

					group->group_bundle_id = FK;

					group->group_name = getCellValueOrFail<std::string>(row.findCell(3));

					const std::string& target_type_str = getCellValueOrFail<std::string>(row.findCell(4));
					group->target_type = targetTypeFromString(target_type_str);

					group->target_count = getCellValueOrFail<int>(row.findCell(5));

					const std::string& group_priority_str = getCellValueOrFail<std::string>(row.findCell(6));
					group->group_priority = priorityFromString(group_priority_str);

					group->trajectory_folder = getCellValueOrFail<std::string>(row.findCell(7));

					// Store setting
					groups.insert({ group->group_id, group });
				}
			}
			catch (const std::exception& e)
			{
				throw std::runtime_error("Error loading group config at row " + std::to_string(row.rowNumber()) + ": " + e.what());
			}
		}

		return groups;
	}

	AgentConfigMap ConfigParser::parseAgents(OpenXLSX::XLWorkbook& book, const ID& team_id, const Vector2r& mission_altitude_constraint)
	{
		// Create output
		AgentConfigMap agents;

		// Iterate through data rows
		auto sheet = book.worksheet("Agents");

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
					return agents; // Return filled agents

				// Read foreign key and verify if the target belongs to the group selected
				std::string FK = getCellValueOrFail<std::string>(row.findCell(2));

				if (FK == team_id)
				{
					// Create instance
					AgentConfigPtr agent = std::make_shared<AgentConfig>();

					// Populate fields
					agent->agent_id = getCellValueOrFail<std::string>(row.findCell(1));

					agent->agent_team_id = FK;

					agent->agent_name = getCellValueOrFail<std::string>(row.findCell(3));

					agent->head_payload_id = getCellValueOrFail<std::string>(row.findCell(4));

					const std::string& tracking_mode_str = getCellValueOrFail<std::string>(row.findCell(5));
					agent->tracking_mode = trackingModeFromString(tracking_mode_str);

					agent->drone_model_id = getCellValueOrFail<std::string>(row.findCell(6));

					auto initial_position_str = getCellValueOrFail<std::string>(row.findCell(7));
					auto initial_position_vec = parseStringToVector<float>(initial_position_str, 3, ',');
					if (initial_position_vec.size() >= 3)
					{
						agent->initial_position(0) = initial_position_vec[0];
						agent->initial_position(1) = initial_position_vec[1];
						agent->initial_position(2) = initial_position_vec[2];
					}

					auto initial_orientation_str = getCellValueOrFail<std::string>(row.findCell(8));
					auto initial_orientation_vec = parseStringToVector<float>(initial_orientation_str, 3, ',');
					if (initial_orientation_vec.size() >= 3)
					{
						agent->initial_orientation(0) = MathUtils::degToRad(initial_orientation_vec[2]);
						agent->initial_orientation(1) = MathUtils::degToRad(initial_orientation_vec[0]);
						agent->initial_orientation(2) = MathUtils::degToRad(initial_orientation_vec[1]);
					}

					agent->safety_radius = getCellValueOrFail<float>(row.findCell(9));

					agent->max_altitude = getCellValueOrFail<float>(row.findCell(10));

					agent->battery_capacity = getCellValueOrFail<float>(row.findCell(11));

					// Resolve external ID references
					agent->heads = parseAgentPayload(book, agent->head_payload_id);
					if (agent->heads.empty())
						throw std::runtime_error("No heads found for agent " + agent->agent_id);
					agent->drone = parseDroneModel(book.worksheet("DroneModels"), agent->drone_model_id);
					if (!agent->drone)
						throw std::runtime_error("No drone model found for agent " + agent->agent_id);

					// Other parameters
					agent->central_head_id = "";
					agent->tracking_head_ids.clear();
					for (const auto& [head_id, head_config] : agent->heads)
					{
						switch (head_config->head_role)
						{
						case HeadRole::Central:
							agent->central_head_id = head_id;
							break;

						case HeadRole::Tracking:
							agent->tracking_head_ids.push_back(head_id);
							break;
						}
					}
					agent->min_admissible_height = mission_altitude_constraint(0);
					agent->max_admissible_height = std::min(agent->max_altitude, mission_altitude_constraint(1));
					agent->max_assignments = 0;
					if (agent->tracking_mode == TrackingMode::MultiCameraTracking)
						agent->max_assignments = static_cast<int32_t>(agent->tracking_head_ids.size());
					else if (agent->tracking_mode == TrackingMode::MultiWindowTracking)
						agent->max_assignments = 4;
					else if (agent->tracking_mode == TrackingMode::PriorityHybridTracking)
						agent->max_assignments = 4;

					// Store setting
					agents.insert({ agent->agent_id, agent });
				}
			}
			catch (const std::exception& e)
			{
				throw std::runtime_error("Error loading agent config at row " + std::to_string(row.rowNumber()) + ": " + e.what());
			}
		}

		return agents;
	}

	HeadConfigMap ConfigParser::parseAgentPayload(OpenXLSX::XLWorkbook& book, const ID& payload_id)
	{
		// Create output
		HeadConfigMap heads;

		// Open AgentHeads sheet
		auto sheet = book.worksheet("AgentHeads");

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
					return heads; // Return filled heads

				// Read foreign key and verify if the target belongs to the group selected
				std::string FK = getCellValueOrFail<std::string>(row.findCell(2));

				if (FK == payload_id)
				{
					// Create instance
					HeadConfigPtr head = std::make_shared<HeadConfig>();

					// Populate fields
					head->head_id = getCellValueOrFail<std::string>(row.findCell(1));

					head->head_payload_id = FK;

					head->head_name = getCellValueOrFail<std::string>(row.findCell(3));

					head->gimbal_model_id = getCellValueOrFail<std::string>(row.findCell(4));

					head->camera_model_id = getCellValueOrFail<std::string>(row.findCell(5));

					const std::string& head_role_str = getCellValueOrFail<std::string>(row.findCell(6));
					head->head_role = headRoleFromString(head_role_str);

					auto mount_position_str = getCellValueOrFail<std::string>(row.findCell(7));
					auto mount_position_vec = parseStringToVector<float>(mount_position_str, 3, ',');
					if (mount_position_vec.size() >= 3)
					{
						head->mount_position(0) = mount_position_vec[0];
						head->mount_position(1) = mount_position_vec[1];
						head->mount_position(2) = mount_position_vec[2];
					}

					auto mount_orientation_str = getCellValueOrFail<std::string>(row.findCell(8));
					auto mount_orientation_vec = parseStringToVector<float>(mount_orientation_str, 3, ',');
					if (mount_orientation_vec.size() >= 3)
					{
						head->mount_orientation(0) = MathUtils::degToRad(mount_orientation_vec[2]);
						head->mount_orientation(1) = MathUtils::degToRad(mount_orientation_vec[0]);
						head->mount_orientation(2) = MathUtils::degToRad(mount_orientation_vec[1]);
					}

					head->initial_focal = getCellValueOrFail<float>(row.findCell(9)) / 1000.0f;

					head->min_focal = getCellValueOrFail<float>(row.findCell(10)) / 1000.0f;

					head->max_focal = getCellValueOrFail<float>(row.findCell(11)) / 1000.0f;

					// Resolve external ID references
					head->gimbal = parseGimbalModel(book, head->gimbal_model_id);
					if (!head->gimbal)
						throw std::runtime_error("No gimbal model found for head " + head->head_id);
					head->camera = parseCameraModel(book.worksheet("CameraModels"), head->camera_model_id);
					if (!head->camera)
						throw std::runtime_error("No camera model found for head " + head->head_id);

					// Store setting
					heads.insert({ head->head_id, head });
				}
			}
			catch (const std::exception& e)
			{
				throw std::runtime_error("Error loading head config at row " + std::to_string(row.rowNumber()) + ": " + e.what());
			}
		}

		return heads;
	}

	DroneConfigPtr ConfigParser::parseDroneModel(const OpenXLSX::XLWorksheet& sheet, const ID& drone_id)
	{
		// Create output
		DroneConfigPtr drone = std::make_shared<DroneConfig>();

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
					return nullptr; // No drone model found

				// Read foreign key and verify if the target belongs to the group selected
				std::string PK = getCellValueOrFail<std::string>(row.findCell(1));

				if (PK == drone_id)
				{
					// Populate fields
					drone->drone_model_id = PK;

					drone->drone_name = getCellValueOrFail<std::string>(row.findCell(2));

					const std::string& drone_type_str = getCellValueOrFail<std::string>(row.findCell(3));
					drone->drone_type = droneTypeFromString(drone_type_str);

					drone->cruise_speed = getCellValueOrFail<float>(row.findCell(4));

					drone->max_speed = getCellValueOrFail<float>(row.findCell(5));

					drone->base_weight = getCellValueOrFail<float>(row.findCell(6));

					drone->max_payload_weight = getCellValueOrFail<float>(row.findCell(7));

					drone->hover_power = getCellValueOrFail<float>(row.findCell(8));

					drone->cruise_power = getCellValueOrFail<float>(row.findCell(9));

					drone->load_factor = getCellValueOrFail<float>(row.findCell(10));

					// Only first found drone model is loaded
					return drone;
				}
			}
			catch (const std::exception& e)
			{
				throw std::runtime_error("Error loading drone config at row " + std::to_string(row.rowNumber()) + ": " + e.what());
			}
		}

		return nullptr;
	}

	GimbalConfigPtr ConfigParser::parseGimbalModel(OpenXLSX::XLWorkbook& book, const ID& gimbal_id)
	{
		// Create output
		GimbalConfigPtr gimbal = std::make_shared<GimbalConfig>();

		// Open GimbalModels sheet
		auto sheet = book.worksheet("GimbalModels");

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
					return nullptr; // No gimbal model found

				// Read foreign key and verify if the target belongs to the group selected
				std::string PK = getCellValueOrFail<std::string>(row.findCell(1));

				if (PK == gimbal_id)
				{
					// Populate fields
					gimbal->gimbal_model_id = PK;

					gimbal->gimbal_name = getCellValueOrFail<std::string>(row.findCell(2));

					gimbal->link_configuration_id = getCellValueOrFail<std::string>(row.findCell(3));

					gimbal->weight = getCellValueOrFail<float>(row.findCell(4));

					gimbal->idle_power = getCellValueOrFail<float>(row.findCell(5));

					gimbal->active_power = getCellValueOrFail<float>(row.findCell(6));

					// Resolve external ID references
					gimbal->links = parseGimbalLinks(book.worksheet("GimbalLinks"), gimbal->link_configuration_id);
					if (gimbal->links.empty())
						throw std::runtime_error("No gimbal links found for gimbal " + gimbal->gimbal_model_id);

					// Only first found gimbal model is loaded
					return gimbal;
				}
			}
			catch (const std::exception& e)
			{
				throw std::runtime_error("Error loading gimbal config at row " + std::to_string(row.rowNumber()) + ": " + e.what());
			}
		}

		return nullptr;
	}

	GimbalLinkConfigMap ConfigParser::parseGimbalLinks(const OpenXLSX::XLWorksheet& sheet, const ID& links_id)
	{
		// Create output
		GimbalLinkConfigMap links;

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
					return links; // Return filled links

				// Read foreign key and verify if the target belongs to the group selected
				std::string FK = getCellValueOrFail<std::string>(row.findCell(2));

				if (FK == links_id)
				{
					// Create instance
					GimbalLinkConfigPtr link = std::make_shared<GimbalLinkConfig>();

					// Populate fields
					link->gimbal_link_id = getCellValueOrFail<std::string>(row.findCell(1));

					link->link_configuration_id = FK;

					const std::string& axis_type_str = getCellValueOrFail<std::string>(row.findCell(3));
					link->axis_type = axisTypeFromString(axis_type_str);

					auto joint_range_str = getCellValueOrFail<std::string>(row.findCell(4));
					auto joint_range_vec = parseStringToVector<float>(joint_range_str, 2, ',');
					if (joint_range_vec.size() >= 2)
					{
						link->joint_range(0) = MathUtils::degToRad(joint_range_vec[0]);
						link->joint_range(1) = MathUtils::degToRad(joint_range_vec[1]);
					}

					link->max_angular_speed = MathUtils::degToRad(getCellValueOrFail<float>(row.findCell(5)));

					link->motor_rise_time = getCellValueOrFail<float>(row.findCell(6));

					link->motor_damping_factor = getCellValueOrFail<float>(row.findCell(7));

					// Store setting
					links.insert({ link->gimbal_link_id, link });
				}
			}
			catch (const std::exception& e)
			{
				throw std::runtime_error("Error loading link config at row " + std::to_string(row.rowNumber()) + ": " + e.what());
			}
		}

		return links;
	}

	CameraConfigPtr ConfigParser::parseCameraModel(const OpenXLSX::XLWorksheet& sheet, const ID& camera_id)
	{
		// Create output
		CameraConfigPtr camera = std::make_shared<CameraConfig>();

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
					return nullptr; // No camera model found

				// Read foreign key and verify if the target belongs to the group selected
				std::string PK = getCellValueOrFail<std::string>(row.findCell(1));

				if (PK == camera_id)
				{
					// Populate fields
					camera->camera_model_id = PK;

					camera->camera_name = getCellValueOrFail<std::string>(row.findCell(2));

					const std::string& camera_type_str = getCellValueOrFail<std::string>(row.findCell(3));
					camera->camera_type = cameraTypeFromString(camera_type_str);

					auto resolution_str = getCellValueOrFail<std::string>(row.findCell(4));
					auto resolution_vec = parseStringToVector<int>(resolution_str, 2, 'x');
					if (resolution_vec.size() >= 2)
					{
						camera->resolution(0) = resolution_vec[0];
						camera->resolution(1) = resolution_vec[1];
					}

					camera->default_focal = getCellValueOrFail<float>(row.findCell(5)) / 1000.0f;

					camera->sensor_width = getCellValueOrFail<float>(row.findCell(6)) / 1000.0f;

					camera->sensor_height = getCellValueOrFail<float>(row.findCell(7)) / 1000.0f;

					camera->weight = getCellValueOrFail<float>(row.findCell(8));

					camera->idle_power = getCellValueOrFail<float>(row.findCell(9));

					camera->active_power = getCellValueOrFail<float>(row.findCell(10));

					// Only first found camera model is loaded
					return camera;
				}
			}
			catch (const std::exception& e)
			{
				throw std::runtime_error("Error loading camera config at row " + std::to_string(row.rowNumber()) + ": " + e.what());
			}
		}

		return nullptr;
	}

	bool ConfigParser::parseAirsimSettings(const ConfigPtr& config_ptr, const std::string& path)
	{
		// Log
		std::cout << "Parsing Airsim settings at " << path << std::endl;

		// Generate settings.json content
		nlohmann::ordered_json settings;
		populateGeneralSettings(config_ptr, settings);
		populateVehicles(config_ptr, settings);
		populateSubWindows(settings);

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

	void ConfigParser::populateGeneralSettings(const ConfigPtr& config_ptr, nlohmann::ordered_json& settings)
	{
		settings["SettingsVersion"] = 2.0;
		settings["SimMode"] = "Multirotor";
		settings["ClockType"] = "SteppableClock";
		settings["ClockSpeed"] = config_ptr->simulation->clock_speed;
		settings["ViewMode"] = "NoDisplay";
		settings["LogMessagesVisible"] = true;
		settings["ApiServerPort"] = 41451;

		/*
		settings["PawnPaths"] = {
			{"DefaultQuadrotor", {{"PawnBP", "Class'/Game/Agents/BP_DJI_S900_Quadcopter.BP_DJI_S900_Quadcopter_C'"}}},
			{"DefaultHexacopter", {{"PawnBP", "Class'/Game/Agents/BP_DJI_S900_Hexacopter.BP_DJI_S900_Hexacopter_C'"}}}};
		*/

		settings["OriginGeopoint"] = {
			{"Latitude", config_ptr->map->origin_geopoint.latitude},
			{"Longitude", config_ptr->map->origin_geopoint.longitude},
			{"Altitude", config_ptr->map->origin_geopoint.altitude} };

		// Time of day format: %Y-%m-%d %H:%M:%S
		DateTime time = config_ptr->map->start_time;
		settings["TimeOfDay"] = {
			{"Enabled", true},
			{"StartDateTime",
			 std::to_string(time.year) + "-" +
				 std::to_string(time.month) + "-" +
				 std::to_string(time.day) + " " +
				 std::to_string(time.hours) + ":" +
				 std::to_string(time.minutes) + ":" +
				 std::to_string(time.seconds)},
			{"CelestialClockSpeed", 1},
			{"StartDateTimeDst", false},
			{"UpdateIntervalSecs", 1} };

		settings["Wind"] = {
			{"X", config_ptr->map->wind_velocity.x()},
			{"Y", -config_ptr->map->wind_velocity.y()},
			{"Z", -config_ptr->map->wind_velocity.z()} };
	}

	void ConfigParser::populateVehicles(const ConfigPtr& config_ptr, nlohmann::ordered_json& settings)
	{
		nlohmann::ordered_json vehicles;

		bool first_time = true;
		int instance = 0;
		for (const auto& [agent_id, agent_ptr] : config_ptr->agents)
		{
			// Parameters
			std::string vehicle_model = (agent_ptr->drone->drone_type == DroneType::Quadcopter) ? "Quadcopter" : "Hexacopter";
			std::string pawn_path = (agent_ptr->drone->drone_type == DroneType::Quadcopter) ? "DefaultQuadrotor" : "DefaultHexacopter";
			Vector3r ini_pos = agent_ptr->initial_position;
			Vector3r ini_ori = agent_ptr->initial_orientation;

			if (config_ptr->mission->autopilot == Autopilot::PX4)
			{
				vehicles[agent_id] = {
					{"VehicleType", "PX4Multirotor"},
					//{"PawnPath", pawn_path},
					{"Model", vehicle_model},
					{"UseSerial", false},
					{"LockStep", true},
					{"UseTcp", true},
					{"TcpPort", 4560 + instance},
					{"ControlIp", "remote"},
					{"ControlPortLocal", 14540 + instance},
					{"ControlPortRemote", 14580 + instance},
					{"LocalHostIp", "172.17.0.1"},
					{"Sensors", {{"Barometer", {{"SensorType", 1}, {"Enabled", true}, {"PressureFactorSigma", 0.0001825}}}}},
					{"Parameters", {{"NAV_RCL_ACT", 0}, {"NAV_DLL_ACT", 0}, {"COM_OBL_ACT", 1}, {"LPE_LAT", config_ptr->map->origin_geopoint.latitude}, {"LPE_LON", config_ptr->map->origin_geopoint.longitude}}},
					{"X", ini_pos.x()},
					{"Y", -ini_pos.y()},
					{"Z", -ini_pos.z()},
					{"Roll", MathUtils::radToDeg(ini_ori.x())},
					{"Pitch", MathUtils::radToDeg(-ini_ori.y())},
					{"Yaw", MathUtils::radToDeg(-ini_ori.z())} };
			}
			else
			{
				vehicles[agent_id] = {
					{"VehicleType", "SimpleFlight"},
					//{"PawnPath", pawn_path},
					{"DefaultVehicleState", "Armed"},
					{"AutoCreate", true},
					{ "X", ini_pos.x() },
					{ "Y", -ini_pos.y() },
					{ "Z", -ini_pos.z() },
					{ "Roll", MathUtils::radToDeg(ini_ori.x()) },
					{ "Pitch", MathUtils::radToDeg(-ini_ori.y()) },
					{ "Yaw", MathUtils::radToDeg(-ini_ori.z()) } };
			}

			// Add cameras to the vehicle
			populateCameras(agent_id, agent_ptr->heads, vehicles[agent_id]["Cameras"]);

			// Add scene view once
			if (first_time)
			{
				populateExternalCameras(vehicles[agent_id]["Cameras"]);
				first_time = false;
			}

			// Update instance offset
			instance++;

			settings["Vehicles"] = vehicles;
		}
	}

	// Helper method: Populate cameras and gimbals
	void ConfigParser::populateCameras(const ID& agent_id, const HeadConfigMap& heads, nlohmann::ordered_json& cameras)
	{
		for (const auto& [head_id, head_ptr] : heads)
		{
			const auto& mount_pos = head_ptr->mount_position;
			const auto& mount_ori = head_ptr->mount_orientation;

			// Get camera parameters
			const auto& width = head_ptr->camera->resolution.x();
			const auto& height = head_ptr->camera->resolution.y();
			const auto& sensor_width = head_ptr->camera->sensor_width;
			const auto& sensor_height = head_ptr->camera->sensor_height;
			const auto& focal = head_ptr->initial_focal;
			const auto& fov = MathUtils::radToDeg(CameraUtils::computeFov(focal, sensor_width));

			// Get gimbal parameters
			float yaw_min = 0.0f;
			float pitch_min = 0.0f;
			float roll_min = 0.0f;
			float yaw_max = 0.0f;
			float pitch_max = 0.0f;
			float roll_max = 0.0f;
			float yaw_speed = 0.0f;
			float pitch_speed = 0.0f;
			float roll_speed = 0.0f;
			float yaw_rise_time = 0.0f;
			float pitch_rise_time = 0.0f;
			float roll_rise_time = 0.0f;
			float yaw_damping = 0.0f;
			float pitch_damping = 0.0f;
			float roll_damping = 0.0f;

			for (const auto& [link_id, link_ptr] : head_ptr->gimbal->links)
			{
				const auto& axis_type = link_ptr->axis_type;
				const auto& joint_range = link_ptr->joint_range;
				const auto& max_angular_speed = link_ptr->max_angular_speed;
				const auto& motor_rise_time = link_ptr->motor_rise_time;
				const auto& motor_damping_factor = link_ptr->motor_damping_factor;

				switch (axis_type)
				{
				case AxisType::Yaw:
					yaw_min = MathUtils::radToDeg(joint_range(0));
					yaw_max = MathUtils::radToDeg(joint_range(1));
					yaw_speed = MathUtils::degToRad(max_angular_speed);
					yaw_rise_time = motor_rise_time;
					yaw_damping = motor_damping_factor;
					break;

				case AxisType::Pitch:
					pitch_min = MathUtils::radToDeg(joint_range(0));
					pitch_max = MathUtils::radToDeg(joint_range(1));
					pitch_speed = MathUtils::degToRad(max_angular_speed);
					pitch_rise_time = motor_rise_time;
					pitch_damping = motor_damping_factor;
					break;

				case AxisType::Roll:
					roll_min = MathUtils::radToDeg(joint_range(0));
					roll_max = MathUtils::radToDeg(joint_range(1));
					roll_speed = MathUtils::degToRad(max_angular_speed);
					roll_rise_time = motor_rise_time;
					roll_damping = motor_damping_factor;
					break;
				}
			}

			cameras[head_id] = {
				{"CaptureSettings", {
					{
						{"ImageType", 0},
						{"Width", width},
						{"Height", height},
						{"FOV_Degrees", fov}
					}
				}},
				{"X", mount_pos.x()}, {"Y", -mount_pos.y()}, {"Z", -mount_pos.z()},
				{"Roll", 0.0f}, {"Pitch", MathUtils::radToDeg(-mount_ori.y())}, {"Yaw", 0.0f},
				{"EnableGimbal", true}, {"CameraVisible", true},
				{"Gimbal", {
					{"YawMin", yaw_min}, {"PitchMin", pitch_min}, {"RollMin", roll_min},
					{"YawMax", yaw_max}, {"PitchMax", pitch_max}, {"RollMax", roll_max},
					{"YawSpeed", yaw_speed}, {"PitchSpeed", pitch_speed}, {"RollSpeed", roll_speed},
					{"YawRiseTime", yaw_rise_time}, {"PitchRiseTime", pitch_rise_time}, {"RollRiseTime", roll_rise_time},
					{"YawDamping", yaw_damping}, {"PitchDamping", pitch_damping}, {"RollDamping", roll_damping},
					{"Roll", MathUtils::radToDeg(mount_ori.x())}, { "Pitch", 0.0f }, {"Yaw", MathUtils::radToDeg(-mount_ori.z())}
				}}
			};
		}

		// Get agent view camera pose (ENU frame)
		Vector3r agent_view_pos;
		agent_view_pos.x() = -1.5f;
		agent_view_pos.y() = -1.5f;
		agent_view_pos.z() = -1.5f;
		Vector3r agent_view_rot;
		agent_view_rot.x() = 0.0f;
		agent_view_rot.y() = -33.33f;
		agent_view_rot.z() = 45.0f;

		cameras["CAM_" + agent_id] = {
			{"CaptureSettings",{
				{
					{"ImageType", 0},
					{"Width", 1280},
					{"Height", 720},
					{"FOV_Degrees", 70}
				}
			}},
			{"X", agent_view_pos.x()},
			{"Y", -agent_view_pos.y()},
			{"Z", -agent_view_pos.z()},
			{"Roll", agent_view_rot.x()},
			{"Pitch", -agent_view_rot.y()},
			{"Yaw", -agent_view_rot.z()} };
	}

	void ConfigParser::populateExternalCameras(nlohmann::ordered_json& cameras)
	{
		// Get scene view camera pose (ENU frame)
		Vector3r scene_view_pos;
		scene_view_pos.x() = -75.0f;
		scene_view_pos.y() = -75.0f;
		scene_view_pos.z() = 75.0f;
		Vector3r scene_view_rot;
		scene_view_rot.x() = 0.0f;
		scene_view_rot.y() = 33.33f;
		scene_view_rot.z() = 45.0f;

		cameras["CAM_SCENE"] = {
			{"CaptureSettings",{
				{
					{"ImageType", 0},
					{"Width", 1920},
					{"Height", 1080},
					{"FOV_Degrees", 90}
				}
			}},
			{"X", scene_view_pos.x()},
			{"Y", -scene_view_pos.y()},
			{"Z", -scene_view_pos.z()},
			{"Roll", scene_view_rot.x()},
			{"Pitch", -scene_view_rot.y()},
			{"Yaw", -scene_view_rot.z()},
			{"External", true} };
	}

	void ConfigParser::populateSubWindows(nlohmann::ordered_json& settings)
	{
		int idx = 0;
		settings["SubWindows"] = nlohmann::ordered_json::array();

		// Scene view sub-window
		settings["SubWindows"].push_back({ {"WindowID", idx++},
										  {"CameraName", ""},
										  {"ImageType", 0},
										  {"VehicleName", ""},
										  {"Visible", true} });

		// Agent view sub-window
		settings["SubWindows"].push_back({ {"WindowID", idx++},
										  {"CameraName", ""},
										  {"ImageType", 0},
										  {"VehicleName", ""},
										  {"Visible", true} });

		// Map view sub-window
		settings["SubWindows"].push_back({ {"WindowID", idx++},
										  {"CameraName", ""},
										  {"ImageType", 0},
										  {"VehicleName", ""},
										  {"Visible", true} });

		// Telemetry view sub-window
		settings["SubWindows"].push_back({ {"WindowID", idx++},
										  {"CameraName", ""},
										  {"ImageType", 0},
										  {"VehicleName", ""},
										  {"Visible", true} });

		// Camera sub-windows
		for (int i = 0; i < 5; i++)
		{
			settings["SubWindows"].push_back({ {"WindowID", idx++},
											  {"CameraName", ""},
											  {"ImageType", 0},
											  {"VehicleName", ""},
											  {"Visible", true} });
		}
	}

} // namespace flychams::config