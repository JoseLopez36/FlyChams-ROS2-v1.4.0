#pragma once

// Standard includes
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <iostream>
#include <memory>

namespace flychams::targets
{
    /**
     * ════════════════════════════════════════════════════════════════
     * @brief Trajectory parser
     *
     * @details
     * This class implements a trajectory parser.
     * It provides methods for parsing trajectories from a file.
     *
     * ════════════════════════════════════════════════════════════════
     * @author Jose Francisco Lopez Ruiz
     * @date 2025-01-29
     * ════════════════════════════════════════════════════════════════
     */
    class TrajectoryParser
    {
    public: // Types
        struct Point
        {
            float t;
            float x, y, z;
        };

    public: // Methods
        static std::vector<Point> parse(const std::string& path)
        {
            // Initialize output data
            std::vector<Point> points;

            // Get and open trajectory file
            std::ifstream file(path);
            if (!file.is_open())
            {
                std::cerr << "Failed to open trajectory file: " << path << std::endl;
                return points;
            }

            // Read trajectory file
            std::string line;
            while (std::getline(file, line))
            {
                std::stringstream ss(line);
                std::string value;
                Point point;

                // Read time
                if (std::getline(ss, value, '\t'))
                    point.t = std::stod(value);

                // Read x
                if (std::getline(ss, value, '\t'))
                    point.x = std::stod(value);

                // Read y
                if (std::getline(ss, value, '\t'))
                    point.y = std::stod(value);

                // Read z
                if (std::getline(ss, value, '\t'))
                    point.z = std::stod(value);

                // Add point to trajectory
                points.push_back(point);
            }

            // Close file
            file.close();

            // Return points
            return points;
        }
    };

} // namespace flychams::targets