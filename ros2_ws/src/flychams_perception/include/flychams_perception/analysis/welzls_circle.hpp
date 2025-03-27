#pragma once

// Standard includes
#include <random>
#include <vector>
#include <cassert>
#include <cmath>
#include <algorithm>

namespace flychams::perception
{
    /**
     * ════════════════════════════════════════════════════════════════
     * @brief Welzl's circle implementation
     *
     * @details
     * This class implements Welzl's circle algorithm.
     * It provides methods for calculating the minimum enclosing circle.
     * (based on https://www.geeksforgeeks.org/minimum-enclosing-circle-using-welzls-algorithm/)
     *
     * ════════════════════════════════════════════════════════════════
     * @author Jose Francisco Lopez Ruiz
     * @date 2025-03-27
     * ════════════════════════════════════════════════════════════════
     */
    class WelzlsCircle
    {
    public: // Types
        struct Point2D
        {
            float x, y;
        };
        struct Circle
        {
            Point2D C;
            float R;
        };

    public: // Public methods
        static Circle welzl(const std::vector<Point2D>& P_input)
        {
            // Make a copy of P to allow modifications
            std::vector<Point2D> P = P_input;

            // Shuffle the points randomly to ensure random order
            static std::mt19937 rng(std::random_device{}());
            std::shuffle(P.begin(), P.end(), rng);

            // Call the recursive helper function with an empty boundary set
            std::vector<Point2D> R;
            return welzlHelper(P, R, static_cast<int>(P.size()));
        }

    private: // Implementation
        // Euclidean distance between two Point2D
        static float distance(const Point2D& a, const Point2D& b)
        {
            return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2));
        }

        // Check whether a Point2D lies inside or on the boundaries of the circle
        static bool isInside(const Circle& c, const Point2D& p)
        {
            return distance(c.C, p) <= c.R;
        }

        // Get a circle defined by 3 Point2D
        static Point2D getCircleCenter(float bx, float by, float cx, float cy)
        {
            float B = bx * bx + by * by;
            float C = cx * cx + cy * cy;
            float D = bx * cy - by * cx;
            return { (cy * B - by * C) / (2.0f * D), (bx * C - cx * B) / (2.0f * D) };
        }

        // Return a unique circle that intersects three Point2D
        static Circle circleFrom(const Point2D& A, const Point2D& B, const Point2D& C)
        {
            Point2D I = getCircleCenter(B.x - A.x, B.y - A.y, C.x - A.x, C.y - A.y);
            I.x += A.x;
            I.y += A.y;
            return { I, distance(I, A) };
        }

        // Return the smallest circle that intersects two Point2D
        static Circle circleFrom(const Point2D& A, const Point2D& B)
        {
            // Set the center to be the midpoint of A and B
            Point2D C = { (A.x + B.x) / 2.0f, (A.y + B.y) / 2.0f };

            // Set the radius to be half the distance AB
            return { C, distance(A, B) / 2.0f };
        }

        // Check whether a circle encloses the given Point2D points
        static bool isValidCircle(const Circle& c, const std::vector<Point2D>& P)
        {
            // Iterate through all the points to check whether they lie inside the circle
            for (const Point2D& p : P)
                if (!isInside(c, p))
                    return false;
            return true;
        }

        // Return the minimum enclosing circle for N <= 3
        static Circle minCircleTrivial(std::vector<Point2D>& P)
        {
            assert(P.size() <= 3);
            if (P.empty())
            {
                return { {0.0f, 0.0f}, 0.0f };
            }
            else if (P.size() == 1)
            {
                return { P[0], 0.0f };
            }
            else if (P.size() == 2)
            {
                return circleFrom(P[0], P[1]);
            }

            // To check if MEC can be determined by 2 points only
            for (int i = 0; i < 3; i++)
            {
                for (int j = i + 1; j < 3; j++)
                {
                    Circle c = circleFrom(P[i], P[j]);
                    if (isValidCircle(c, P))
                        return c;
                }
            }
            return circleFrom(P[0], P[1], P[2]);
        }

        // Returns the MEC using Welzl's algorithm
        // Takes a set of input Point2D P and a set R of points on the circle boundary.
        // n represents the number of Point2D in P that are not yet processed.
        static Circle welzlHelper(std::vector<Point2D>& P, std::vector<Point2D> R, int n)
        {
            // Base case when all points are processed or |R| = 3
            if (n == 0 || R.size() == 3)
                return minCircleTrivial(R);

            // Select the last point
            Point2D p = P[n - 1];

            // Find the MEC without the last point
            Circle c = welzlHelper(P, R, n - 1);

            // If p is inside the MEC, return it
            if (isInside(c, p))
                return c;

            // Otherwise, p must be on the boundary of the MEC
            R.push_back(p);

            // Return the MEC for P - {p} and R U {p}
            return welzlHelper(P, R, n - 1);
        }
    };

} // namespace flychams::perception