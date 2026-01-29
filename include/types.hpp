#pragma once
#include <cmath>
#include <fstream>
#include <sstream>
#include <vector>

// Struct to store the vehicle states
struct states
{
    double x = 0.0;             // Global x position [m]
    double y = 0.0;             // Global y position [m]
    double heading = 0.0;       // psi in radians
    double steeringAngle = 0.0; // delta in radians
};

// Struct to store the vehicle control inputs
struct inputs
{
    double velocity = 0.0;     // m/s
    double steeringRate = 0.0; // rad/s
};

struct vehicleSpecs
{
    double wheelbase = 2.5; // meters
    double dt = 0.05;       // seconds
};

struct vehicleLimits
{
    // Define limits - physical and comfort constraints
    const double max_velocity = 15.0;            // Maximum velocity in m/s
    const double min_velocity = -5.0;            // Minimum velocity in m/s (reverse)
    const double max_acceleration = 3.0;         // Maximum acceleration in m/s²
    const double max_steering_rate = 0.5;        // Maximum steering rate in rad/s
    const double min_steering_rate = -0.5;       // Minimum steering rate in rad/s
    const double max_steering_angle = M_PI / 4;  // Maximum steering angle 45 degrees in radians
    const double min_steering_angle = -M_PI / 4; // Minimum steering angle -45 degrees in radians
    const double abs_min_velocity = 0.1;         // Minimum absolute velocity m/s to avoid division by zero
    const double abs_min_steering_rate = 0.01;   // Minimum absolute steering rate in rad/s to avoid division by zero
    const double a_lat_max = 3.0;                // Maximum centripetal acceleration 3.0 m/s2
    const double a_long_max = 4.0;               // Maximum longitudinal acceleration 2.0 m/s2
};

// Struct to store waypoints x,y in meters
struct Waypoint
{
    double x; // Global x position [m]
    double y; // Global y position [m]
};

// Struct to store path points x,y with heading and curvature
struct PathPoint : public Waypoint
{
    double heading;   // Path heading ψ(s) [rad]
    double curvature; // Path curvature κ(s) [1/m]
};

// Struct to hold simulation performance metrics
struct SimulationMetrics
{
    double total_squared_error = 0.0;
    double max_error = 0.0;
    int step_count = 0;
};

// ========================================================================
// =========================== HELPER FUNCTIONS ===========================
// ========================================================================

// Load waypoints from a CSV file
std::vector<Waypoint> loadWaypointsFromCSV(const std::string &filepath)
{
    std::ifstream waypoint_file(filepath);
    std::vector<Waypoint> waypoints;

    if (!waypoint_file.is_open())
        return waypoints;

    std::string line;
    std::getline(waypoint_file, line); // skip header

    while (std::getline(waypoint_file, line))
    {
        std::stringstream ss(line);
        std::string x_str, y_str;

        std::getline(ss, x_str, ',');
        std::getline(ss, y_str, ',');

        waypoints.push_back({std::stod(x_str), std::stod(y_str)});
    }

    return waypoints;
}

// Update simulation performance metrics
void updateMetrics(SimulationMetrics &metrics, const states &current, double ref_x, double ref_y)
{
    // 1. Calculate Euclidean distance between Vehicle and Target
    double dx = ref_x - current.x;
    double dy = ref_y - current.y;
    double distance_error = std::sqrt(dx * dx + dy * dy);

    // 2. Accumulate Squared Error
    metrics.total_squared_error += (distance_error * distance_error);

    // 3. Track Max Error
    if (distance_error > metrics.max_error)
    {
        metrics.max_error = distance_error;
    }

    metrics.step_count++;
}

// Print simulation performance metrics
void printMetrics(const SimulationMetrics &metrics)
{
    // Prevent division by zero if step_count is 0
    if (metrics.step_count == 0)
        return;

    double rmse = std::sqrt(metrics.total_squared_error / metrics.step_count);

    std::cout << "------------------------------------------" << std::endl;
    std::cout << "Path Following Performance Metrics:" << std::endl;
    std::cout << "Total Steps: " << metrics.step_count << std::endl;
    std::cout << "Maximum Error Recorded: " << metrics.max_error << " meters" << std::endl;
    std::cout << "Root Mean Square Error (RMSE): " << rmse << " meters" << std::endl;
    std::cout << "------------------------------------------" << std::endl;
}

// Save simulation step data to CSV
void save_simulation_step(std::ofstream& file, double time, double ref_x, double ref_y, 
                          const states& current, const inputs& input) 
{
    file << time 
         << ", " << ref_x 
         << ", " << ref_y 
         << ", " << current.x 
         << ", " << current.y 
         << ", " << current.heading 
         << ", " << current.steeringAngle 
         << ", " << input.velocity 
         << ", " << input.steeringRate 
         << "\n"; // Using \n is slightly faster than endl for file I/O
}