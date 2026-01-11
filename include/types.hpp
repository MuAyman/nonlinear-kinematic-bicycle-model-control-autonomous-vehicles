#pragma once
#include <cmath>

// Struct to store the vehicle states
struct states
{
    double x;             // Global x position [m]
    double y;             // Global y position [m]
    double heading;       // psi in radians
    double steeringAngle; // delta in radians
};

// Struct to store the vehicle control inputs
struct inputs
{
    double velocity;     // m/s
    double steeringRate; // rad/s
};

struct limits
{
    // Define limits - physical and comfort constraints
    const double max_velocity = 20.0;            // Maximum velocity in m/s
    const double min_velocity = -5.0;            // Minimum velocity in m/s (reverse)
    const double max_acceleration = 3.0;         // Maximum acceleration in m/s²
    const double max_steering_rate = 0.5;        // Maximum steering rate in rad/s
    const double min_steering_rate = -0.5;       // Minimum steering rate in rad/s
    const double max_steering_angle = M_PI / 4;  // Maximum steering angle 45 degrees in radians
    const double min_steering_angle = -M_PI / 4; // Minimum steering angle -45 degrees in radians
    const double abs_min_velocity = 0.1;         // Minimum absolute velocity m/s to avoid division by zero
    const double abs_min_steering_rate = 0.01;   // Minimum absolute steering rate in rad/s to avoid division by zero
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
