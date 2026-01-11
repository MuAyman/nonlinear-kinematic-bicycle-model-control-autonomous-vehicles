#pragma once

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
