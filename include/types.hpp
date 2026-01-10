#pragma once

struct states {
    double x;
    double y;
    double heading;
    double steeringAngle;
};

struct inputs {
    double velocity;
    double steeringRate;
};

// Struct to store waypoints x,y in meters
struct Waypoint {
    double x;         // Global x position [m]
    double y;         // Global y position [m]
};

/*
    PathPoint
    ---------
    A single evaluated point on the continuous path.
    All quantities are purely geometric.
*/
struct PathPoint : public Waypoint {
    double heading;       // Path heading ψ(s) [rad]
    double curvature; // Path curvature κ(s) [1/m]
};
