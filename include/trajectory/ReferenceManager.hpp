#pragma once
#include <vector>
#include <cmath>
#include "PathGenerator.hpp"

class ReferenceManager
{

public:
    ReferenceManager() = default;

    // Constructor with path parameters
    ReferenceManager(double pathLength = 100.0, double pointsSpacing = 0.5)
        : path_gen_(pathLength, pointsSpacing) {};

    // Calculate the error between current state and reference point in path frame
    states calculateError(const states &current_state, double s, double wheelbase)
    {
        states error;
        s_ = s;
        reference_point_globalframe = path_gen_.evaluate(s);

        // Position error in body frame
        double dx = reference_point_globalframe.x - current_state.x;
        double dy = reference_point_globalframe.y - current_state.y;

        double heading_ref = reference_point_globalframe.heading;
        error.x = cos(heading_ref) * dx + sin(heading_ref) * dy;  // longitudinal error
        error.y = -sin(heading_ref) * dx + cos(heading_ref) * dy; // lateral error

        // Heading error
        error.heading = heading_ref - current_state.heading;
        // Normalize heading error to [-pi, pi]
        error.heading = atan2(sin(error.heading), cos(error.heading));

        // Steering angle error
        double steering_ref = atan(reference_point_globalframe.curvature * wheelbase);
        error.steeringAngle = steering_ref - current_state.steeringAngle;

        return error;
    };

private:
    double s_ = 0.0;                       // progress along the path
    PathGenerator path_gen_;               // instance of PathGenerator
    PathPoint reference_point_globalframe; // reference point in global frame
};