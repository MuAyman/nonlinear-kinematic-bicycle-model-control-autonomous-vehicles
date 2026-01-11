#pragma once
#include <vector>
#include <cmath>
#include "PathGenerator.hpp"

class ReferenceManager
{

public:
    // ReferenceManager() = default;

    // Constructor with path parameters
    ReferenceManager(double pathLength = 100.0, double pointsSpacing = 0.5, double wheelbase = 2.5)
        : path_gen_(pathLength, pointsSpacing), wheelbase_(wheelbase), delta_s_(pointsSpacing) // default delta_s_ increment
    {};

    // Calculate the error between current state and reference point in path frame
    states calculateErrorPathFrame(const states &current_state)
    {
        states error;
        reference_point_globalframe = path_gen_.evaluate(s_);

        // Compute position error in global frame
        double dx = reference_point_globalframe.x - current_state.x;
        double dy = reference_point_globalframe.y - current_state.y;

        // Transform to path frame
        double heading_ref = reference_point_globalframe.heading;
        error.x = cos(heading_ref) * dx + sin(heading_ref) * dy;  // longitudinal error
        error.y = -sin(heading_ref) * dx + cos(heading_ref) * dy; // lateral error

        // Heading error
        error.heading = heading_ref - current_state.heading;
        // Normalize heading error to [-pi, pi]
        error.heading = atan2(sin(error.heading), cos(error.heading));

        // Steering angle error
        double steering_ref = atan(reference_point_globalframe.curvature * wheelbase_);
        error.steeringAngle = steering_ref - current_state.steeringAngle;

        // Advance s along the path
        s_ += delta_s_;

        return error;
    };

    // Calculate the error between current state and reference point in global frame
    states calculateErrorGlobalFrame(const states &current_state)
    {
        states error;
        reference_point_globalframe = path_gen_.evaluate(s_);

        // Compute position error in global frame
        error.x = reference_point_globalframe.x - current_state.x;
        error.y = reference_point_globalframe.y - current_state.y;

        // Heading error
        double heading_ref = reference_point_globalframe.heading;
        error.heading = heading_ref - current_state.heading;
        // Normalize heading error to [-pi, pi]
        error.heading = atan2(sin(error.heading), cos(error.heading));

        // Steering angle error
        double steering_ref = atan(reference_point_globalframe.curvature * wheelbase_);
        error.steeringAngle = steering_ref - current_state.steeringAngle;

        // Advance s along the path
        s_ += delta_s_;

        return error;
    };

    PathPoint getReferencePointGlobalframe() const
    {
        return reference_point_globalframe;
    }

    // Get the lookahead point at a specified distance ahead on the path
    PathPoint getLookaheadPoint(double lookahead_distance) const
    {
        // Evaluate point at distance s_ + lookahead_distance ahead on path
        return path_gen_.evaluate(s_ + lookahead_distance);
    }

    // Advance path progress
    void advancePathProgress()
    {
        s_ += delta_s_;
    }

private:
    double s_ = 0.0;                       // progress along the path
    double delta_s_;                       // increment in s for each step
    double wheelbase_;                     // vehicle wheelbase in meters
    PathGenerator path_gen_;               // instance of PathGenerator
    PathPoint reference_point_globalframe; // reference point in global frame
};