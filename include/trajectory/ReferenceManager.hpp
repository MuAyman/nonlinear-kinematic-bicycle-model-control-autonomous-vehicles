#pragma once
#include <vector>
#include <cmath>
#include "PathGenerator.hpp"

class ReferenceManager
{
public:
    // Constructor with path parameters
    ReferenceManager(double pathLength = 100.0, double pointsSpacing = 0.5, double wheelbase = 2.5)
        : path_gen_(pathLength, pointsSpacing), wheelbase_(wheelbase), delta_s_(pointsSpacing) // default delta_s_ increment
    {};

    // Calculate state errors in global frame to a reference point at lookahead_distance ahead (default = 0.0) on the path
    states calculateErrorGlobalFrame(const states &current_state, double lookahead_distance = 0.0)
    {
        states error;
        if (lookahead_distance > 0.0)
            reference_point_globalframe = path_gen_.evaluate(s_ + lookahead_distance);
        else
            reference_point_globalframe = path_gen_.evaluate(s_);

        // Compute position error in global frame
        error.x = reference_point_globalframe.x - current_state.x; // longitudinal error
        error.y = reference_point_globalframe.y - current_state.y; // lateral error

        // Heading error
        error.heading = reference_point_globalframe.heading - current_state.heading;
        // Normalize heading error to [-pi, pi]
        error.heading = atan2(sin(error.heading), cos(error.heading));

        // Steering angle error
        double steering_ref = atan(reference_point_globalframe.curvature * wheelbase_);
        error.steeringAngle = steering_ref - current_state.steeringAngle;

        // Advance s along the path
        s_ += delta_s_;

        return error;
    };

    // Transform errors from global frame to path frame
    void errorGlobaltoPathFrame(states &state_errors_global, const states &current_state) const
    {
        // Extract global errors
        double dx = state_errors_global.x;
        double dy = state_errors_global.y;

        // Transform to path frame
        double heading_ref = reference_point_globalframe.heading;
        state_errors_global.x = cos(heading_ref) * dx + sin(heading_ref) * dy;  // longitudinal error
        state_errors_global.y = -sin(heading_ref) * dx + cos(heading_ref) * dy; // lateral error
    }

    // Transform errors from global frame to vehicle frame
    void errorGlobaltoVehicleFrame(states &state_errors, const states &current_state) const
    {
        // Extract global errors
        double dx = state_errors.x;
        double dy = state_errors.y;

        // Transform to vehicle frame
        double heading_vehicle = current_state.heading;
        state_errors.x = cos(heading_vehicle) * dx + sin(heading_vehicle) * dy;  // longitudinal error
        state_errors.y = -sin(heading_vehicle) * dx + cos(heading_vehicle) * dy; // lateral error
    }

    // // Calculate the x,y error in vehicle frame to a lookahead point ahead on the path
    // Waypoint lookaheadPointErrorGlobalFrame(double lookahead_distance, const states &current_state)
    // {
    //     PathPoint lookahead_point_globalframe = path_gen_.evaluate(s_ + lookahead_distance);

    //     // Compute position error in global frame
    //     double dx = lookahead_point_globalframe.x - current_state.x;
    //     double dy = lookahead_point_globalframe.y - current_state.y;

    //     // Transform to vehicle frame
    //     // errorGlobaltoVehicleFrame({dx,dy,dx,dy}, current_state)
    //     double x = cos(current_state.heading) * dx + sin(current_state.heading) * dy;
    //     double y = -sin(current_state.heading) * dx + cos(current_state.heading) * dy;

    //     // Advance s along the path
    //     s_ += delta_s_;

    //     return {x, y};
    // };

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

private:
    double s_ = 0.0;                       // progress along the path
    double delta_s_;                       // increment in s for each step
    double wheelbase_;                     // vehicle wheelbase in meters
    PathGenerator path_gen_;               // instance of PathGenerator
    PathPoint reference_point_globalframe; // reference point in global frame
};