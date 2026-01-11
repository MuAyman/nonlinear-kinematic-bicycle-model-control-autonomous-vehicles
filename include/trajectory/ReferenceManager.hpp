#pragma once
#include <vector>
#include <cmath>
#include "PathGenerator.hpp"

class ReferenceManager
{
public:
    // Constructor with path parameters
    ReferenceManager(double pathLength = 100.0, double pointsSpacing = 0.5, double wheelbase = 2.5, double delta_s = 0.5)
        : path_gen_(pathLength, pointsSpacing), wheelbase_(wheelbase), delta_s_(delta_s) // default delta_s_ increment
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

        return error;
    };

    // Transform errors from global frame to path frame
    states errorGlobaltoPathFrame(states &state_errors_global, const states &current_state) const
    {
        states state_errors_path = state_errors_global;

        // Extract global errors
        double dx = state_errors_global.x;
        double dy = state_errors_global.y;

        // Transform to path frame
        double heading_ref = reference_point_globalframe.heading;
        state_errors_path.x = cos(heading_ref) * dx + sin(heading_ref) * dy;  // longitudinal error
        state_errors_path.y = -sin(heading_ref) * dx + cos(heading_ref) * dy; // lateral error

        return state_errors_path;
    }

    // Transform errors from global frame to vehicle frame
    states errorGlobaltoVehicleFrame(states &state_errors_global, const states &current_state) const
    {
        states state_errors_vehicle = state_errors_global;
        // Extract global errors
        double dx = state_errors_global.x;
        double dy = state_errors_global.y;

        // Transform to vehicle frame
        double heading_vehicle = current_state.heading;
        state_errors_vehicle.x = cos(heading_vehicle) * dx + sin(heading_vehicle) * dy;  // longitudinal error
        state_errors_vehicle.y = -sin(heading_vehicle) * dx + cos(heading_vehicle) * dy; // lateral error

        return state_errors_vehicle;
    }

    // Get the current reference point in global frame
    PathPoint getReferencePointGlobalframe() const
    {
        return reference_point_globalframe;
    }
    
    // Get point at distance s_ + lookahead_distance ahead on path 
    PathPoint getLookaheadPoint(double lookahead_distance) const
    {
        return path_gen_.evaluate(s_ + lookahead_distance);
    }
    
    // Adaptive update of progress along the path based on current velocity
    void updateProgress(double delta_s)
    {
        s_ += delta_s;
        // Clamp s_ to the path length
        s_ = std::clamp(s_, 0.0, path_gen_.getPathLength());
    }

    // Check if the path is completed
    bool isPathCompleted() const
    {
        // usign a margin to avoid floating point issues
        return s_ >= (path_gen_.getPathLength()); // margin of 1 meter
    }

    double getPathLength() const
    {
        return path_gen_.getPathLength();
    }

private:
    double s_ = 0.0;                       // progress along the path
    double delta_s_;                       // increment in s for each step
    double wheelbase_;                     // vehicle wheelbase in meters
    PathGenerator path_gen_;               // instance of PathGenerator
    PathPoint reference_point_globalframe; // reference point in global frame
};