#pragma once
#include "PathGenerator.hpp"

class ReferenceManager
{
public:
    // Constructor with provided waypoints
    ReferenceManager(const std::vector<Waypoint> &input_waypoints,
                     double delta_s = 0.5)
        : path_gen_(input_waypoints), delta_s_(delta_s) {};

    // Calculate state errors in global frame to a reference point at lookahead_distance ahead (default = 0.0) on the path
    states calculateErrorGlobalFrame(const states &current_state,
                                     double lookahead_distance = 0.0)
    {
        states error;
        reference_point_globalframe = path_gen_.evaluate(s_);

        if (lookahead_distance > 0.0)
        {
            double ex = reference_point_globalframe.x - current_state.x;
            double ey = reference_point_globalframe.y - current_state.y;
            double distance_to_target = sqrt(ex * ex + ey * ey);
            double s_temp = s_; // temp s to increment to get ref_point at Ld form vehicle
            while (distance_to_target < lookahead_distance && !isPathCompleted())
            {
                s_temp += delta_s_; // increment s to find the lookahead point
                reference_point_globalframe = path_gen_.evaluate(s_temp);
                ex = reference_point_globalframe.x - current_state.x;
                ey = reference_point_globalframe.y - current_state.y;
                distance_to_target = sqrt(ex * ex + ey * ey);
            }
        }

        // Compute position error in global frame
        error.x = reference_point_globalframe.x - current_state.x; // longitudinal error
        error.y = reference_point_globalframe.y - current_state.y; // lateral error

        // Heading error
        error.heading = reference_point_globalframe.heading - current_state.heading;
        // Normalize heading error to [-pi, pi]
        error.heading = atan2(sin(error.heading), cos(error.heading));

        // Steering angle error
        double steering_ref = atan(reference_point_globalframe.curvature * specs.wheelbase);
        error.steeringAngle = steering_ref - current_state.steeringAngle;

        return error;
    };

    // Transform errors from global frame to path frame
    states errorGlobaltoPathFrame(const states &state_errors_global) const
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
    states errorGlobaltoVehicleFrame(const states &state_errors_global, const states &current_state) const
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

    // Get reference speed at current s_ + lookahead_distance (optional)
    // double getReferenceSpeed(double lookahead_distance = 0.0) const
    // {
    //     return path_gen_.speedAt(s_ + lookahead_distance);
    // }

    // Update path progress by finding s_closest to the current vehicle position
    void updateProgress(const states &current_state, const states &prev_state, double globalHeading)
    {
        // Calculate actual distance moved in global space
        double dx = current_state.x - prev_state.x;
        double dy = current_state.y - prev_state.y;
        double actual_ds = sqrt(dx * dx + dy * dy);

        // Rotate the displacement vector to the path frame and extract longitudinal component
        double longitudinal_ds = cos(globalHeading) * dx + sin(globalHeading) * dy;

        s_ += longitudinal_ds; // increment s_ by ds

        // Clamp s_ to valid range
        s_ = std::clamp(s_, 0.0, path_gen_.getPathLength());
    }

    // Find closest point on path to current vehicle position
    // void updateProgress(const states &current_state)
    // {
    //     // Define search window: backwards and forwards
    //     double s_window = 10.0; // meters
    //     double s_start = std::max(0.0, s_ - s_window);
    //     double s_end = std::min(path_gen_.getPathLength(), s_ + s_window);

    //     // Search for closest point in [s_start, s_end]
    //     double s_closest = s_;
    //     PathPoint current_point = path_gen_.evaluate(s_);
    //     double dx = current_point.x - current_state.x;
    //     double dy = current_point.y - current_state.y;
    //     double s_distance_min = sqrt(dx * dx + dy * dy);

    //     for (double s = s_start; s <= s_end; s += delta_s_)
    //     {
    //         PathPoint point = path_gen_.evaluate(s);
    //         dx = point.x - current_state.x;
    //         dy = point.y - current_state.y;
    //         double distance = sqrt(dx * dx + dy * dy);
    //         if (distance < s_distance_min)
    //         {
    //             s_distance_min = distance;
    //             s_closest = s;
    //         }
    //     }
    //     // Update s_ to the closest point, but only allow progress to increase
    //     s_ = std::max(s_, s_closest);
    // }

    // Check if the path is completed
    bool isPathCompleted() const
    {
        return s_ >= (path_gen_.getPathLength());
    }

    double getPathReminingDistance() const
    {
        return (path_gen_.getPathLength() - s_);
    }

    double getPathLength() const
    {
        return path_gen_.getPathLength();
    }

    double getS() const
    {
        return s_;
    }

private:
    double s_ = 0.0;                       // progress along the path
    double delta_s_ = 0.5;                 // increment in s for each step
    double s_window = 5.0;                 // search window for closest point
    vehicleSpecs specs;                    // vehicle specs
    PathGenerator path_gen_;               // instance of PathGenerator
    PathPoint reference_point_globalframe; // reference point in global frame
};