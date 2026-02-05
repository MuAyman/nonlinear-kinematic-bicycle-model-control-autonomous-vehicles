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
        double ex = reference_point_globalframe.x - current_state.x;
        double ey = reference_point_globalframe.y - current_state.y;

        if (lookahead_distance > 0.0)
        {
            double distance_to_target = sqrt(ex * ex + ey * ey);
            double s_temp = s_; // temp s to increment to get ref_point at Ld form vehicle
            while (distance_to_target < lookahead_distance)
            {
                s_temp += delta_s_; // increment s to find the lookahead point
                if (s_temp >= getPathLength())
                {
                    lookaheadPoint = path_gen_.evaluate(getPathLength());
                    break;
                }
                lookaheadPoint = path_gen_.evaluate(s_temp);
                ex = lookaheadPoint.x - current_state.x;
                ey = lookaheadPoint.y - current_state.y;
                distance_to_target = sqrt(ex * ex + ey * ey);
            }
            // Compute position error in global frame
            error.x = ex; // longitudinal error
            error.y = ey; // lateral error

            // Heading error normalized to [-pi, pi]
            error.heading = lookaheadPoint.heading - current_state.heading;
            error.heading = atan2(sin(error.heading), cos(error.heading));

            // Steering angle error
            double steering_ref = atan(lookaheadPoint.curvature * specs.wheelbase);
            error.steeringAngle = steering_ref - current_state.steeringAngle;

            return error;
        }
        else
        {
            // Compute position error in global frame
            error.x = ex; // longitudinal error
            error.y = ey; // lateral error

            // Heading error normalized to [-pi, pi]
            error.heading = reference_point_globalframe.heading - current_state.heading;
            error.heading = atan2(sin(error.heading), cos(error.heading));

            // Steering angle error
            double steering_ref = atan(reference_point_globalframe.curvature * specs.wheelbase);
            error.steeringAngle = steering_ref - current_state.steeringAngle;

            return error;
        }
    }

    // Get the current reference point in global frame (no Ld)
    PathPoint getReferencePointGlobalframe() const
    {
        return path_gen_.evaluate(s_);
    }

    // Get point at a lookahead_distance form the vehicle on path
    PathPoint getLookaheadPoint() const
    {
        return lookaheadPoint;
    }

    // Update path progress by the actual distance projected onto the path direction
    void updateProgress(const states &current_state, const states &prev_state)
    {
        // Calculate actual distance moved in global space
        double dx = current_state.x - prev_state.x;
        double dy = current_state.y - prev_state.y;
        double ref_heading = path_gen_.evaluate(s_).heading;
        // Rotate the displacement vector to the path frame and extract longitudinal component
        double longitudinal_ds = cos(ref_heading) * dx + sin(ref_heading) * dy;

        // increment s_ only by postive longitudinal progress
        if (longitudinal_ds > 0.0)
            s_ += longitudinal_ds; // increment s_ by ds

        // Clamp s_ to valid range
        s_ = std::clamp(s_, 0.0, path_gen_.getPathLength());
    }

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
    double s_ = 0.0;                                          // progress along the path
    double delta_s_ = 0.5;                                    // increment in s for each step
    double s_window = 5.0;                                    // search window for closest point
    vehicleSpecs specs;                                       // vehicle specs
    PathGenerator path_gen_;                                  // instance of PathGenerator
    PathPoint reference_point_globalframe;                    // reference point in global frame
    PathPoint lookaheadPoint; // Lookahead point (initialized)
};