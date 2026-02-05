#pragma once
#include <cmath>
#include "../types.hpp"

class StanleyController
{
public:
    // Constructor
    StanleyController(double k = 1.0, double wheelbase = 2.5) : k_(k), wheelbase_(wheelbase) {}

    // Compute the desired steering angle using Stanley controller
    inputs computeControlInput(const states &current_state,
                               const inputs &input,
                               const states &ErrorGlobalFrame)
    {
        inputs control_inputs;

        // Extract errors
        double lateral_error = ErrorGlobalFrame.y; // lateral error in path frame
        double heading_error = ErrorGlobalFrame.heading; // heading error

        // Stanley formula: steering_angle = heading_error + atan(k * e_y / v)
        double velocity = std::max(input.velocity, 0.1); // avoid division by zero
        double steering_angle = heading_error + atan(k_ * lateral_error / velocity);

        // Normalize steering angle to [-pi, pi]
        steering_angle = atan2(sin(steering_angle), cos(steering_angle));

        // Compute steering rate as P control on steering angle error
        double error_steering = steering_angle - current_state.steeringAngle;
        error_steering = atan2(sin(error_steering), cos(error_steering));
        control_inputs.steeringRate = kp_steering * error_steering;

        // Velocity control (simple)
        control_inputs.velocity = limits.max_velocity * cos(heading_error);

        return control_inputs;
    }

private:
    double k_; // Stanley gain
    double wheelbase_;
    double kp_steering = 20.0;
    vehicleLimits limits;
};