#pragma once
#include "../types.hpp"

class PurePursuitController
{
public:
    // Constructor to initialize lookahead distance
    PurePursuitController() = default;

    // Compute the desired steering angle using Pure Pursuit algorithm
    double computeControlInput(const states &current_state,
                               const states &ErrorGlobalFrame) const
    {
        // inputs control_inputs;
        double steeringRate;
        // Extract position errors
        double ex = ErrorGlobalFrame.x;
        double ey = ErrorGlobalFrame.y;

        // Compute distance to target point
        double dist_to_target = sqrt(ex * ex + ey * ey);

        // Prevent division by zero. 10 cm threshold
        if (dist_to_target < 0.1)
            steeringRate = 0.0; // Prevent division by zero
        else
        { // Compute steering command
            double heading = atan2(ey, ex);
            double alpha = heading - current_state.heading; // angle between vehicle heading and line to target
            alpha = atan2(sin(alpha), cos(alpha));          // Normalize to [-pi, pi]
            double curvature = 2 * sin(alpha) / dist_to_target;
            double steering_angle = atan(specs.wheelbase * curvature);
            double error_steering = steering_angle - current_state.steeringAngle;
            steeringRate = std::clamp(kp_steering * error_steering,
                                      -limits.max_steering_rate,
                                      limits.max_steering_rate);
        }
        return steeringRate;
    }

    // Adaptive lookahead distance based on velocity
    double getLd() const
    {
        return Ld;
    }

    // Add this to your PurePursuitController class
    double getAdaptiveLd(double velocity) const
    {
        return std::max(min_Ld, k_v * velocity);
    }

private:
    double Ld = 3.5;
    double min_Ld = 2.5;
    double k_v = 1.8;         // Lookahead gain
    double kp_steering = 5.0; // Reduced from 20.0 to reduce overshoot

    vehicleLimits limits; // vehicle limits
    vehicleSpecs specs;   // vehicle specs
};