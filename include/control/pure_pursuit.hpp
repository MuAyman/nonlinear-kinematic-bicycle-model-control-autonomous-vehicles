#pragma once
#include "../types.hpp"

class PurePursuitController
{
public:
    // Constructor to initialize lookahead distance
    PurePursuitController() = default;

    // Compute the desired steering angle using Pure Pursuit algorithm
    inputs computeControlInput(const states &current_state,
                               const inputs &input,
                               const states &ErrorGlobalFrame,
                               const int PathReminingDistance) const
    {
        inputs control_inputs;

        // Extract position errors
        double ex = ErrorGlobalFrame.x;
        double ey = ErrorGlobalFrame.y;

        // Compute distance to target point
        double dist_to_target = sqrt(ex * ex + ey * ey);

        // Prevent division by zero. 10 cm threshold
        if (dist_to_target < 0.1)
            control_inputs.steeringRate = 0.0; // Prevent division by zero
        else
        { // Compute steering command
            double heading = atan2(ey, ex);
            double alpha = heading - current_state.heading; // angle between vehicle heading and line to target
            alpha = atan2(sin(alpha), cos(alpha));          // Normalize to [-pi, pi]
            double curvature = 2 * sin(alpha) / dist_to_target;
            double steering_angle = atan(specs.wheelbase * curvature);
            double error_steering = steering_angle - current_state.steeringAngle;
            control_inputs.steeringRate = std::clamp(kp_steering * error_steering,
                                                     -limits.max_steering_rate,
                                                     limits.max_steering_rate);
            // Compute velocity command
            control_inputs.velocity = computeVelocity(steering_angle, input, PathReminingDistance);
        }
        return control_inputs;
    }

    // Compute desired velocity based on steering angle and path remaining distance
    double computeVelocity(const double steering_angle, const inputs &input, const double PathReminingDistance) const
    {
        // Reduce speed based on steering angle
        double steering_ratio = std::abs(steering_angle) / limits.max_steering_angle;
        double base_velocity = limits.max_velocity * (1 - steering_ratio);

        //  If close to the end of the path, plan to stop
        if (PathReminingDistance <= 50.0)
        {
            double decel_needed = -(input.velocity * input.velocity) / (2.0 * (50.0));
            double decel_velocity = std::max(0.0, input.velocity + decel_needed * specs.dt);
            double curve_velocity = updateVelocity(input.velocity, base_velocity, limits.max_acceleration);
            return std::min(curve_velocity, decel_velocity);
        }
        else
            return updateVelocity(input.velocity, base_velocity, limits.max_acceleration);
    }

    // Update velocity with acceleration limits
    double updateVelocity(const double v_current, const double v_des, const double a_max) const
    {
        double a_cmd = (v_des - v_current) / specs.dt;
        double a = std::clamp(a_cmd, -a_max, a_max);
        double v_cmd = v_current + a * specs.dt;
        v_cmd = std::clamp(v_cmd, limits.min_velocity, limits.max_velocity);

        return v_cmd;
    }

    // Adaptive lookahead distance based on velocity
    double getLd() const
    {
        return Ld;
    }

private:
    double Ld = 3.5;
    double kp_speed = 10.0;
    double kp_steering = 15.0; // Reduced from 20.0 to reduce overshoot

    vehicleLimits limits;           // vehicle limits
    vehicleSpecs specs;            // vehicle specs
};